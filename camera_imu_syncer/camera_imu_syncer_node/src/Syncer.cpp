#include "Syncer.hpp"
#include "camera_imu_syncer_interfaces/msg/camera_imu_sync.hpp"
#include <CameraApi.h>
#include <CameraDefine.h>
#include <cstdlib>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <thread>
using CameraImuMsg = camera_imu_syncer_interfaces::msg::CameraImuSync;
Syncer::~Syncer() {
    running = false;
    stable = false;
    cameraImuIndex = 65534;
    cameraImuIndex.notify_all();
    sendMsg.join();
}
Syncer::Syncer(
    rclcpp::Publisher<camera_imu_syncer_interfaces::msg::CameraImuSync>::SharedPtr _pub,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _imagePub,
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _infoPub,
    rclcpp::Clock::SharedPtr _clock,
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tfBroadcaster,
    sensor_msgs::msg::CameraInfo _cameraInfoMsg

):
    pub(_pub),
    imagePub(_imagePub),
    infoPub(_infoPub),
    clock(_clock),
    tfBroadcaster(_tfBroadcaster),
    cameraInfoMsg(_cameraInfoMsg) {
    sendMsg = std::thread { [this]() {
        uint16_t cameraImuIndexOld = 65535;

        while (running) {
            cameraImuIndex.wait(cameraImuIndexOld);

            if (stable) {
                cameraImuIndexOld = cameraImuIndex;
                int cameraIndex = (static_cast<uint8_t>(cameraImuIndexOld >> 8) + 0) % 10;
                int imuIndex = static_cast<uint8_t>(cameraImuIndexOld & 0xFF);

                std::vector<geometry_msgs::msg::TransformStamped> transforms;

                auto infoMsg = std::make_unique<sensor_msgs::msg::CameraInfo>(cameraInfoMsg);
                geometry_msgs::msg::TransformStamped ts1;
                geometry_msgs::msg::TransformStamped ts2;
                imageMsg = std::make_unique<sensor_msgs::msg::Image>();
                //gu yi jia 10ms
                // ts1.header.stamp = time + rclcpp::Duration(0, 10000000);

                ts1.header.stamp = ts2.header.stamp = imageMsg->header.stamp =
                    infoMsg->header.stamp = clock->now();
                ts1.header.frame_id = "odoom";
                ts1.child_frame_id = "yaw_link";
                ts1.transform.translation.x = 0;
                ts1.transform.translation.y = 0;
                ts1.transform.translation.z = 0;
                tf2::Quaternion q1;
                q1.setRPY(0, 0, imuInfos[imuIndex]->yaw / 57.2958);
                ts1.transform.rotation = tf2::toMsg(q1);
                transforms.push_back(ts1);

                ts2.header.frame_id = "yaw_link";
                ts2.child_frame_id = "pitch_link";
                ts2.transform.translation.x = 0;
                ts2.transform.translation.y = 0;
                ts2.transform.translation.z = 0;
                tf2::Quaternion q2;
                q2.setRPY(0, imuInfos[imuIndex]->pitch / 57.2958, 0);
                ts2.transform.rotation = tf2::toMsg(q2);
                transforms.push_back(ts2);

                tfBroadcaster->sendTransform(transforms);
                imageMsg->header.frame_id = "camera_optical_frame";
                imageMsg->encoding = "rgb8";
                imageMsg->height = cameraInfos[cameraIndex]->pHead.iHeight;
                imageMsg->width = cameraInfos[cameraIndex]->pHead.iWidth;
                imageMsg->step = cameraInfos[cameraIndex]->pHead.iWidth * 3;
                imageMsg->data.resize(
                    cameraInfos[cameraIndex]->pHead.iWidth * cameraInfos[cameraIndex]->pHead.iHeight
                    * 3
                );
                CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_RGB8);
                CameraImageProcessEx(
                    hCamera,
                    cameraInfos[cameraIndex]->pBuffer,
                    imageMsg->data.data(),
                    &cameraInfos[cameraIndex]->pHead,
                    CAMERA_MEDIA_TYPE_BGR8,
                    0
                );

                imagePub->publish(std::move(imageMsg));
                infoPub->publish(std::move(infoMsg));
                std::cout << "\n  ├─ IMU Time:    "
                          << Imu::ms_to_utc_time(imuInfos[imuIndex]->targetTime)
                          << "\n  ├─ Camera Time: "
                          << Imu::ms_to_utc_time(cameraInfos[cameraIndex]->targetTime);

                // std::cout << "offset: " << offset << ","
                //           << " cameraId: " << cameraInfos[cameraIndex]->id << ","
                //           << " imuId: " << imuInfos[imuIndex]->id << ","<<"newest cameraId: "<<cameraId<<",newest imuId: "<<imuId
                //           << " imu-camera time difference(ms):"
                //           << std::chrono::duration_cast<std::chrono::nanoseconds>(
                //                  cameraInfos[cameraIndex]->time - imuInfos[imuIndex]->time
                //              )
                //                  .count()
                //         / (double)1000000
                //           << std::endl;
            }
        }
    } };
}

void Syncer::addCameraFrame(std::shared_ptr<CameraInfo> cameraInfo) {
    cameraInfo->id = cameraId;
    cameraInfos[cameraId % 10] = cameraInfo;
    if (stable) {
        if (imuInfos[(cameraId - offset) % 10]->id == cameraId - offset) {
            uint8_t cameraIndex = cameraId % 10;
            uint8_t imuIndex = (cameraId - offset) % 10;
            cameraImuIndex = (static_cast<uint16_t>(cameraIndex) << 8) | imuIndex;
            cameraImuIndex.notify_one();
        }
        if (cameraId % 1000 == 0) {
            int timeSum = 0;
            int n = 0;

            for (int i = 0; i < 10; ++i) {
                if (imuInfos[(cameraInfos[i]->id - offset) % 10]
                    && imuInfos[(cameraInfos[i]->id - offset) % 10]->id
                        == cameraInfos[i]->id - offset)
                {
                    timeSum += std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   cameraInfos[i]->time
                                   - imuInfos[(cameraInfos[i]->id - offset) % 10]->time
                    )
                                   .count();
                    ++n;
                }
            }
            if (n != 0 && (timeSum / n) < 10000000 && (timeSum / n) > 1000000) {
            } else {
                std::cerr << "Delay happened" << std::endl;
                cameraId = 0;
                imuId = 0;
                stable = false;
                offset = -5;
            }
        }

    } else {
        if (cameraId % 10 == 0 && imuId > 10) {
            while (true) {
                std::cout << "offset: " << offset << std::endl;
                int timeSum = 0;
                int n = 0;
                for (int i = 0; i < 10; ++i) {
                    if (imuInfos[(cameraInfos[i]->id - offset) % 10]
                        && imuInfos[(cameraInfos[i]->id - offset) % 10]->id
                            == cameraInfos[i]->id - offset)
                    {
                        timeSum += std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       cameraInfos[i]->time
                                       - imuInfos[(cameraInfos[i]->id - offset) % 10]->time
                        )
                                       .count();
                        ++n;
                    }
                }
                if (n != 0 && (timeSum / n) < 10000000 && (timeSum / n) > 1000000) {
                    std::cout << timeSum / n << std::endl;
                    stable = true;
                    break;
                }
                if (offset > 5) {
                    std::cerr << "No suitable offset matched" << std::endl;
                    break;
                }
                ++offset;
            }
        } else {
            std::cout << "cameraId: " << cameraId << " imuId: " << imuId << std::endl;
        }
    }
    if (cameraId > imuId && cameraId - imuId > 5) {
        cameraId = 0;
        imuId = 0;
        stable = false;
        offset = -5;
        std::cerr << "Imu data missing" << std::endl;
    }
    if (cameraId < imuId && imuId - cameraId > 5) {
        cameraId = 0;
        imuId = 0;
        stable = false;
        offset = -5;
        std::cerr << "Camera data missing" << std::endl;
    }
    ++cameraId;
}

void Syncer::addIMUFrame(std::shared_ptr<Syncer::ImuInfo> imuInfo) {
    imuInfo->id = imuId;
    imuInfos[imuId % 10] = imuInfo;
    ++imuId;
}
