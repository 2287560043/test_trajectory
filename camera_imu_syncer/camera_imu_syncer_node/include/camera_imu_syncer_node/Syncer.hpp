#pragma once
#include "CameraDefine.h"
#include "Imu.hpp"
#include "camera_imu_syncer_interfaces/msg/camera_imu_sync.hpp"
#include <atomic>
#include <camera_info_manager/camera_info_manager.hpp>
#include <chrono>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>

#include <CameraApi.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <thread>
struct HI91Data;
#pragma pack(push, 1)
struct HI91Data {
    uint8_t tag;
    uint16_t status;
    int8_t temperature;
    float air_pressure;
    uint32_t system_time;
    float acc_b_x, acc_b_y, acc_b_z;
    float gyr_b_x, gyr_b_y, gyr_b_z;
    float mag_b_x, mag_b_y, mag_b_z;
    float roll, pitch, yaw;
    float q_w, q_x, q_y, q_z;
};
#pragma pack(pop)

class Syncer {
public:
    struct CameraInfo {
        std::chrono::high_resolution_clock::time_point time;
        uint64_t id;
        int64_t targetTime;
        tSdkFrameHead pHead;
        BYTE* pBuffer;
        CameraInfo(tSdkFrameHead* _pHead, BYTE* _pBuffer, int64_t _targetTime):
            time(std::chrono::high_resolution_clock::now()),
            targetTime(_targetTime),
            pHead(*_pHead) {
            auto size = _pHead->uBytes;
            pBuffer = new BYTE[size];
            std::memcpy(pBuffer, _pBuffer, size);
        }
        ~CameraInfo() {
            delete[] pBuffer;
        }
    };

    struct ImuInfo {
        std::chrono::high_resolution_clock::time_point time;
        uint64_t id;
        uint8_t tag;
        uint16_t status;
        int8_t temperature;
        float air_pressure;
        uint32_t system_time;
        float acc_b_x, acc_b_y, acc_b_z;
        float gyr_b_x, gyr_b_y, gyr_b_z;
        float mag_b_x, mag_b_y, mag_b_z;
        float roll, pitch, yaw;
        float q_w, q_x, q_y, q_z;
        int64_t targetTime;

        ImuInfo(const HI91Data& _HI91Data, int64_t _targetTime):
            time(std::chrono::high_resolution_clock::now()),
            tag(_HI91Data.tag),
            status(_HI91Data.status),
            temperature(_HI91Data.temperature),
            air_pressure(_HI91Data.air_pressure),
            system_time(_HI91Data.system_time),
            acc_b_x(_HI91Data.acc_b_x),
            acc_b_y(_HI91Data.acc_b_y),
            acc_b_z(_HI91Data.acc_b_z),
            gyr_b_x(_HI91Data.gyr_b_x),
            gyr_b_y(_HI91Data.gyr_b_y),
            gyr_b_z(_HI91Data.gyr_b_z),
            mag_b_x(_HI91Data.mag_b_x),
            mag_b_y(_HI91Data.mag_b_y),
            mag_b_z(_HI91Data.mag_b_z),
            roll(_HI91Data.roll),
            pitch(_HI91Data.pitch),
            yaw(_HI91Data.yaw),
            q_w(_HI91Data.q_w),
            q_x(_HI91Data.q_x),
            q_y(_HI91Data.q_y),
            q_z(_HI91Data.q_z),
            targetTime(_targetTime) {}
    };
    Syncer(
        rclcpp::Publisher<camera_imu_syncer_interfaces::msg::CameraImuSync>::SharedPtr _pub,
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _imagePub,
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _infoPub,

        rclcpp::Clock::SharedPtr _clock,
        std::shared_ptr<tf2_ros::TransformBroadcaster> _tfBroadcaster,
        sensor_msgs::msg::CameraInfo _cameraInfoMsg

    );
    ~Syncer();
    void addCameraFrame(std::shared_ptr<CameraInfo> cameraInfo);
    void addIMUFrame(std::shared_ptr<Syncer::ImuInfo> imuInfo);
    CameraHandle hCamera;

private:
    std::atomic<uint64_t> cameraId = 0;
    std::atomic<uint64_t> imuId = 0;
    std::atomic<int> offset = -5;
    std::atomic<bool> stable { false };
    std::shared_ptr<CameraInfo> cameraInfos[10];
    std::shared_ptr<ImuInfo> imuInfos[10];
    rclcpp::Publisher<camera_imu_syncer_interfaces::msg::CameraImuSync>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub;
    rclcpp::Clock::SharedPtr clock;
    sensor_msgs::msg::Image::UniquePtr imageMsg;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    std::thread sendMsg;
    std::atomic<bool> running = true;
    sensor_msgs::msg::CameraInfo cameraInfoMsg;

    std::atomic<uint16_t> cameraImuIndex = 65535;
};
