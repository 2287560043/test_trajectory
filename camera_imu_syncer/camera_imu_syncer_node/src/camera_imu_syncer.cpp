#include "Camera.hpp"
#include "Imu.hpp"
#include "Syncer.hpp"
#include "camera_imu_syncer_interfaces/msg/camera_imu_sync.hpp"
#include <camera_info_manager/camera_info_manager.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

using CameraImuMsg = camera_imu_syncer_interfaces::msg::CameraImuSync;
class CameraImuSyncerNode: public rclcpp::Node {
public:
    explicit CameraImuSyncerNode(const rclcpp::NodeOptions& options):
        Node("camera_imu_syncer_node", options) {
        pub = this->create_publisher<CameraImuMsg>("camera_imu_data", 3);
        imagePub = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        infoPub = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        auto cameraName = this->declare_parameter("camera_name", "mv_camera");
        std::unique_ptr<camera_info_manager::CameraInfoManager> cameraInfoManager =
            std::make_unique<camera_info_manager::CameraInfoManager>(this, cameraName);
        auto cameraInfoUrl = this->declare_parameter(
            "camera_info_url",
            "package://camera_imu_syncer/config/camera_info.yaml"
        );
        sensor_msgs::msg::CameraInfo cameraInfoMsg;

        if (cameraInfoManager->validateURL(cameraInfoUrl)) {
            cameraInfoManager->loadCameraInfo(cameraInfoUrl);
            cameraInfoMsg = cameraInfoManager->getCameraInfo();
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", cameraInfoUrl.c_str());
        }

        syncer = std::make_shared<Syncer>(
            pub,
            imagePub,
            infoPub,
            this->get_clock(),
            tfBroadcaster,
            cameraInfoMsg
        );
        camera = std::make_shared<Camera>(syncer);
        imu = std::make_shared<Imu>(syncer);
        if (!imu->open("/dev/ttyUSB0")) {
            throw std::runtime_error("IMU open failed");
        }
    }

private:
    std::shared_ptr<Syncer> syncer;
    std::shared_ptr<Camera> camera;
    std::shared_ptr<Imu> imu;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub;
    rclcpp::Publisher<CameraImuMsg>::SharedPtr pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<CameraImuSyncerNode>(rclcpp::NodeOptions());
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
