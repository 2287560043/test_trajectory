
#include "camera_imu_bridge/Camera.hpp"
#include "camera_imu_bridge/Imu.hpp"
#include "camera_imu_bridge/Syncer.hpp"
#include "camera_imu_bridge/SyncerParams.hpp"
#include "camera_imu_bridge/camera_parameters.hpp"
#include <camera_imu_bridge/CameraFrame.hpp>
#include <camera_imu_bridge/ImuFrame.hpp>
#include <camera_imu_bridge/LogLevel.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
namespace helios_cv {
class CameraImuBridgeNode: public rclcpp::Node {
public:
    CameraImuBridgeNode(const CameraImuBridgeNode&) = delete;
    CameraImuBridgeNode(CameraImuBridgeNode&&) = delete;
    CameraImuBridgeNode& operator=(const CameraImuBridgeNode&) = delete;
    CameraImuBridgeNode& operator=(CameraImuBridgeNode&&) = delete;
    explicit CameraImuBridgeNode(const rclcpp::NodeOptions& options);

private:
    struct BusinessParams {
        CameraParams camera;
        std::string imu_type;
        SyncerParams syncer;
    };
    std::function<void(LogLevel, const std::string&)> makeLogCallback();
    std::function<void(std::shared_ptr<CameraFrame>, std::shared_ptr<ImuFrame>)>
    makeProcessAndPublishSyncedFrameCallback();
    BusinessParams extractParams(const camera_imu_bridge::Params& params);
    std::function<void(std::shared_ptr<CameraFrame>, std::shared_ptr<ImuFrame>)>
        process_and_publish_synced_frame_callback_;
    std::function<void(const LogLevel, const std::string&)> log_callback_;
    std::unique_ptr<Camera> camera_;
    std::unique_ptr<Imu> imu_;
    std::unique_ptr<Syncer> syncer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    std::shared_ptr<camera_imu_bridge::ParamListener> param_listener_;
    camera_imu_bridge::Params params_;
};
} // namespace helios_cv
