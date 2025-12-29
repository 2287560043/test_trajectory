#include <camera_imu_bridge/Camera.hpp>
#include <camera_imu_bridge/CameraFrame.hpp>
#include <camera_imu_bridge/Imu.hpp>
#include <camera_imu_bridge/ImuFactory.hpp>
#include <camera_imu_bridge/ImuFrame.hpp>
#include <camera_imu_bridge/LogLevel.hpp>
#include <camera_imu_bridge/Syncer.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>

namespace helios_cv {
class CameraImuBridgeNode: public rclcpp::Node {
public:
    explicit CameraImuBridgeNode(const rclcpp::NodeOptions& options):
        Node("camera_imu_bridge_node", options) {
        log_callback_ = makeLogCallback();
        process_and_publish_synced_frame_callback_ = makeProcessAndPublishSyncedFrameCallback();
        syncer_ =
            std::make_unique<Syncer>(log_callback_, process_and_publish_synced_frame_callback_);
        imu_ = createImu(
            "Ftd2xx",
            log_callback_,
            [this](std::shared_ptr<ImuFrame> imu_frame, int frames_since_trigger) {
                syncer_->onImuFrame(imu_frame, frames_since_trigger);
            }
        );
        camera_ = std::make_unique<Camera>(
            log_callback_,
            [this](std::shared_ptr<CameraFrame> camera_frame) {
                syncer_->onCameraFrame(camera_frame);
            }
        );
    }

private:
    std::function<void(LogLevel, const std::string&)> makeLogCallback() {
        return [this](const LogLevel loglevel, const std::string& msg) {
            switch (loglevel) {
                case LogLevel::Info: {
                    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
                    break;
                }
                case LogLevel::Warn: {
                    RCLCPP_WARN(get_logger(), "%s", msg.c_str());
                    break;
                }
                case LogLevel::Error: {
                    RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
                    break;
                }
            }
        };
    }
    std::function<void(std::shared_ptr<CameraFrame>, std::shared_ptr<ImuFrame>)>
    makeProcessAndPublishSyncedFrameCallback() {
        return
            [this](std::shared_ptr<CameraFrame> camera_frame, std::shared_ptr<ImuFrame> imu_frame) {
                if (!camera_) {
                    return;
                }
            };
    }
    std::function<void(std::shared_ptr<CameraFrame>, std::shared_ptr<ImuFrame>)>
        process_and_publish_synced_frame_callback_;
    std::function<void(const LogLevel, const std::string&)> log_callback_;
    std::unique_ptr<Camera> camera_;
    std::unique_ptr<Imu> imu_;
    std::unique_ptr<Syncer> syncer_;
    sensor_msgs::msg::CameraInfo camera_info_msg;
};
} // namespace helios_cv
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::CameraImuBridgeNode)
