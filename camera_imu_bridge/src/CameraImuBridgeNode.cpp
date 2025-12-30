#include <camera_imu_bridge/CameraImuBridgeNode.hpp>
#include <camera_imu_bridge/ImuFactory.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace helios_cv {
CameraImuBridgeNode::CameraImuBridgeNode(const rclcpp::NodeOptions& options):
    Node("camera_imu_bridge_node", options) {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw_test", 10);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    param_listener_ =
        std::make_shared<camera_imu_bridge::ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();
    auto business = extractParams(params_);
    auto cameraName = this->declare_parameter("camera_name", "mv_camera");
    std::unique_ptr<camera_info_manager::CameraInfoManager> cameraInfoManager =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, cameraName);
    auto cameraInfoUrl = this->declare_parameter(
        "camera_info_url",
        "package://camera_imu_bridge/config/camera_info.yaml"
    );
    sensor_msgs::msg::CameraInfo cameraInfoMsg;

    if (cameraInfoManager->validateURL(cameraInfoUrl)) {
        cameraInfoManager->loadCameraInfo(cameraInfoUrl);
        camera_info_msg_ = cameraInfoManager->getCameraInfo();
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", cameraInfoUrl.c_str());
    }

    log_callback_ = makeLogCallback();
    process_and_publish_synced_frame_callback_ = makeProcessAndPublishSyncedFrameCallback();
    syncer_ = std::make_unique<Syncer>(log_callback_, process_and_publish_synced_frame_callback_);
    //ftd2xx中的winTypes和相机的SDK冲突
    imu_ = createImu(
        business.imu_type,
        log_callback_,
        [this](std::shared_ptr<ImuFrame> imu_frame, int frames_since_trigger) {
            syncer_->onImuFrame(imu_frame, frames_since_trigger);
        }
    );
    camera_ =
        std::make_unique<Camera>(log_callback_, [this](std::shared_ptr<CameraFrame> camera_frame) {
            syncer_->onCameraFrame(camera_frame);
        });
    camera_->setParams(business.camera);
}

std::function<void(LogLevel, const std::string&)> CameraImuBridgeNode::makeLogCallback() {
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
CameraImuBridgeNode::makeProcessAndPublishSyncedFrameCallback() {
    return [this](std::shared_ptr<CameraFrame> camera_frame, std::shared_ptr<ImuFrame> imu_frame) {
        if (!camera_) {
            return;
        }
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(camera_info_msg_);
        geometry_msgs::msg::TransformStamped ts1;
        geometry_msgs::msg::TransformStamped ts2;
        auto image_msg_ = std::make_unique<sensor_msgs::msg::Image>();
        ts1.header.stamp = ts2.header.stamp = image_msg_->header.stamp = info_msg->header.stamp =
            this->now();
        ts1.header.frame_id = "odoom";
        ts1.child_frame_id = "yaw_link";
        ts1.transform.translation.x = 0;
        ts1.transform.translation.y = 0;
        ts1.transform.translation.z = 0;
        tf2::Quaternion q1;
        q1.setRPY(0, 0, imu_frame->yaw / 57.2958);
        ts1.transform.rotation = tf2::toMsg(q1);
        transforms.push_back(ts1);

        ts2.header.frame_id = "yaw_link";
        ts2.child_frame_id = "pitch_link";
        ts2.transform.translation.x = 0;
        ts2.transform.translation.y = 0;
        ts2.transform.translation.z = 0;
        tf2::Quaternion q2;
        q2.setRPY(0, imu_frame->pitch / 57.2958, 0);
        ts2.transform.rotation = tf2::toMsg(q2);
        transforms.push_back(ts2);

        image_msg_->header.frame_id = "camera_optical_frame";
        image_msg_->encoding = "rgb8";
        image_msg_->height = camera_frame->head.iHeight;
        image_msg_->width = camera_frame->head.iWidth;
        image_msg_->step = camera_frame->head.iWidth * 3;
        image_msg_->data.resize(camera_frame->head.iWidth * camera_frame->head.iHeight * 3);
        camera_->processFrame(camera_frame, image_msg_->data.data());

        image_pub_->publish(std::move(image_msg_));
        info_pub_->publish(std::move(info_msg));
        tf_broadcaster_->sendTransform(transforms);
    };
}
CameraImuBridgeNode::BusinessParams
CameraImuBridgeNode::extractParams(const camera_imu_bridge::Params& params) {
    BusinessParams business_params;
    business_params.camera.camera_name = params.camera.camera_name;
    business_params.camera.camera_info_url = params.camera.camera_info_url;
    business_params.camera.config_file_path = params.camera.config_file_path;
    business_params.camera.print_fps = params.camera.print_fps;
    business_params.camera.exposure_time = params.camera.exposure_time;
    business_params.camera.analog_gain = params.camera.analog_gain;
    business_params.camera.rgb_gain.r = params.camera.rgb_gain.r;
    business_params.camera.rgb_gain.g = params.camera.rgb_gain.g;
    business_params.camera.rgb_gain.b = params.camera.rgb_gain.b;
    business_params.camera.saturation = params.camera.saturation;
    business_params.camera.gamma = params.camera.gamma;
    business_params.camera.rotate = params.camera.rotate;
    business_params.imu_type = params.imu.type;
    business_params.syncer.camera_imu_time_difference_us.max =
        params.syncer.camera_imu_time_difference_us.max;
    business_params.syncer.camera_imu_time_difference_us.min =
        params.syncer.camera_imu_time_difference_us.min;
    business_params.syncer.imu_frame_since_trigger = params.syncer.imu_frame_since_trigger;
    return business_params;
}
} // namespace helios_cv
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::CameraImuBridgeNode)
