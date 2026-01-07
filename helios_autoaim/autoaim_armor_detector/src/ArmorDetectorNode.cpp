// created by liuhan on 2023/10/29
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */

#include <autoaim_armor_detector/ArmorDetectorNode.hpp>

#include "autoaim_armor_detector/ArmorDetectorFactory.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoaim_armor_detector/DetectStream.hpp>
#include <autoaim_utilities/Armor.hpp>
#include <autoaim_utilities/PnPSolver.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <exception>
#include <memory>
#include <opencv2/core.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace helios_cv {

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options):
    rclcpp::Node("armor_detector_node", options) {
    initParameters();
    initDectors();
    initInterfaces();
    initMarker();
}

void ArmorDetectorNode::initParameters() {
    param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();
    armor_use_traditional_ = params_.armor.use_traditional;
    node_namespace_ = this->get_namespace();
    if (node_namespace_.size() == 1) {
        node_namespace_.clear();
    }
    frame_namespace_ = node_namespace_;
    if (!frame_namespace_.empty()) {
        frame_namespace_.erase(frame_namespace_.begin());
        frame_namespace_ = frame_namespace_ + "_";
    }
}

void ArmorDetectorNode::initInterfaces() {
    result_img_pub_ =
        image_transport::create_publisher(this, node_namespace_ + "/detector/result_img");
    armors_pub_ = this->create_publisher<autoaim_interfaces::msg::Armors>(
        node_namespace_ + "/detector/armors",
        10
    );
    marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

    auto image_qos = rclcpp::QoS(1)
                         .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                         .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        node_namespace_ + "/camera_info",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::UniquePtr camera_info) {
            camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
            pnp_solver_ = std::make_unique<ArmorProjectYaw>(camera_info->k, camera_info->d);
            pnp_solver_->set_use_projection(params_.use_projection);
            armor_detect_stream_ =
                std::make_unique<ArmorDetectStream>(std::move(detector_), std::move(pnp_solver_));
            camera_info_sub_.reset();
        }
    );

    serial_sub_ = this->create_subscription<autoaim_interfaces::msg::ReceiveData>(
        "/receive_data",
        rclcpp::SensorDataQoS(),
        [this](autoaim_interfaces::msg::ReceiveData::SharedPtr mcu_packet) {
            if (params_.autoaim_mode != mcu_packet->autoaim_mode) {
                rclcpp::Parameter param("autoaim_mode", mcu_packet->autoaim_mode);
                this->set_parameter(param);
            }
            if (params_.is_blue != mcu_packet->target_color) {
                rclcpp::Parameter param("is_blue", mcu_packet->target_color);
                this->set_parameter(param);
            }
        }
    );
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        node_namespace_ + "/image_raw",
        image_qos,
        [this](sensor_msgs::msg::Image::UniquePtr image_msg) {
            armorImageCallback(std::move(image_msg));
        }
    );

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
    );
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
}

void ArmorDetectorNode::initDectors() {
    // create factory
    detector_factory_ = std::make_unique<DetectorFactory>();
    // create detectors, default create armor detector
    if (params_.armor.use_traditional) {
        detector_ = std::move(detector_factory_->createTraditionalArmorDetector(params_));
    } else {
        detector_ = std::move(detector_factory_->createOvnetArmorDetector(params_));
    }
}
void ArmorDetectorNode::initMarker() {
    armor_marker_.ns = "armors";
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.05;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.g = 0.5;
    armor_marker_.color.b = 1.0;
    armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    text_marker_.ns = "classification";
    text_marker_.action = visualization_msgs::msg::Marker::ADD;
    text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker_.scale.z = 0.1;
    text_marker_.color.a = 1.0;
    text_marker_.color.r = 1.0;
    text_marker_.color.g = 1.0;
    text_marker_.color.b = 1.0;
    text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
}

void ArmorDetectorNode::armorImageCallback(sensor_msgs::msg::Image::UniquePtr image_msg) {
    if (!armor_detect_stream_) {
        return;
    }
    // convert image msg to cv::Mat
    auto image_header = image_msg->header;

    try {
        // image_ = std::move(
        //     cv_bridge::toCvShare(std::move(image_msg), sensor_msgs::image_encodings::RGB8)->image
        // );
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
        return;
    }
    // Get transform
    geometry_msgs::msg::TransformStamped ts_odom2cam, ts_cam2odom;
    double yaw;

    if (pnp_solver_ != nullptr) {
        armors_msg_.header = armor_marker_.header = text_marker_.header = image_header;

        armors_msg_.armors.clear();
        marker_array_.markers.clear();
        armor_marker_.id = 0;
        text_marker_.id = 0;
    }

    try {
        ts_odom2cam = tf2_buffer_->lookupTransform(
            frame_namespace_ + "camera_optical_frame",
            frame_namespace_ + "odoom",
            //timestamp from image
            image_header.stamp
        );
        ts_cam2odom = tf2_buffer_->lookupTransform(
            frame_namespace_ + "odoom",
            frame_namespace_ + "camera_optical_frame",
            image_header.stamp
        );
        auto odom2yawlink = tf2_buffer_->lookupTransform(
            frame_namespace_ + "yaw_link",
            frame_namespace_ + "odoom",
            image_header.stamp
        );
        tf2::Quaternion q(
            odom2yawlink.transform.rotation.x,
            odom2yawlink.transform.rotation.y,
            odom2yawlink.transform.rotation.z,
            odom2yawlink.transform.rotation.w
        );
        tf2::Matrix3x3 m(q);
        // blank roll and pitch, not using them
        double roll, pitch;
        m.getRPY(roll, pitch, yaw);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
        return;
    }
    // detect

    ArmorTransformInfo armor_transform_info { ros2cv(ts_odom2cam.transform.rotation),
                                              ros2cv(ts_cam2odom.transform.rotation),
                                              yaw };
    if (params_.armor.use_traditional) {
        // auto input_image = std::make_shared<Image>(std::move(image_));
        // armors_msg_ = armor_detect_stream_->detect(input_image, &armor_transform_info);
        // armors_msg_.header = image_header;
    } else {
        // auto input_image = std::make_shared<ImageStamped>(std::move(image_), image_header.stamp);
        // armors_msg_ = armor_detect_stream_->detect(input_image, &armor_transform_info);
        // armors_msg_.header.frame_id = image_header.frame_id;
    }

    // publish
    armors_pub_->publish(armors_msg_);
    rclcpp::Time now = this->get_clock()->now();
    RCLCPP_INFO(
        logger_,
        "time cost: %.2f ms",
        (now - rclcpp::Time(image_header.stamp)).seconds() * 1000.0
    );
    // debug info
    if (params_.debug) {
        publish_debug_infos();
        publishMarkers();
    }
}

void ArmorDetectorNode::publishMarkers() {
    using Marker = visualization_msgs::msg::Marker;
    for (const auto& armor: armors_msg_.armors) {
        armor_marker_.id++;
        armor_marker_.scale.y = armor.type == 0 ? 0.135 : 0.23;
        armor_marker_.pose = armor.pose;
        text_marker_.id++;
        text_marker_.pose.position = armor.pose.position;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.number;
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);
    }
    armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
    marker_array_.markers.emplace_back(armor_marker_);
    marker_pub_->publish(marker_array_);
}

void ArmorDetectorNode::publish_debug_infos() {
    try {
        // Armor mode
        result_img_pub_.publish(
            cv_bridge::CvImage(
                std_msgs::msg::Header(),
                sensor_msgs::image_encodings::RGB8,
                armor_detect_stream_->get_debug_images()
            )
                .toImageMsg()
        );
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(
            logger_,
            *this->get_clock(),
            1000,
            "Error in publish_debug_infos: %s",
            e.what()
        );
    }
}

ArmorDetectorNode::~ArmorDetectorNode() {
    result_img_pub_.shutdown();
    detector_.reset();
    pnp_solver_.reset();
    detector_factory_.reset();
    param_listener_.reset();
    RCLCPP_INFO(logger_, "DetectorNode destructed");
}

} // namespace helios_cv

// register node to component
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::ArmorDetectorNode);
