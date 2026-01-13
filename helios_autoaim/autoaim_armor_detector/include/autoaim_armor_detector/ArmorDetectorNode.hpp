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
#pragma once

// ros
#include <autoaim_interfaces/msg/detail/receive_data__struct.hpp>
#include <cv_bridge/cv_bridge.h>

#include "autoaim_armor_detector/ArmorDetectorFactory.hpp"
#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

// image transport
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>

// opencv
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// interfaces
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/subscription_base.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>

// #include "DetectorFactory.hpp"
#include "autoaim_interfaces/msg/armors.hpp"
#include "autoaim_interfaces/msg/debug_armors.hpp"
#include "autoaim_interfaces/msg/debug_lights.hpp"

// tf2
#include <message_filters/subscriber.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// detectors
#include "autoaim_armor_detector/DetectStream.hpp"
// #include "DetectorFactory.hpp"

#include "autoaim_utilities/Armor.hpp"
#include "autoaim_utilities/PnPSolver.hpp"
#include <cstdint>
#include <ctime>
#include <memory>
#include <visualization_msgs/msg/marker_array.hpp>

#include <armor_detector_parameters.hpp>

namespace helios_cv {

using Params = detector_node::Params;
using ParamListener = detector_node::ParamListener;
using tf2_filter = tf2_ros::MessageFilter<sensor_msgs::msg::Image>;

class ArmorDetectorNode: public rclcpp::Node {
public:
    ArmorDetectorNode(const rclcpp::NodeOptions& options);

    ~ArmorDetectorNode();
    rcl_interfaces::msg::SetParametersResult
    onParametersChanged(const std::vector<rclcpp::Parameter>& params);

private:
    template<typename T>
    void registerParamsUpdateHandle(const std::string& name, T& inside_param);
    void initParameters();
    void initDectors();
    void initInterfaces();
    void initMarker();
    void armorImageCallback(sensor_msgs::msg::Image::UniquePtr image_msg);
    void publishMarkers();

    std::string node_namespace_;
    std::string frame_namespace_;
    rclcpp::Publisher<autoaim_interfaces::msg::Armors>::SharedPtr armors_pub_;
    autoaim_interfaces::msg::Armors armors_msg_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<autoaim_interfaces::msg::ReceiveData>::SharedPtr serial_sub_;
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    std::mutex detector_mutex_;
    std::unique_ptr<BaseDetector> detector_;
    std::unique_ptr<PnPSolver> pnp_solver_;
    std::unique_ptr<ArmorDetectStream> armor_detect_stream_;
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    visualization_msgs::msg::MarkerArray marker_array_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    image_transport::Publisher result_img_pub_;
    rclcpp::Publisher<autoaim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
    rclcpp::Publisher<autoaim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
    void publish_debug_infos();
    Params params_;
    std::unordered_map<std::string, std::function<void(const rclcpp::Parameter&)>>
        params_update_handle_table_;
    std::shared_ptr<ParamListener> param_listener_;
    uint8_t armor_use_traditional_ = false;
    rclcpp::Logger logger_ = rclcpp::get_logger("DetectorNode");
};

} // namespace helios_cv
