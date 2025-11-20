#pragma once

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/parameter.hpp>
#include "gimbal_trajectory/gimbal_trajectory_node_parameters.hpp"
#include <message_filters/subscriber.h>
#include <tf2_ros/create_timer_ros.h>

// opencv
#include <opencv2/core.hpp>

// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

// interfaces
#include <sensor_msgs/msg/camera_info.hpp>

// custom interfaces
#include <autoaim_interfaces/msg/planned_target.hpp>
#include <autoaim_interfaces/msg/target.hpp>
#include <autoaim_interfaces/msg/receive_data.hpp>


namespace helios_cv {

using Params = gimbal_trajectory_node::Params;
using ParamListener = gimbal_trajectory_node::ParamListener;
using tf2_filter = tf2_ros::MessageFilter<autoaim_interfaces::msg::Target>;


class GimbalTrajectoryNode : public rclcpp::Node
{
public:
    explicit GimbalTrajectoryNode(const rclcpp::NodeOptions & options);
    ~GimbalTrajectoryNode();
    
private:
    void gimbal_trajectory_callback(const autoaim_interfaces::msg::Target::SharedPtr target_msg);
    

    autoaim_interfaces::msg::PlannedTarget planned_target_msg_;
    rclcpp::Publisher<autoaim_interfaces::msg::PlannedTarget>::SharedPtr planned_target_pub_;
    message_filters::Subscriber<autoaim_interfaces::msg::Target> target_sub_;

    // tf2
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_filter> tf2_filter_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // camera info
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    cv::Point2f cam_center_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

    // serial info
    rclcpp::Subscription<autoaim_interfaces::msg::ReceiveData>::SharedPtr serial_sub_;
    int last_autoaim_mode_;


    Params params_;
    std::shared_ptr<ParamListener> param_listener_;
};

} // namespace helios_cv