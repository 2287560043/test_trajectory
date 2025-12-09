#pragma once

// ros
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/parameter.hpp>
#include "gimbal_planner/gimbal_planner_node_parameters.hpp"
#include <message_filters/subscriber.h>
#include <tf2_ros/create_timer_ros.h>
#include <rclcpp/qos.hpp>

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
#include "../../tinympc/types.hpp"
#include "../../tinympc/tiny_api.hpp"
#include "../../../autoaim_utilities/include/autoaim_utilities/BulletTrajectory.hpp"
#include "../../../autoaim_utilities/include/autoaim_utilities/Target.hpp"
#include "../../../autoaim_utilities/include/autoaim_utilities/Math.hpp"
// #include "../../tinympc/trajectory.hpp"


namespace helios_cv {

using Params = gimbal_planner_node::Params;
using ParamListener = gimbal_planner_node::ParamListener;
using tf2_filter = tf2_ros::MessageFilter<autoaim_interfaces::msg::Target>;

// 以下参数位于BulletTrajectory.hpp
// constexpr double DT = 0.01;
// constexpr int HALF_HORIZON = 50;
// constexpr int HORIZON = HALF_HORIZON * 2;

class GimbalPlannerNode : public rclcpp::Node
{
public:
    explicit GimbalPlannerNode(const rclcpp::NodeOptions & options);
    ~GimbalPlannerNode();
    
private:
    void gimbal_planner_callback(const autoaim_interfaces::msg::Target::SharedPtr target_msg);
    void setup_yaw_planner();
    void setup_pitch_planner();
    Trajectory get_trajectory(const autoaim_interfaces::msg::Target::SharedPtr target_msg, double yaw0);
    

    autoaim_interfaces::msg::PlannedTarget planned_target_msg_;
    rclcpp::Publisher<autoaim_interfaces::msg::PlannedTarget>::SharedPtr planned_target_pub_;
    rclcpp::Subscription<autoaim_interfaces::msg::Target>::SharedPtr target_sub_;

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

    // params
    Params params_;
    std::shared_ptr<ParamListener> param_listener_;

    // logger
    rclcpp::Logger logger_ = rclcpp::get_logger("GimbalPlannerNode");

    // time
    double time_planner_start_ = 0.0;

    // planners
    TinySolver* yaw_planner_;
    TinySolver* pitch_planner_;
    double yaw_offset_;
    double pitch_offset_;
    double fire_thresh_;
    double yaw0_;

    // bullet trajectory
    double bullet_speed_;
    

    double decision_speed_;
    double high_speed_delay_time_;
    double low_speed_delay_time_;

    
};

} // namespace helios_cv