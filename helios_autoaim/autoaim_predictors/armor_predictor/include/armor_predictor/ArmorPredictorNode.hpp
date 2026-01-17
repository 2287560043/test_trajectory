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
#include <message_filters/subscriber.h>
#include <rclcpp/subscription_base.hpp>

// tf2
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

// interfaces
#include <visualization_msgs/msg/marker_array.hpp>
#include "autoaim_interfaces/msg/receive_data.hpp"
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>

// custom
#include "armor_predictor/BaseObserver.hpp"
#include "armor_predictor/OutpostObserver.hpp"
#include "armor_predictor/StandardObserver.hpp"


#include <armor_predictor/armor_predictor_node_parameters.hpp>


namespace helios_cv
{

constexpr double SYSTEM_LATENCY_COMPENSATION = 0.05;

using tf2_filter = tf2_ros::MessageFilter<autoaim_interfaces::msg::Armors>;
using ParamListener = armor_predictor_node::ParamListener;
using Params = armor_predictor_node::Params;

class ArmorPredictorNode : public rclcpp::Node
{
public:
  ArmorPredictorNode(const rclcpp::NodeOptions& options);

  ~ArmorPredictorNode();

private:
  void armor_predictor_callback(autoaim_interfaces::msg::Armors::SharedPtr armors_msg);

  // time series
  double time_predictor_start_;

  double last_target_distance_;

  TargetType last_target_type_;
  std::shared_ptr<BaseObserver> vehicle_observer_;
  void init_predictors();

  std::string node_namespace_;
  std::string frame_namespace_;
  int gimbal_id_ = 0;
  autoaim_interfaces::msg::Target target_msg_;

  rclcpp::Publisher<autoaim_interfaces::msg::Target>::SharedPtr target_pub_;
  // tf2
  // Subscriber with tf2 message_filter
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<autoaim_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

  // Serial info part
  rclcpp::Subscription<autoaim_interfaces::msg::ReceiveData>::SharedPtr serial_sub_;

  // parameter utilities
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  void update_predictor_params();

  void update_predictor_type(std::shared_ptr<BaseObserver>& vehicle_observer);

  uint8_t last_autoaim_mode_ = 0;
  double yaw_ = 0.0;
  double bullet_speed_ = 0.0;
  double gimbal_yaw_ = 0.0;
  
  // debug info
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker target_armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray target_marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr target_marker_pub_;
  void get_marker_array(autoaim_interfaces::msg::Target target);
  void create_visualization_markers();
  void check_and_kill_invalid(const autoaim_interfaces::msg::Target& target_msg);

  rclcpp::Logger logger_ = rclcpp::get_logger("ArmorPredictorNode");
};

}  // namespace helios_cv