// created by liuhan on 2024/4/27
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

#include <memory>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace helios_cv
{

class AutoaimPriority : public rclcpp::Node
{
public:
  AutoaimPriority(const rclcpp::NodeOptions& options);

  ~AutoaimPriority() = default;

private:
  std::string last_priority_sequence_;

  void priority_callback(std_msgs::msg::String::SharedPtr priority_msg);

  void set_param_callback();

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr priority_sub_;
  rclcpp::TimerBase::SharedPtr timer;

  // params client
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  rclcpp::AsyncParametersClient::SharedPtr armor_predictor_param_client_;
  ResultFuturePtr set_armor_predictor_param_future_;
  bool set_new_flag_ = false;

  std::vector<std::string> new_priority_;

  rclcpp::Logger logger_ = rclcpp::get_logger("AutoaimPriority");
};

}  // namespace helios_cv