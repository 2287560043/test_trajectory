// created by liuhan on 2024/3/28
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */
#include "CtrlBridge.hpp"

#include <memory>
#include <sys/types.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <vector>

#include "Packets.hpp"
#include "autoaim_interfaces/msg/receive_data.hpp"
#include "ctrl_bridge_parameters.hpp"

namespace helios_cv
{

CtrlBridge::CtrlBridge(const rclcpp::NodeOptions& options) : Node("ctrl_bridge", options)
{
  // create params
  param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();
  // create serial
  RCLCPP_DEBUG(logger_, "Create Serial");
  try
  {
    serial_ = std::make_shared<Serial>(params_.serial_name, params_.serial_baudrate);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s", params_.serial_name.c_str(), ex.what());
    exit(-1);
  }
  // create publisher
  mcu_publisher_ =
      this->create_publisher<autoaim_interfaces::msg::ReceiveData>("receive_data", rclcpp::SensorDataQoS());
  // create tf broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  RCLCPP_DEBUG(logger_, "Register Callback And Publisher");
  register_callbacks();
  // create subscriptions
  target_info_sub_ = this->create_subscription<autoaim_interfaces::msg::Target>(
      "predictor/target", rclcpp::SensorDataQoS(),
      std::bind(&CtrlBridge::target_info_callback, this, std::placeholders::_1));
  RCLCPP_DEBUG(logger_, "Start Spin Serial");
  serial_->spin();
}

void CtrlBridge::register_callbacks()
{
  serial_->register_publisher(RECEIVE_AUTOAIM_RECEIVE_CMD_ID,
                              std::make_shared<PublisherImpl<autoaim_interfaces::msg::ReceiveData>>(mcu_publisher_));
  serial_->register_callback(RECEIVE_AUTOAIM_RECEIVE_CMD_ID, [this](const std::vector<uint8_t>& data) {
    auto mcu_info = fromVector<MCUPacket>(data);
    auto msg = std::make_shared<autoaim_interfaces::msg::ReceiveData>();
    recv_packet_ = mcu_info;
    msg->autoaim_mode = mcu_info.autoaim_mode;
    msg->bullet_speed = mcu_info.bullet_speed;
    msg->x = mcu_info.x;
    msg->y = mcu_info.y;
    msg->z = mcu_info.z;
    msg->yaw = -mcu_info.yaw;
    msg->pitch = mcu_info.pitch;
    msg->roll = mcu_info.roll;
    msg->target_color = !mcu_info.self_color;
    msg->header.stamp = this->now() + rclcpp::Duration::from_seconds(params_.time_stamp_offset);
    serial_->publish<autoaim_interfaces::msg::ReceiveData>(msg, 0x107);
    RCLCPP_DEBUG_ONCE(logger_, "First Publish Message %d", RECEIVE_AUTOAIM_RECEIVE_CMD_ID);
    // Send tf2 transform
    // publish transform of left gimbal
    geometry_msgs::msg::TransformStamped ts;
    ts.header.frame_id = "odoom";
    ts.child_frame_id = "yaw_link";
    ts.header.stamp = msg->header.stamp;
    tf2::Quaternion q;
    q.setRPY(0, 0, -mcu_info.yaw);
    ts.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(ts);
    ts.header.frame_id = "yaw_link", ts.child_frame_id = "pitch_link";
    ts.header.stamp = msg->header.stamp;
    q.setRPY(0, -mcu_info.pitch, 0);
    ts.transform.rotation = tf2::toMsg(q);
    ts.transform.translation.x = params_.left_yaw2pitch_translation;
    tf_broadcaster_->sendTransform(ts);
    ts.transform.translation.x = 0;
    // RCLCPP_INFO(logger_, "roll = %.2lf, pitch = %.2lf, yaw=%.2lf",msg->roll,msg->pitch,msg->yaw );
    RCLCPP_DEBUG_ONCE(logger_, "First Send Transform");
    // RCLCPP_WARN(logger_, "TEST");
  });
}

void CtrlBridge::target_info_callback(autoaim_interfaces::msg::Target::SharedPtr target_msg)
{
  RCLCPP_DEBUG_ONCE(logger_, "First Target Info Callback Begin");
  // clang-format off
  static const std::map<std::string, uint8_t> id_unit8_map{
      {"", 0},  {"outpost", 0}, {"1", 1}, 
      {"2", 2}, {"3", 3},       {"4", 4}, 
      {"5", 5}, {"guard", 6}, {"base", 7}};
  // clang-format on
  // Copy msg
  target_info_.gimbal_id = target_msg->gimbal_id;
  target_info_.id = id_unit8_map.at(target_msg->id);
  target_info_.tracking = target_msg->tracking;
  target_info_.armors_num = target_msg->armors_num;
  target_info_.x = target_msg->position.x;
  target_info_.y = target_msg->position.y;
  target_info_.z = target_msg->position.z;
  target_info_.yaw = target_msg->yaw;
  target_info_.vx = target_msg->velocity.x;
  target_info_.vy = target_msg->velocity.y;
  target_info_.vz = target_msg->velocity.z;
  target_info_.v_yaw = target_msg->v_yaw;
  target_info_.r1 = target_msg->radius_1;
  target_info_.r2 = target_msg->radius_2;
  target_info_.dz = target_msg->dz;
  target_info_.vision_delay = (this->now() - target_msg->header.stamp).seconds();
  RCLCPP_INFO(logger_, "Vision Delay: %.6f ms", target_info_.vision_delay * 1000);
  // Push into write fifo
  serial_->write(target_info_, SEND_TARGET_INFO_CMD_ID);
  RCLCPP_DEBUG_ONCE(logger_, "First Target Info Callback End");
}

CtrlBridge::~CtrlBridge()
{
  serial_->close();
  serial_.reset();
}

}  // namespace helios_cv

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::CtrlBridge)
