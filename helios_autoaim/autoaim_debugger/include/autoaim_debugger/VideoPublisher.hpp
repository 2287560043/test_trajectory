// created by liuhan on 2023/12/28
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

#include <cv_bridge/cv_bridge.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace helios_cv
{

class VideoPublisher : public rclcpp::Node
{
public:
  VideoPublisher(const rclcpp::NodeOptions& options);

  ~VideoPublisher() = default;

private:
  image_transport::CameraPublisher image_pub_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  std::string video_path_ = "/home/helios/autoaim_ws/src/2025-05-21_16_50_S.avi";
  std::string camera_info_path_ = "package://mindvision_camera/config/camera_info.yaml";

  cv::VideoCapture video_capture_;
  cv::Mat frame_;

  sensor_msgs::msg::CameraInfo camera_info_msg_;
};

}  // namespace helios_cv