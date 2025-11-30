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

#include "VideoPublisher.hpp"

#include <cstdlib>
#include <image_transport/image_transport.hpp>
#include <list>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <thread>

namespace helios_cv
{

VideoPublisher::VideoPublisher(const rclcpp::NodeOptions& options) : rclcpp::Node("video_publisher", options)
{
  // Create publisher
  image_pub_ = image_transport::create_camera_publisher(this, "/image_raw");
  // Get camera info
  camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "video");
  camera_info_manager_->loadCameraInfo(camera_info_path_);
  camera_info_msg_ = camera_info_manager_->getCameraInfo();
  // Read video
  video_capture_.open(video_path_);
  if (!video_capture_.isOpened())
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot open video file");
    exit(0);
  }
  std::list<int> a;
  // Loop
  std::thread([this]() -> void {
    RCLCPP_WARN(this->get_logger(), "start");
    rclcpp::Rate rate(40);
    while (rclcpp::ok())
    {
      video_capture_ >> frame_;
      if (frame_.empty())
      {
        RCLCPP_INFO(this->get_logger(), "Video end");
        exit(0);
      }
      sensor_msgs::msg::Image::SharedPtr msg =
          cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, frame_).toImageMsg();
      msg->header.frame_id = "camera_optical_frame";
      msg->header.stamp = this->now();
      camera_info_msg_.header = msg->header;
      image_pub_.publish(*msg, camera_info_msg_);
      rate.sleep();
    }
  }).detach();
}

}  // namespace helios_cv

// register node to component
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::VideoPublisher);