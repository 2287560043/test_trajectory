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
#include "AutoAimDebugger.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/message_filter.h>

#include <autoaim_interfaces/msg/detail/receive_data__struct.hpp>
#include <autoaim_interfaces/msg/detail/target__struct.hpp>
#include <cmath>
#include <cstddef>
#include <functional>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/detail/marker__struct.hpp>

#include "BulletModel.hpp"

namespace helios_cv
{
AutoAimDebugger::AutoAimDebugger(const rclcpp::NodeOptions& options) : rclcpp::Node("autoaim_debugger", options)
{
  // create params
  param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();
  node_namespace_ = this->get_namespace();
  if (node_namespace_.size() == 1)
  {
    node_namespace_.clear();
  }
  last_time_ = this->now();
  bullet_model_ = std::make_shared<BulletModel>(params_.max_bullet_num, params_.air_resistance, 28);
  /// create publishers
  image_pub_ = image_transport::create_publisher(this, node_namespace_ + "/autoaim_debugger/result_img");
  init_object_points();
  /// create subscribers
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      node_namespace_ + "/camera_info", 10, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        distortion_coefficients_ = cv::Mat(1, 5, CV_64FC1, const_cast<double*>(msg->d.data())).clone();
        image_center_ = cv::Point2d(camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
        camera_info_sub_.reset();
      });
  target_sub_.subscribe(this, node_namespace_ + "/predictor/target");
  // receive_data_sub_.subscribe(this, node_namespace_ + "/autoaim_bridge/receive_data");
  receive_data_sub_ = this->create_subscription<autoaim_interfaces::msg::ReceiveData>(
      "/autoaim_bridge/receive_data", rclcpp::SystemDefaultsQoS(),
      [this](autoaim_interfaces::msg::ReceiveData::SharedPtr receive_msg) { receive_data_ = *receive_msg; });
  // init tf2 utilities
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  image_sub_.subscribe(this, node_namespace_ + "/image_raw", rmw_qos_profile_sensor_data);
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  frame_namespace_ = node_namespace_;
  if (!frame_namespace_.empty())
  {
    frame_namespace_.erase(frame_namespace_.begin());
    frame_namespace_ = frame_namespace_ + "_";
  }
  tf2_filter_ = std::make_shared<tf2_filter>(image_sub_, *tf2_buffer_, frame_namespace_ + "camera_optical_frame", 10,
                                             this->get_node_logging_interface(), this->get_node_clock_interface(),
                                             std::chrono::duration<int>(2));
  if (!params_.no_hardware)
  {
    sync_ = std::make_shared<message_filters::Synchronizer<NormalPolicy>>(NormalPolicy(10), image_sub_, target_sub_);
    sync_->registerCallback(&AutoAimDebugger::image_callback, this);
  }
  else
  {
    no_hardware_sync_ = std::make_shared<message_filters::Synchronizer<NoHardwarePolicy>>(NoHardwarePolicy(10),
                                                                                          image_sub_, target_sub_);
    no_hardware_sync_->registerCallback(&AutoAimDebugger::no_hardware_callback, this);
  }
}

void AutoAimDebugger::init_object_points()
{
  double armor_half_y = 135 / 2.0 / 1000.0;
  double armor_half_z = 125 / 2.0 / 1000.0;

  armor_object_points_.emplace_back(cv::Point3f(0, armor_half_y, -armor_half_z));
  armor_object_points_.emplace_back(cv::Point3f(0, armor_half_y, armor_half_z));
  armor_object_points_.emplace_back(cv::Point3f(0, -armor_half_y, armor_half_z));
  armor_object_points_.emplace_back(cv::Point3f(0, -armor_half_y, -armor_half_z));

  armor_half_y = 0.430 / 2.0;
  armor_half_z = 0.370 / 2.0;
  energy_object_points_.emplace_back(cv::Point3f(0, armor_half_y, -armor_half_z));
  energy_object_points_.emplace_back(cv::Point3f(0, armor_half_y, armor_half_z));
  energy_object_points_.emplace_back(cv::Point3f(0, -armor_half_y, armor_half_z));
  energy_object_points_.emplace_back(cv::Point3f(0, -armor_half_y, -armor_half_z));
  energy_object_points_.emplace_back(cv::Point3f(0, 0, -0.7));

  // give a random value for radius, we only need the centre point
  bullet_object_points_.emplace_back(cv::Point3f(0, 0.025, 0));
  bullet_object_points_.emplace_back(cv::Point3f(0, -0.025, 0));
  bullet_object_points_.emplace_back(cv::Point3f(0, 0, 0.025));
  bullet_object_points_.emplace_back(cv::Point3f(0, 0, -0.025));
}

void AutoAimDebugger::no_hardware_callback(sensor_msgs::msg::Image::SharedPtr msg,
                                           autoaim_interfaces::msg::Target::SharedPtr target_msg)
{
  // Update params
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_WARN(this->get_logger(), "Update params");
  }
  /// Convert image
  try
  {
    raw_image_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  /// Check camera info
  if (camera_matrix_.empty() || distortion_coefficients_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "camera matrix or distortion coefficients is empty");
    return;
  }
  // Get transform infomation
  try
  {
    cam2odom_ = tf2_buffer_->lookupTransform("camera_optical_frame", "odoom", msg->header.stamp,
                                             rclcpp::Duration::from_seconds(0.01));
    odom2cam_ = tf2_buffer_->lookupTransform("odoom", "camera_optical_frame", msg->header.stamp,
                                             rclcpp::Duration::from_seconds(0.01));
    gimbal2odom_ =
        tf2_buffer_->lookupTransform("odoom", "pitch_link", msg->header.stamp, rclcpp::Duration::from_seconds(0.01));
  }
  catch (tf2::LookupException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "tf2 exception: %s", e.what());
    return;
  }
  caculate_bullets(target_msg);
  caculate_target(target_msg);
  // Draw image center
  cv::circle(raw_image_, image_center_, 5, cv::Scalar(0, 0, 255), 2);
  image_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, raw_image_).toImageMsg());
}

void AutoAimDebugger::image_callback(sensor_msgs::msg::Image::SharedPtr msg,
                                     autoaim_interfaces::msg::Target::SharedPtr target_msg)
{
  // Update params
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_WARN(this->get_logger(), "Update params");
  }
  /// Convert image
  try
  {
    raw_image_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  /// Check camera info
  if (camera_matrix_.empty() || distortion_coefficients_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "camera matrix or distortion coefficients is empty");
    return;
  }
  // Get transform infomation
  try
  {
    cam2odom_ = tf2_buffer_->lookupTransform("camera_optical_frame", "odoom", msg->header.stamp,
                                             rclcpp::Duration::from_seconds(0.1));
    odom2cam_ = tf2_buffer_->lookupTransform("odoom", "camera_optical_frame", msg->header.stamp,
                                             rclcpp::Duration::from_seconds(0.1));
    gimbal2odom_ =
        tf2_buffer_->lookupTransform("pitch_link", "odoom", msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  }
  catch (tf2::LookupException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "tf2 exception: %s", e.what());
    return;
  }
  caculate_bullets(target_msg);
  caculate_target(target_msg);
  draw_predicted_points();
  // Draw image center
  cv::circle(raw_image_, image_center_, 5, cv::Scalar(0, 0, 255), 2);
  image_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, raw_image_).toImageMsg());
}

void AutoAimDebugger::draw_predicted_points()
{
    // RCLCPP_WARN(logger_, "draw_predicted_points");
  /// Draw Prediction Point
  geometry_msgs::msg::Point point;
  point.x = receive_data_.x;
  point.y = receive_data_.y;
  point.z = receive_data_.z;
  try
  {
    tf2::doTransform(point, point, cam2odom_);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "tf2 exception: %s", e.what());
    return;
  }
  cv::Mat_<double> predict_point = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
  predict_point = (camera_matrix_ / predict_point.at<double>(2)) * predict_point;
  cv::circle(raw_image_, cv::Point2f(predict_point.at<double>(0), predict_point.at<double>(1)), 5,
             cv::Scalar(255, 0, 0), 2);
}

// void AutoAimDebugger::caculate_target(autoaim_interfaces::msg::Target::SharedPtr target_msg)
// {
//   if (target_msg->tracking)
//   {
//     target_pose_ros_.clear();
//     target_rvecs_.clear();
//     target_tvecs_.clear();
//     if (target_msg->armors_num == 1)
//     {
//       // armor only
//       geometry_msgs::msg::Pose pose;
//       tf2::Quaternion tf_q;
//       tf_q.setRPY(0, target_msg->id == "outpost" ? -0.26 : 0.26, 0);
//       pose.orientation = tf2::toMsg(tf_q);
//       pose.position = target_msg->position;
//       target_pose_ros_.emplace_back(pose);
//     }
//     else if (target_msg->armors_num >= 2 && target_msg->armors_num <= 4)
//     {
//       // target observer
//       double yaw = target_msg->yaw, r1 = target_msg->radius_1, r2 = target_msg->radius_2;
//       double xc = target_msg->position.x, yc = target_msg->position.y, zc = target_msg->position.z;
//       double vxc = target_msg->velocity.x, vyc = target_msg->velocity.y, vzc = target_msg->velocity.z,
//              vyaw = target_msg->v_yaw;
//       double dz = target_msg->dz;

//       bool is_current_pair = true;
//       size_t a_n = target_msg->armors_num;
//       geometry_msgs::msg::Point p_a;
//       double r = 0;
//       for (size_t i = 0; i < a_n; i++)
//       {
//         double tmp_yaw = yaw + i * (2 * M_PI / a_n);
//         // Only 4 armors has 2 radius and height
//         if (a_n == 4)
//         {
//           r = is_current_pair ? r1 : r2;
//           p_a.z = zc + (is_current_pair ? 0 : dz);
//           is_current_pair = !is_current_pair;
//         }
//         else
//         {
//           r = r1;
//           p_a.z = zc;
//         }
//         p_a.x = xc - r * std::cos(tmp_yaw);
//         p_a.y = yc - r * std::sin(tmp_yaw);
//         geometry_msgs::msg::Pose pose;

//         pose.position = p_a;
//         tf2::Quaternion q;
//         q.setRPY(0, target_msg->id == "outpost" ? -0.26 : 0.26, tmp_yaw);
//         pose.orientation = tf2::toMsg(q);

//         // Get target armor corners
//         target_pose_ros_.emplace_back(pose);
//       }
//     }
//     else if (target_msg->armors_num == 5)
//     {
//       // energy target
//       double yaw = target_msg->yaw, roll = target_msg->v_yaw;
//       double r = target_msg->radius_1;
//       double a = target_msg->velocity.x, w = target_msg->velocity.y, phi = target_msg->velocity.z;
//       double xc = target_msg->position.x, yc = target_msg->position.y, zc = target_msg->position.z;
//       // Energy Fan
//       size_t a_n = target_msg->armors_num;
//       geometry_msgs::msg::Point p_a;
//       for (size_t i = 0; i < a_n; i++)
//       {
//         double tmp_roll = roll + i * (2 * M_PI / a_n);
//         p_a.x = xc - r * std::sin(-tmp_roll) * std::sin(yaw);
//         p_a.y = yc + r * std::sin(-tmp_roll) * std::cos(yaw);
//         p_a.z = zc + r * std::cos(-tmp_roll);
//         geometry_msgs::msg::Pose pose;
//         pose.position = p_a;
//         tf2::Quaternion q;
//         q.setRPY(tmp_roll, 0, yaw);
//         pose.orientation = tf2::toMsg(q);
//         target_pose_ros_.emplace_back(pose);
//       }
//     }
//     // transform armors to camera optical frame
//     try
//     {
//       for (auto& pose : target_pose_ros_)
//       {
//         tf2::doTransform(pose, pose, cam2odom_);
//       }
//     }
//     catch (tf2::TransformException& e)
//     {
//       RCLCPP_ERROR(this->get_logger(), "tf2 exception: %s", e.what());
//       return;
//     }
//     /// Draw target armors
//     // convert ros pose to cv pose
//     for (auto& pose : target_pose_ros_)
//     {
//       cv::Mat tvec = (cv::Mat_<double>(3, 1) << pose.position.x, pose.position.y, pose.position.z);
//       target_tvecs_.emplace_back(tvec);
//       cv::Quatd q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
//       target_rvecs_.emplace_back(q);
//     }
//     if (target_msg->armors_num == 5)
//     {
//       for (std::size_t i = 0; i < target_tvecs_.size(); i++)
//       {
//         std::vector<cv::Point2f> image_points;
//         cv::projectPoints(energy_object_points_, target_rvecs_[i].toRotMat3x3(), target_tvecs_[i], camera_matrix_,
//                           distortion_coefficients_, image_points);
//         cv::putText(raw_image_, std::to_string(i), image_points[2], cv::FONT_HERSHEY_SIMPLEX, 0.8,
//                     cv::Scalar(0, 255, 255), 2);
//         for (size_t i = 0; i < image_points.size(); i++)
//         {
//           cv::line(raw_image_, image_points[i], image_points[(i + 1) % image_points.size()], cv::Scalar(0, 0, 255),
//                    cv::LINE_4);
//         }
//       }
//     }
//     else
//     {
//       for (std::size_t i = 0; i < target_tvecs_.size(); i++)
//       {
//         std::vector<cv::Point2f> image_points;
//         cv::projectPoints(armor_object_points_, target_rvecs_[i].toRotMat3x3(), target_tvecs_[i], camera_matrix_,
//                           distortion_coefficients_, image_points);
//         cv::putText(raw_image_, std::to_string(i), image_points[2], cv::FONT_HERSHEY_SIMPLEX, 0.8,
//                     cv::Scalar(0, 255, 255), 2);
//         for (size_t i = 0; i < image_points.size(); i++)
//         {
//           cv::line(raw_image_, image_points[i], image_points[(i + 1) % image_points.size()], cv::Scalar(0, 0, 255),
//                    cv::LINE_4);
//         }
//       }
//     }
//   }
// }

// [AutoAimDebugger.cpp]
void AutoAimDebugger::caculate_target(autoaim_interfaces::msg::Target::SharedPtr target_msg)
{
  if (target_msg->tracking)
  {
    target_pose_ros_.clear();
    target_rvecs_.clear();
    target_tvecs_.clear();

    size_t armors_num = target_msg->armors_num;
    
    if (armors_num >= 2 && armors_num <= 4)
    {
      double yaw = target_msg->yaw;
      double r1 = target_msg->radius_1; 
      double r2 = target_msg->radius_2;
      double xc = target_msg->position.x;
      double yc = target_msg->position.y;
      double zc = target_msg->position.z;
      double dz = target_msg->dz;

      bool is_current_pair = true;
      double r = 0;
      double z_offset = 0;

      for (size_t i = 0; i < armors_num; i++)
      {
        // 计算每个装甲板的 Yaw 角
        double tmp_yaw = yaw + i * (2 * M_PI / armors_num);
        
        // 处理 4 板结构的大小板半径和高度差 (如平衡步兵/前哨站可能不同，标准车辆一般 r1=r2)
        if (armors_num == 4)
        {
          r = is_current_pair ? r1 : r2;
          z_offset = is_current_pair ? 0 : dz;
          is_current_pair = !is_current_pair;
        }
        else
        {
          r = r1;
          z_offset = 0;
        }

        geometry_msgs::msg::Point p_a;
        p_a.x = xc - r * std::cos(tmp_yaw);
        p_a.y = yc - r * std::sin(tmp_yaw);
        p_a.z = zc + z_offset;

        geometry_msgs::msg::Pose pose;
        pose.position = p_a;
        
        // 设置装甲板朝向
        tf2::Quaternion q;
        // Outpost 有特定的倾斜角，其他车辆默认为 15 度 (0.26 rad)
        q.setRPY(0, target_msg->id == "outpost" ? -0.26 : 0.26, tmp_yaw);
        pose.orientation = tf2::toMsg(q);

        target_pose_ros_.emplace_back(pose);
      }
    }
    else if (target_msg->armors_num == 1) // 只有1个板的情况（罕见，通常用于调试）
    {
       geometry_msgs::msg::Pose pose;
       tf2::Quaternion tf_q;
       tf_q.setRPY(0, target_msg->id == "outpost" ? -0.26 : 0.26, 0);
       pose.orientation = tf2::toMsg(tf_q);
       pose.position = target_msg->position;
       target_pose_ros_.emplace_back(pose);
    }
    // 能量机关 (5板) 逻辑保持不变，此处省略，如果需要请保留原文件中的 else if (target_msg->armors_num == 5) ...

    // 2. 坐标变换：Odom -> Camera Optical Frame
    try
    {
      for (auto& pose : target_pose_ros_)
      {
        tf2::doTransform(pose, pose, cam2odom_);
      }
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_ERROR(this->get_logger(), "tf2 exception: %s", e.what());
      return;
    }

    // 3. 投影到图像并绘制
    // 为了确定哪个是“击打目标”，我们计算哪个装甲板中心距离图像中心最近
    int best_target_idx = -1;
    double min_dist_sq = DBL_MAX;
    std::vector<cv::Point2f> centers_2d; // 存储所有板的2D中心

    for (size_t i = 0; i < target_pose_ros_.size(); i++)
    {
      cv::Mat tvec = (cv::Mat_<double>(3, 1) << target_pose_ros_[i].position.x, 
                                               target_pose_ros_[i].position.y, 
                                               target_pose_ros_[i].position.z);
      cv::Quatd q(target_pose_ros_[i].orientation.w, target_pose_ros_[i].orientation.x, 
                  target_pose_ros_[i].orientation.y, target_pose_ros_[i].orientation.z);
      
      target_tvecs_.emplace_back(tvec);
      target_rvecs_.emplace_back(q);

      // 简单的中心点投影用于距离判断
      std::vector<cv::Point3f> center_3d = {cv::Point3f(0,0,0)};
      std::vector<cv::Point2f> center_2d_vec;
      cv::projectPoints(center_3d, q.toRotMat3x3(), tvec, camera_matrix_, distortion_coefficients_, center_2d_vec);
      
      if (!center_2d_vec.empty()) {
        centers_2d.push_back(center_2d_vec[0]);
        // 计算距离图像中心的欧氏距离平方
        double dist_sq = std::pow(center_2d_vec[0].x - image_center_.x, 2) + 
                         std::pow(center_2d_vec[0].y - image_center_.y, 2);
        // 同时要保证装甲板在相机前方 (z > 0)
        if (target_pose_ros_[i].position.z > 0 && dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          best_target_idx = i;
        }
      } else {
        centers_2d.push_back(cv::Point2f(-1, -1));
      }
    }

    // 4. 正式绘制
    for (std::size_t i = 0; i < target_tvecs_.size(); i++)
    {
      // 过滤掉相机后方的点
      if (target_tvecs_[i].at<double>(2) <= 0) continue;

      std::vector<cv::Point2f> image_points;
      // 根据装甲板类型选择 3D 点模型 (小板/大板)
      // 注意：target_msg 中有 armor_type，但这里为了简化，使用初始化好的 armor_object_points_
      // 严谨做法是根据 target_msg->armor_type 切换 points
      cv::projectPoints(armor_object_points_, target_rvecs_[i].toRotMat3x3(), target_tvecs_[i], 
                        camera_matrix_, distortion_coefficients_, image_points);

      // 设置颜色：击打目标为红色/洋红，其他为蓝色/青色
      cv::Scalar line_color = (int)i == best_target_idx ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 0); // BGR
      int thickness = (int)i == best_target_idx ? 3 : 1;

      // 绘制四边形
      for (size_t j = 0; j < image_points.size(); j++)
      {
        cv::line(raw_image_, image_points[j], image_points[(j + 1) % image_points.size()], 
                 line_color, thickness);
      }
      
      // 绘制中心点和标签
      if (i < centers_2d.size()) {
          cv::circle(raw_image_, centers_2d[i], 4, line_color, -1);
          // 标记 "Hit"
          if ((int)i == best_target_idx) {
             cv::putText(raw_image_, "HIT", centers_2d[i] + cv::Point2f(10, -10), 
                         cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
          }
      }
    }
  }
}

void AutoAimDebugger::caculate_bullets(autoaim_interfaces::msg::Target::SharedPtr target_msg)
{
  double dt = (this->now() - last_time_).seconds();
  last_time_ = this->now();
  // Get yaw and pitch from gimbal2odom
  tf2::Quaternion q(gimbal2odom_.transform.rotation.x, gimbal2odom_.transform.rotation.y,
                    gimbal2odom_.transform.rotation.z, gimbal2odom_.transform.rotation.w);
  double yaw, pitch, roll;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  // RCLCPP_WARN(logger_, "yaw %f pitch %f roll %f", yaw, pitch, roll);
  bullet_model_->update_params(params_.air_resistance, receive_data_.bullet_speed);
  if (target_msg->tracking)
  {
    bullet_model_->add_new_bullet(
        yaw, -pitch, Eigen::Vector3d{ target_msg->position.x, target_msg->position.y, target_msg->position.z });
  }
  bullet_model_->update_bullets_status(dt);
  bullet_tvecs_ = bullet_model_->get_bullet_positions_in_imu();
  // transform bullets to camera optical frame
  std::vector<cv::Quatd> bullet_rvecs;

  try
  {
    for (auto& tvec : bullet_tvecs_)
    {
      geometry_msgs::msg::Pose point;
      point.orientation.w = 1;
      point.orientation.x = 0;
      point.orientation.y = 0;
      point.orientation.z = 0;
      point.position.x = tvec.at<double>(0, 0);
      point.position.y = tvec.at<double>(1, 0);
      point.position.z = tvec.at<double>(2, 0);
      tf2::doTransform(point, point, cam2odom_);
      tvec.at<double>(0, 0) = point.position.x;
      tvec.at<double>(1, 0) = point.position.y;
      tvec.at<double>(2, 0) = point.position.z;
      bullet_rvecs.emplace_back(
          cv::Quatd{ point.orientation.w, point.orientation.x, point.orientation.y, point.orientation.z });
    }
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_ERROR(logger_, "catch %s in bullet solve", e.what());
  }

  // for (int i = 0; i < bullet_tvecs_.size(); i++)
  //     RCLCPP_WARN(logger_, "X: %f, Y: %f, Z: %f", bullet_tvecs_[i].at<double>(0, 0), bullet_tvecs_[i].at<double>(1,
  //     0), bullet_tvecs_[i].at<double>(2, 0));

  // Draw Bullet tracks
  for (std::size_t i = 1; i < bullet_tvecs_.size(); i++)
  {
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(bullet_object_points_, bullet_rvecs[i].toRotMat3x3(), bullet_tvecs_[i], camera_matrix_,
                      distortion_coefficients_, image_points);
    cv::Point2f bullet_center = (image_points[0] + image_points[1] + image_points[2] + image_points[3]) / 4;
    // caculate radius
    double radius = cv::norm(bullet_center - image_points[0]);
    if (radius < 2)
    {
      radius = 2;
    }

    if (std::isnan(bullet_center.x) || std::isnan(bullet_center.y) || 
    std::isinf(bullet_center.x) || std::isinf(bullet_center.y)) {
    // RCLCPP_WARN(logger_, "Invalid bullet_center coordinates: x=%f, y=%f", bullet_center.x, bullet_center.y);
    continue; 
    }

    if (bullet_center.x < 0 || bullet_center.y < 0 || 
    bullet_center.x >= raw_image_.cols || bullet_center.y >= raw_image_.rows) {
    // RCLCPP_WARN(logger_, "Bullet center outside image: x=%f, y=%f, img_size=%dx%d", 
    //           bullet_center.x, bullet_center.y, raw_image_.cols, raw_image_.rows);
    continue; 
    }

    cv::circle(raw_image_, bullet_center, radius, cv::Scalar(255, 0, 0), 4);
    // RCLCPP_WARN(logger_, "bullet x %f, y %f", bullet_center.x, bullet_center.y);
    // RCLCPP_WARN(logger_, "radius %f", radius);
    // cv::putText(raw_image_, std::to_string(bullet_distance_[i]), image_points[2], cv::FONT_HERSHEY_SIMPLEX, 0.8,
    // cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace helios_cv

// register node to component
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::AutoAimDebugger);