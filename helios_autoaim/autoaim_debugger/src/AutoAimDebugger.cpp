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
  // detection_image_pub_ = image_transport::create_publisher(this, node_namespace_ + "/autoaim_debugger/detection_img");
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
  armors_sub_.subscribe(this, node_namespace_ + "/detector/armors", rclcpp::SensorDataQoS().get_rmw_qos_profile());
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
  // subscriber - 硬触发模式下不需要tf2_filter，直接订阅
  image_sub_.subscribe(this, node_namespace_ + "/image_raw", rmw_qos_profile_sensor_data);

  if (!params_.no_hardware)
  {
    sync_ = std::make_shared<message_filters::Synchronizer<NormalPolicy>>(NormalPolicy(10), image_sub_, target_sub_, armors_sub_);
    sync_->registerCallback(&AutoAimDebugger::image_callback, this);
  }
  else
  {
    no_hardware_sync_ = std::make_shared<message_filters::Synchronizer<NoHardwarePolicy>>(NoHardwarePolicy(10),
                                                                                          image_sub_, target_sub_, armors_sub_);
    no_hardware_sync_->registerCallback(&AutoAimDebugger::no_hardware_callback, this);
  }

  // 参数回调
  auto param_change_callback = [this](const std::vector<rclcpp::Parameter>& parameters) {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto& param : parameters) {
      RCLCPP_INFO(this->get_logger(), "Parameter changed: %s", param.get_name().c_str());
    }

    // 更新参数
    if (param_listener_->is_old(params_)) {
      params_ = param_listener_->get_params();
      bullet_model_->update_params(params_.air_resistance, receive_data_.bullet_speed);
      RCLCPP_INFO(this->get_logger(), "Parameters updated");
    }

    return result;
  };

  param_callback_ = this->add_on_set_parameters_callback(param_change_callback);
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
                                           autoaim_interfaces::msg::Target::SharedPtr target_msg,
                                           autoaim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  /// Convert image
  try
  {
    raw_image_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8)->image.clone();
    // Convert RGB to BGR for OpenCV processing
    cv::cvtColor(raw_image_, raw_image_, cv::COLOR_RGB2BGR);
    // 为检测图像创建副本
    detection_image_ = raw_image_.clone();
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
    // gimbal2odom_ =
    //     tf2_buffer_->lookupTransform("imu_link", "odoom", msg->header.stamp, rclcpp::Duration::from_seconds(0.01));
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "tf2 exception: %s", e.what());
    return;
  }

  // 在检测图像上只绘制检测相关内容（检测角点+重投影）
  draw_detected_armors(armors_msg, detection_image_);
  cv::circle(detection_image_, image_center_, 5, cv::Scalar(0, 0, 255), 2);

  // 在完整调试图像上绘制跟踪预测内容（不包含检测，避免重复）
  // caculate_bullets(target_msg);
  caculate_target(target_msg);
  cv::circle(raw_image_, image_center_, 5, cv::Scalar(0, 0, 255), 2);

  // 发布两个图像
  image_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, raw_image_).toImageMsg());
  // detection_image_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, detection_image_).toImageMsg());
}

void AutoAimDebugger::image_callback(sensor_msgs::msg::Image::SharedPtr msg,
                                     autoaim_interfaces::msg::Target::SharedPtr target_msg,
                                     autoaim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  /// Convert image
  try
  {
    raw_image_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8)->image.clone();
    // Convert RGB to BGR for OpenCV processing
    cv::cvtColor(raw_image_, raw_image_, cv::COLOR_RGB2BGR);
    // 为检测图像创建副本
    detection_image_ = raw_image_.clone();
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
                                             rclcpp::Duration::from_seconds(1.0));
    odom2cam_ = tf2_buffer_->lookupTransform("odoom", "camera_optical_frame", msg->header.stamp,
                                             rclcpp::Duration::from_seconds(1.0));
  //   gimbal2odom_ =
  //       tf2_buffer_->lookupTransform("imu_link", "odoom", msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "tf2 exception: %s", e.what());
    return;
  }

  // 在检测图像上只绘制检测相关内容（检测角点+重投影）
  // draw_detected_armors(armors_msg, detection_image_);
  // cv::circle(detection_image_, image_center_, 5, cv::Scalar(0, 0, 255), 2);

  // 在完整调试图像上绘制跟踪预测内容（不包含检测，避免重复）
  // caculate_bullets(target_msg);
  caculate_target(target_msg);
  draw_predicted_points();
  cv::circle(raw_image_, image_center_, 5, cv::Scalar(0, 0, 255), 2);

  // 发布两个图像
  image_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, raw_image_).toImageMsg());
  // detection_image_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, detection_image_).toImageMsg());
}

void AutoAimDebugger::draw_predicted_points()
{
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
void AutoAimDebugger::caculate_target(autoaim_interfaces::msg::Target::SharedPtr target_msg)
{
  if (target_msg->tracking)
  {
    target_pose_ros_.clear();
    
    // ==========================================
    // 1. 定义装甲板 3D 顶点 (透视变换的基础)
    // ==========================================
    // 重点：在 ROS Body Frame (X前 Y左 Z上) 中，
    // 我们定义板子面朝 X 轴，所以顶点分布在 Y-Z 平面上 (x=0)。
    
    // 简单判断大小装甲板 (英雄 1 是大装甲，其他默认为小)
    bool is_large = (target_msg->id == "1"); 
    double w = is_large ? 0.23 : 0.135; // 宽: 大230mm, 小135mm
    double h = 0.055;                   // 高: 55mm
    
    // 逆时针定义四个角点: 左上 -> 左下 -> 右下 -> 右上
    std::vector<cv::Point3f> object_points;
    object_points.emplace_back(0,  w/2,  h/2);
    object_points.emplace_back(0,  w/2, -h/2);
    object_points.emplace_back(0, -w/2, -h/2);
    object_points.emplace_back(0, -w/2,  h/2);

    // ==========================================
    // 2. 构建位姿与旋转
    // ==========================================
    geometry_msgs::msg::Pose pose;
    pose.position = target_msg->position; 

    tf2::Quaternion q;
    // 恢复 15 度倾角 (约 0.26 rad)
    // 规则：Yaw 决定朝向，Pitch 决定倾斜。
    // tf2 setRPY(r, p, y) 对应 Fixed Axis Z-Y-X 旋转，
    // 即先 Yaw(Z) 转到朝向，再 Pitch(Y) 仰起，符合物理直觉。
    double pitch = (target_msg->id == "outpost") ? -0.26 : 0.26; 
    
    q.setRPY(0, pitch, target_msg->yaw);
    pose.orientation = tf2::toMsg(q);

    // 转换到相机坐标系
    try {
      tf2::doTransform(pose, pose, cam2odom_);
    } catch (tf2::TransformException& e) {
      RCLCPP_WARN(rclcpp::get_logger("AutoAimDebugger"), "Transform Error: %s", e.what());
      return;
    }

    target_pose_ros_.emplace_back(pose);

    // ==========================================
    // 3. 投影 (3D -> 2D)
    // ==========================================
    auto& target_pose = target_pose_ros_[0];

    cv::Mat tvec = (cv::Mat_<double>(3, 1) << target_pose.position.x, 
                                             target_pose.position.y, 
                                             target_pose.position.z);
    
    cv::Quatd q_eigen(target_pose.orientation.w, target_pose.orientation.x, 
                      target_pose.orientation.y, target_pose.orientation.z);
    
    // 剔除相机后方的点
    if (tvec.at<double>(2) <= 0.1) return;

    std::vector<cv::Point2f> image_points;

    // 使用刚才定义的 object_points 进行投影，这样画出来的框才有透视感
    cv::projectPoints(object_points, q_eigen.toRotMat3x3(), tvec, 
                      camera_matrix_, distortion_coefficients_, image_points);

    std::vector<cv::Point3f> center_3d = {cv::Point3f(0, 0, 0)};
    std::vector<cv::Point2f> center_2d;
    cv::projectPoints(center_3d, q_eigen.toRotMat3x3(), tvec, 
                      camera_matrix_, distortion_coefficients_, center_2d);

    // ==========================================
    // 4. 绘制 (保持原逻辑)
    // ==========================================
    cv::Scalar hit_color = cv::Scalar(0, 0, 255);
    
    // 画出装甲板的四边形 (现在是平行四边形/梯形了)
    for (size_t j = 0; j < image_points.size(); j++)
    {
      cv::line(raw_image_, image_points[j], image_points[(j + 1) % image_points.size()], 
               hit_color, 4);
    }

    if (!center_2d.empty()) {
        cv::Point2f c = center_2d[0];
        cv::line(raw_image_, c - cv::Point2f(10, 0), c + cv::Point2f(10, 0), hit_color, 4);
        cv::line(raw_image_, c - cv::Point2f(0, 10), c + cv::Point2f(0, 10), hit_color, 4);
        cv::circle(raw_image_, c, 5, hit_color, 2);

        double dist = std::sqrt(target_msg->position.x * target_msg->position.x + 
                                target_msg->position.y * target_msg->position.y + 
                                target_msg->position.z * target_msg->position.z);
        
        std::string info_text = "Dist: " + std::to_string(dist).substr(0, 4) + "m";
        cv::putText(raw_image_, info_text, cv::Point2f(150, 210), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.2, hit_color, 2);
        
        if (!target_msg->id.empty()) {
            cv::putText(raw_image_, "ID: " + target_msg->id, cv::Point2f(150, 150), 
                        cv::FONT_HERSHEY_SIMPLEX, 1.2, hit_color, 2);
        }
        cv::putText(raw_image_, "Yaw: " + std::to_string(target_msg->yaw).substr(0, 4), cv::Point2f(150, 90), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.2, hit_color, 2);
        cv::putText(raw_image_, "Vel_yaw: " + std::to_string(target_msg->v_yaw).substr(0, 4), cv::Point2f(150, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.2, hit_color, 2);
    }
  }
}

// void AutoAimDebugger::caculate_bullets(autoaim_interfaces::msg::Target::SharedPtr target_msg)
// {
//   double dt = (this->now() - last_time_).seconds();
//   last_time_ = this->now();
//   // Get yaw and pitch from gimbal2odom
//   tf2::Quaternion q(gimbal2odom_.transform.rotation.x, gimbal2odom_.transform.rotation.y,
//                     gimbal2odom_.transform.rotation.z, gimbal2odom_.transform.rotation.w);
//   double yaw, pitch, roll;
//   tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
//   bullet_model_->update_params(params_.air_resistance, receive_data_.bullet_speed);
//   if (target_msg->tracking)
//   {
//     bullet_model_->add_new_bullet(
//         yaw, -pitch, Eigen::Vector3d{ target_msg->position.x, target_msg->position.y, target_msg->position.z });
//   }
//   bullet_model_->update_bullets_status(dt);
//   bullet_tvecs_ = bullet_model_->get_bullet_positions_in_imu();
//   // transform bullets to camera optical frame
//   std::vector<cv::Quatd> bullet_rvecs;

//   try
//   {
//     for (auto& tvec : bullet_tvecs_)
//     {
//       geometry_msgs::msg::Pose point;
//       point.orientation.w = 1;
//       point.orientation.x = 0;
//       point.orientation.y = 0;
//       point.orientation.z = 0;
//       point.position.x = tvec.at<double>(0, 0);
//       point.position.y = tvec.at<double>(1, 0);
//       point.position.z = tvec.at<double>(2, 0);
//       tf2::doTransform(point, point, cam2odom_);
//       tvec.at<double>(0, 0) = point.position.x;
//       tvec.at<double>(1, 0) = point.position.y;
//       tvec.at<double>(2, 0) = point.position.z;
//       bullet_rvecs.emplace_back(
//           cv::Quatd{ point.orientation.w, point.orientation.x, point.orientation.y, point.orientation.z });
//     }
//   }
//   catch (tf2::TransformException& e)
//   {
//     RCLCPP_ERROR(logger_, "catch %s in bullet solve", e.what());
//   }

//   // Draw Bullet tracks
//   for (std::size_t i = 1; i < bullet_tvecs_.size(); i++)
//   {
//     std::vector<cv::Point2f> image_points;
//     cv::projectPoints(bullet_object_points_, bullet_rvecs[i].toRotMat3x3(), bullet_tvecs_[i], camera_matrix_,
//                       distortion_coefficients_, image_points);
//     cv::Point2f bullet_center = (image_points[0] + image_points[1] + image_points[2] + image_points[3]) / 4;
//     // caculate radius
//     double radius = cv::norm(bullet_center - image_points[0]);
//     if (radius < 2)
//     {
//       radius = 2;
//     }

//     if (std::isnan(bullet_center.x) || std::isnan(bullet_center.y) ||
//         std::isinf(bullet_center.x) || std::isinf(bullet_center.y)) {
//       continue;
//     }

//     if (bullet_center.x < 0 || bullet_center.y < 0 ||
//         bullet_center.x >= raw_image_.cols || bullet_center.y >= raw_image_.rows) {
//       continue;
//     }

//     cv::circle(raw_image_, bullet_center, radius, cv::Scalar(255, 0, 0), 4);
//   }
// }

// void AutoAimDebugger::draw_detected_armors(autoaim_interfaces::msg::Armors::SharedPtr armors_msg, cv::Mat& target_image)
// {
//   if (!armors_msg || armors_msg->armors.empty())
//   {
//     return;
//   }

//   // Draw detected armors from detector
//   for (const auto& armor : armors_msg->armors)
//   {
//     // 装甲板四个角点（检测结果）
//     // 顺序：左下、左上、右上、右下
//     std::vector<cv::Point2f> armor_corners = {
//       cv::Point2f(armor.corners[0].x, armor.corners[0].y),  // 左下
//       cv::Point2f(armor.corners[1].x, armor.corners[1].y),  // 左上
//       cv::Point2f(armor.corners[2].x, armor.corners[2].y),  // 右上
//       cv::Point2f(armor.corners[3].x, armor.corners[3].y)   // 右下
//     };

//     // 1. 绘制灯条（高亮显示，粗线）
//     // 左灯条：左下 → 左上（青色粗线）
//     cv::line(target_image, armor_corners[0], armor_corners[1], cv::Scalar(255, 255, 0), 4);
//     // 右灯条：右下 → 右上（青色粗线）
//     cv::line(target_image, armor_corners[3], armor_corners[2], cv::Scalar(255, 255, 0), 4);

//     // 2. 绘制装甲板上下边框（绿色细线）
//     cv::line(target_image, armor_corners[1], armor_corners[2], cv::Scalar(0, 255, 0), 1);  // 上边
//     cv::line(target_image, armor_corners[0], armor_corners[3], cv::Scalar(0, 255, 0), 1);  // 下边

//     // 3. 绘制灯条端点（绿色圆圈）
//     for (const auto& pt : armor_corners)
//     {
//       cv::circle(target_image, pt, 5, cv::Scalar(0, 255, 0), -1);
//     }

//     // 4. 绘制PnP重投影点（红色）
//     // Detector发布的armor pose已经在相机坐标系下，直接使用即可
//     cv::Mat tvec = (cv::Mat_<double>(3, 1) << armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);
//     cv::Quatd q(armor.pose.orientation.w, armor.pose.orientation.x, armor.pose.orientation.y, armor.pose.orientation.z);

//     // Project armor corners (使用3D物理尺寸)
//     std::vector<cv::Point2f> projected_points;
//     cv::projectPoints(armor_object_points_, q.toRotMat3x3(), tvec, camera_matrix_,
//                       distortion_coefficients_, projected_points);

//     // 绘制重投影轮廓（红色）
//     for (size_t i = 0; i < projected_points.size(); i++)
//     {
//       cv::line(target_image, projected_points[i], projected_points[(i + 1) % projected_points.size()],
//                cv::Scalar(0, 0, 255), 2);
//     }

//     // 绘制重投影角点（红色）
//     for (const auto& pt : projected_points)
//     {
//       cv::circle(target_image, pt, 3, cv::Scalar(0, 0, 255), -1);
//     }

//     // 5. 绘制装甲板编号和距离信息（黄色）
//     float distance_cm = tvec.at<double>(2) * 100.0;  // 米转厘米
//     cv::Point2f center = (armor_corners[0] + armor_corners[1] + armor_corners[2] + armor_corners[3]) / 4.0f;
//     std::string info = armor.number + " " + std::to_string(static_cast<int>(distance_cm)) + "cm";
//     cv::putText(target_image, info, center + cv::Point2f(0, -20),
//                 cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
//   }
// }

}  // namespace helios_cv

// register node to component
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::AutoAimDebugger);