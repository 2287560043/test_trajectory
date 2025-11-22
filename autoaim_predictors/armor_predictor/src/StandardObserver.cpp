// created by liuhan, Yechenyuzhu on 2023/1/16
// Submodule of HeliosRobotSystem
// for more see document:
// https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "StandardObserver.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>

#include <autoaim_interfaces/msg/detail/armor__struct.hpp>
#include <autoaim_utilities/Armor.hpp>
#include <cfloat>
#include <cmath>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace helios_cv
{
StandardObserver::StandardObserver(const StandardObserverParams& params) : params_(params)
{
  find_state_ = LOST;
  // init kalman filter
  auto f = [this](const Eigen::VectorXd& x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt_;
    x_new(2) += x(3) * dt_;
    x_new(8) += x(9) * dt_;
    return x_new;
  };
  auto j_f = [this](const Eigen::VectorXd&) {
    Eigen::MatrixXd f(10, 10);
    // clang-format off
        //     xc vxc   yc vyc   z1 z2 r1 r2 yaw vyaw
        f <<   1, dt_,  0, 0,    0, 0, 0, 0, 0, 0,
               0, 1,    0, 0,    0, 0, 0, 0, 0, 0,
               0, 0,    1, dt_,  0, 0, 0, 0, 0, 0,
               0, 0,    0, 1,    0, 0, 0, 0, 0, 0,
               0, 0,    0, 0,    1, 0, 0, 0, 0, 0,
               0, 0,    0, 0,    0, 1, 0, 0, 0, 0,
               0, 0,    0, 0,    0, 0, 1, 0, 0, 0,
               0, 0,    0, 0,    0, 0, 0, 1, 0, 0,
               0, 0,    0, 0,    0, 0, 0, 0, 1, dt_,
               0, 0,    0, 0,    0, 0, 0, 0, 0, 1;
    // clang-format on
    return f;
  };
  auto h = [this](const Eigen::VectorXd& x) {
    if (armor_match_.size() == 2)
    {
      int sec1 = armor_match_.begin()->second;
      int sec2 = (++armor_match_.begin())->second;
      Eigen::VectorXd z(8);
      double xc = x(0), yc = x(2), yaw1 = x(8) + sec1 * M_PI_2, r1 = x(6 + sec1 % 2);
      double yaw2 = x(8) + sec2 * M_PI_2, r2 = x(6 + sec2 % 2);
      z(0) = xc - r1 * cos(yaw1);  // xa1
      z(1) = yc - r1 * sin(yaw1);  // ya1
      z(2) = x(4 + sec1 % 2);      // za1
      z(3) = yaw1;                 // yaw1
      z(4) = xc - r2 * cos(yaw2);  // xa2
      z(5) = yc - r2 * sin(yaw2);  // ya2
      z(6) = x(4 + sec2 % 2);      // za2
      z(7) = yaw2;                 // yaw2
      return z;
    }
    else
    {
      int sec = armor_match_.begin()->second;
      Eigen::VectorXd z(4);
      double xc = x(0), yc = x(2), yaw = x(8) + sec * M_PI_2, r = x(6 + sec % 2);
      z(0) = xc - r * cos(yaw);  // xa
      z(1) = yc - r * sin(yaw);  // ya
      z(2) = x(4 + sec % 2);     // za
      z(3) = yaw;                // yaw
      return z;
    }
  };
  auto j_h = [this](const Eigen::VectorXd& x) {
    if (armor_match_.size() == 2)
    {
      int sec1 = armor_match_.begin()->second;
      int sec2 = (++armor_match_.begin())->second;
      Eigen::MatrixXd h(8, 10);
      double yaw1 = x(8) + sec1 * M_PI_2, r1 = x(6 + sec1 % 2);
      double yaw2 = x(8) + sec2 * M_PI_2, r2 = x(6 + sec2 % 2);
      // clang-format off
            //  xc   vxc  yc    vyc  z1              z2        r1                            r2                        yaw                   vyaw
            h <<1,   0,   0,    0,   0,              0,        -((sec1 + 1) % 2) * cos(yaw1), -(sec1 % 2) * cos(yaw1), r1 * sin(yaw1),       0,
                0,   0,   1,    0,   0,              0,        -((sec1 + 1) % 2) * sin(yaw1), -(sec1 % 2) * sin(yaw1), -r1 * cos(yaw1),      0,
                0,   0,   0,    0,   (sec1 + 1) % 2, sec1 % 2, 0,                             0,                       0,                    0,        
                0,   0,   0,    0,   0,              0,        0,                             0,                       1,                    0,
                1,   0,   0,    0,   0,              0,        -((sec2 + 1) % 2) * cos(yaw2), -(sec2 % 2) * cos(yaw2), r2 * sin(yaw2),       0,
                0,   0,   1,    0,   0,              0,        -((sec2 + 1) % 2) * sin(yaw2), -(sec2 % 2) * sin(yaw2), -r2 * cos(yaw2),      0,
                0,   0,   0,    0,   (sec2 + 1) % 2, sec2 % 2, 0,                             0,                       0,                    0,        
                0,   0,   0,    0,   0,              0,        0,                             0,                       1,                    0;
      // clang-format on
      return h;
    }
    else
    {
      int sec = armor_match_.begin()->second;
      Eigen::MatrixXd h(4, 10);
      double yaw = x(8) + sec * M_PI_2, r = x(6 + sec % 2);
      // clang-format off
            //  xc   vxc  yc    vyc  z1             z2       r1                           r2                     yaw                       vyaw
            h <<1,   0,   0,    0,   0,             0,       -((sec + 1) % 2) * cos(yaw), -(sec % 2) * cos(yaw), r * sin(yaw),             0,
                0,   0,   1,    0,   0,             0,       -((sec + 1) % 2) * sin(yaw), -(sec % 2) * sin(yaw), -r * cos(yaw),            0,
                0,   0,   0,    0,   (sec + 1) % 2, sec % 2, 0,                           0,                     0,                        0,        
                0,   0,   0,    0,   0,             0,       0,                           0,                     1,                        0;
      // clang-format on
      return h;
    }
  };
  // update_Q - process noise covariance matrix
  auto update_Q = [this](const Eigen::VectorXd& X) -> Eigen::MatrixXd {
    double t = dt_, x = params_.ekf_params.sigma2_q_xyz, y = params_.ekf_params.sigma2_q_yaw,
           r = params_.ekf_params.sigma2_q_r;
    double q_z_z = params_.ekf_params.sigma2_q_z;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
    double q_r = pow(t, 4) / 4 * r;
    Eigen::MatrixXd q(10, 10);
    // clang-format off
        //      xc      vxc      yc      vyc       z1   z2       r1     r2      yaw      vyaw
        q <<   q_x_x,  q_x_vx,   0,       0,       0,     0,     0,     0,      0,       0,
               q_x_vx, q_vx_vx,  0,       0,       0,     0,     0,     0,      0,       0,
               0,       0,       q_x_x,  q_x_vx,   0,     0,     0,     0,      0,       0,
               0,       0,       q_x_vx, q_vx_vx,  0,     0,     0,     0,      0,       0,
               0,       0,       0,       0,       q_z_z, 0,     0,     0,      0,       0,
               0,       0,       0,       0,       0,     q_z_z, 0,     0,      0,       0,
               0,       0,       0,       0,       0,     0,     q_r,   0,      0,       0,
               0,       0,       0,       0,       0,     0,     0,     q_r,    0,       0,
               0,       0,       0,       0,       0,     0,     0,     0,      q_y_y,   q_y_vy,
               0,       0,       0,       0,       0,     0,     0,     0,      q_y_vy,  q_vy_vy;
    // clang-format on
    return q;
  };
  // update_R - observation noise covariance matrix
  auto update_R = [this](const Eigen::VectorXd& z) -> Eigen::MatrixXd {
    if (armor_match_.size() == 2)
    {
      Eigen::DiagonalMatrix<double, 8> r;
      double x = params_.ekf_params.r_xyz_factor;
      double r_z = params_.ekf_params.r_z_factor;
      r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(r_z * z[2]), params_.ekf_params.r_yaw, abs(x * z[4]),
          abs(x * z[5]), abs(r_z * z[6]), params_.ekf_params.r_yaw;
      return r;
    }
    else
    {
      Eigen::DiagonalMatrix<double, 4> r;
      double x = params_.ekf_params.r_xyz_factor;
      r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), params_.ekf_params.r_yaw;
      return r;
    }
  };
  auto measurement_diff = [this](const Eigen::VectorXd& z, const Eigen::VectorXd input_z) -> Eigen::VectorXd {
    if (armor_match_.size() == 2)
    {
      Eigen::VectorXd diff(8);
      diff << z.segment(0, 3) - input_z.segment(0, 3), -angles::shortest_angular_distance(z(3), input_z(3)),
          z.segment(4, 3) - input_z.segment(4, 3), -angles::shortest_angular_distance(z(7), input_z(7));
      return diff;
    }
    else
    {
      Eigen::VectorXd diff(4);
      diff << z.segment(0, 3) - input_z.segment(0, 3), -angles::shortest_angular_distance(z(3), input_z(3));
      return diff;
    }
  };
  Eigen::DiagonalMatrix<double, 10> p0;
  p0.setIdentity();
  ekf_ = ExtendedKalmanFilter{ f, h, j_f, j_h, update_Q, update_R, measurement_diff, p0 };
}

void StandardObserver::set_params(void* params)
{
  params_ = *static_cast<StandardObserverParams*>(params);
}

autoaim_interfaces::msg::Target StandardObserver::predict_target(autoaim_interfaces::msg::Armors armors, double dt)
{
  dt_ = dt;
  autoaim_interfaces::msg::Target target;
  target.header.frame_id = params_.target_frame;
  target.header.stamp = armors.header.stamp;

  if (find_state_ == LOST)
  {
    target.tracking = false;
    if (armors.armors.empty())
    {
      return target;
    }
    // Find armor of the highest priority
    if (params_.is_sentry)
    {
      same_priority_armor_count_ = find_priority_armor(armors, params_.priority_sequence);
    }
    else
    {
      same_priority_armor_count_ = find_priority_armor(armors);
    }
    // if there is not armor in the priority sequence, just jump
    if (same_priority_armor_count_ != -1)
    {
      tracking_armor_ = armors.armors[0];
    }
    else
    {
      find_state_ = LOST;
      return target;
    }
    armor_type_ = tracking_armor_.type == 0 ? "SMALL" : "LARGE";
    reset_kalman();
    tracking_number_ = tracking_armor_.number;
    find_state_ = DETECTING;
    update_target_type(tracking_armor_);
  }
  else
  {
    // get observation
    track_armor(armors);
    if (find_state_ == TRACKING || find_state_ == TEMP_LOST)
    {
      // Pack data
      target.position.x = target_state_(0);
      target.position.y = target_state_(2);
      target.position.z = target_state_(4);
      target.yaw = target_state_(8);
      target.velocity.x = target_state_(1);
      target.velocity.y = target_state_(3);
      target.velocity.z = 0;
      target.v_yaw = target_state_(9);
      target.radius_1 = target_state_(6);
      target.radius_2 = target_state_(7);
      target.dz = target_state_(5) - target_state_(4);
      target.id = tracking_number_;
      target.tracking = true;
      target.armor_type = armor_type_;
      target.armors_num = 4;
    }
    else
    {
      target.tracking = false;
    }
    // Update threshold of temp lost
    params_.max_lost = std::max(static_cast<int>(params_.lost_time_thresh / dt_), 5);
  }
  return target;
}

void StandardObserver::track_armor(autoaim_interfaces::msg::Armors armors)
{
  bool matched = false;
  Eigen::VectorXd measurement;
  target_state_ = ekf_.Predict();
  if (!armors.armors.empty())
  {
    autoaim_interfaces::msg::Armors same_id_armor;
    autoaim_interfaces::msg::Armors standard_armor;

    armor_type_ = tracking_armor_.type == 0 ? "SMALL" : "LARGE";

    // find armor of the highest priority
    if (params_.is_sentry)
    {
      same_priority_armor_count_ = find_priority_armor(armors, params_.priority_sequence);
    }
    else
    {
      same_priority_armor_count_ = find_priority_armor(armors);
    }
    // if tracking armor is not armor of highest priority or
    // not in the priority sequence,
    // reset kalman and track armor of the highest priority
    if (same_priority_armor_count_ == -1)
    {
      // RCLCPP_WARN(logger_, "no priority armor found");
      find_state_ = TEMP_LOST;
    }
    else if (armors.armors[0].number != tracking_number_)
    {
      find_state_ = TEMP_LOST;
    }
    else
    {
      for (const auto& armor : armors.armors)
      {
        if (armor.number == tracking_number_)
        {
          same_id_armor.armors.emplace_back(armor);
        }
      }
      if (!same_id_armor.armors.empty())
      {
        for (int i = 0; i < 4; i++)
        {
          // Get predictions
          double pre_yaw = target_state_(8);
          double angle = pre_yaw + M_PI_2 * i;
          autoaim_interfaces::msg::Armor armor;
          armor.pose.position.x = target_state_(0) - target_state_(6 + i % 2) * std::cos(angle);
          armor.pose.position.y = target_state_(2) - target_state_(6 + i % 2) * std::sin(angle);
          armor.pose.position.z = target_state_(4 + i % 2);
          tf2::Quaternion tf_q;
          tf_q.setRPY(0, angles::from_degrees(15.0), angle);
          armor.pose.orientation = tf2::toMsg(tf_q);
          standard_armor.armors.emplace_back(armor);
        }
        Eigen::MatrixXd score = getScoreMat(same_id_armor.armors, standard_armor.armors);
        armor_match_ = getMatch(score, m_score_tolerance, 4);
        if (armor_match_.size() == 1)
        {
          int num = armor_match_.begin()->first;
          measurement.resize(4);
          measurement << same_id_armor.armors[num].pose.position.x, same_id_armor.armors[num].pose.position.y,
              same_id_armor.armors[num].pose.position.z, orientation2yaw(same_id_armor.armors[num].pose.orientation);
          matched = true;
          target_state_ = ekf_.Correct(measurement);
        }
        else if (armor_match_.size() == 2)
        {
          int num1 = armor_match_.begin()->first;
          int num2 = (++armor_match_.begin())->first;
          measurement.resize(8);
          measurement << same_id_armor.armors[num1].pose.position.x, same_id_armor.armors[num1].pose.position.y,
              same_id_armor.armors[num1].pose.position.z, orientation2yaw(same_id_armor.armors[num1].pose.orientation),
              same_id_armor.armors[num2].pose.position.x, same_id_armor.armors[num2].pose.position.y,
              same_id_armor.armors[num2].pose.position.z, orientation2yaw(same_id_armor.armors[num2].pose.orientation);
          matched = true;
          target_state_ = ekf_.Correct(measurement);
        }
        else
        {
          RCLCPP_WARN(logger_, "no matched armor found! matched armor num: %ld", armor_match_.size());
        }
      }
    }
  }
  // Prevent radius from spreading
  if (target_state_(6) < 0.2)
  {
    target_state_(6) = 0.2;
    ekf_.setState(target_state_);
  }
  else if (target_state_(6) > 0.4)
  {
    target_state_(6) = 0.4;
    ekf_.setState(target_state_);
  }
  if (target_state_(7) < 0.2)
  {
    target_state_(7) = 0.2;
    ekf_.setState(target_state_);
  }
  else if (target_state_(7) > 0.4)
  {
    target_state_(7) = 0.4;
    ekf_.setState(target_state_);
  }
  // Update state machine
  if (find_state_ == DETECTING)
  {
    if (matched)
    {
      detect_cnt_++;
      if (detect_cnt_ > params_.max_detect)
      {
        detect_cnt_ = 0;
        find_state_ = TRACKING;
      }
    }
    else
    {
      detect_cnt_ = 0;
      find_state_ = LOST;
    }
  }
  else if (find_state_ == TRACKING)
  {
    if (!matched)
    {
      find_state_ = TEMP_LOST;
      lost_cnt_++;
    }
  }
  else if (find_state_ == TEMP_LOST)
  {
    if (!matched)
    {
      lost_cnt_++;
      // RCLCPP_WARN(logger_, "max lost %d, lost_cnt %d", params_.max_lost, lost_cnt_);
      
      if (lost_cnt_ > params_.max_lost)
      {
        RCLCPP_WARN(logger_, "Target lost! %d %d", lost_cnt_, params_.max_lost);
        find_state_ = LOST;
        lost_cnt_ = 0;
      }
    }
    else
    {
      find_state_ = TRACKING;
      lost_cnt_ = 0;
    }
  }
}

void StandardObserver::reset_kalman()
{
  // RCLCPP_WARN(logger_, "Kalman Refreshed!");
  // reset kalman
  double armor_x = tracking_armor_.pose.position.x;
  double armor_y = tracking_armor_.pose.position.y;
  double armor_z = tracking_armor_.pose.position.z;
  Eigen::VectorXd target(10);
  double yaw = orientation2yaw(tracking_armor_.pose.orientation);
  double r = 0.26;
  double car_center_x = armor_x + r * std::cos(yaw);
  double car_center_y = armor_y + r * std::sin(yaw);
  double car_center_z = armor_z;
  target << car_center_x, 0, car_center_y, 0, car_center_z, car_center_z, r, r, yaw, 0;
  target_state_ = target;
  ekf_.setState(target_state_);
}

Eigen::Vector3d StandardObserver::state2position(const Eigen::VectorXd& state)
{
  double car_center_x = state(0);
  double car_center_y = state(2);
  double car_center_z = state(4);
  double r = state(7), yaw = state(8);
  double armor_x = car_center_x - r * std::cos(yaw);
  double armor_y = car_center_y - r * std::sin(yaw);
  return Eigen::Vector3d(armor_x, armor_y, car_center_z);
}

}  // namespace helios_cv