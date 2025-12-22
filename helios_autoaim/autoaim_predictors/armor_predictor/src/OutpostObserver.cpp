// created by liuhan, Yechenyuzhu on 20214/1/16
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "OutpostObserver.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>

#include <autoaim_interfaces/msg/detail/armor__struct.hpp>
#include <autoaim_utilities/Armor.hpp>
#include <cfloat>
#include <memory>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace helios_cv
{
OutpostObserver::OutpostObserver(const OutpostObserverParams& params) : params_(params)
{
  find_state_ = LOST;
  // init kalman filter
  auto f = [this](const Eigen::VectorXd& x) {
    Eigen::VectorXd x_new = x;
    x_new(3) += x(4) * dt_;
    return x_new;
  };
  auto j_f = [this](const Eigen::VectorXd&) {
    Eigen::MatrixXd f(5, 5);
    // clang-format off
        //  xc   yc   zc   yaw  vyaw
        f <<1,   0,   0,   0,   0,  // xc
            0,   1,   0,   0,   0,  // yc
            0,   0,   1,   0,   0,  // zc
            0,   0,   0,   1,   dt_,// yaw
            0,   0,   0,   0,   1;  // vyaw
    // clang-format on
    return f;
  };
  auto h = [this](const Eigen::VectorXd& x) {
    int sec = armor_match_.begin()->second;
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(1), yaw = x(3) + sec * 2 * M_PI / 3;
    z(0) = xc - radius_ * std::cos(yaw);  // xa
    z(1) = yc - radius_ * std::sin(yaw);  // ya
    z(2) = x(2);                          // za
    z(3) = yaw;                           // yaw
    return z;
  };
  auto j_h = [this](const Eigen::VectorXd& x) {
    int sec = armor_match_.begin()->second;
    Eigen::MatrixXd h(4, 5);
    double yaw = x(3) + sec * 2 * M_PI / 3;
    // clang-format off
        //   xc   yc   z    yaw                      vyaw
        h << 1,   0,   0,   radius_* std::sin(yaw),  0,
             0,   1,   0,   -radius_* std::cos(yaw), 0,
             0,   0,   1,   0,                       0,
             0,   0,   0,   1,                       0;
    // clang-format on
    return h;
  };
  // update_Q - process noise covariance matrix
  auto update_Q = [this](const Eigen::VectorXd& X) -> Eigen::MatrixXd {
    double t = dt_, x = params_.ekf_params.sigma2_q_xyz, y = params_.ekf_params.sigma2_q_yaw;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
    Eigen::MatrixXd q(5, 5);
    // clang-format off
        //  xc      yc      zc      yaw    vyaw
        q <<x,      0,      0,      0,     0,
            0,      x,      0,      0,     0,  
            0,      0,      x,      0,     0,
            0,      0,      0,      q_y_y, q_y_vy,
            0,      0,      0,      q_y_vy,q_vy_vy;
    // clang-format on
    return q;
  };
  // update_R - observation noise covariance matrix
  auto update_R = [this](const Eigen::VectorXd& z) -> Eigen::MatrixXd {
    Eigen::DiagonalMatrix<double, 4> r;
    double x = params_.ekf_params.r_xyz_factor;
    r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), params_.ekf_params.r_yaw_factor;
    return r;
  };
  auto measurement_diff = [this](const Eigen::VectorXd& z, const Eigen::VectorXd& input_z) -> Eigen::VectorXd {
    Eigen::VectorXd diff(4);
    diff << z.segment(0, 3) - input_z.segment(0, 3), -angles::shortest_angular_distance(z(3), input_z(3));
    return diff;
  };
  Eigen::DiagonalMatrix<double, 5> p0;
  p0.setIdentity();
  ekf_ = ExtendedKalmanFilter{ f, h, j_f, j_h, update_Q, update_R, measurement_diff, p0 };
}

void OutpostObserver::set_params(void* params)
{
  params_ = *static_cast<OutpostObserverParams*>(params);
}

autoaim_interfaces::msg::Target OutpostObserver::predict_target(autoaim_interfaces::msg::Armors armors, double dt)
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
      target.position.y = target_state_(1);
      target.position.z = target_state_(2);
      target.yaw = target_state_(3);
      target.velocity.x = 0;
      target.velocity.y = 0;
      target.velocity.z = 0;
      target.v_yaw = target_state_(4) > 0 ? 0.8 * M_PI : -0.8 * M_PI;
      target.radius_1 = target.radius_2 = radius_;
      target.dz = 0;
      target.id = tracking_number_;
      target.tracking = true;
      target.armor_type = armor_type_;
      target.armors_num = 3;
    }
    else
    {
      target.tracking = false;
    }
    // Update threshold of temp lost
    params_.max_lost = std::max(static_cast<int>(params_.lost_time_thresh / dt_ * 4 / 3.0), 5);
  }
  return target;
}

void OutpostObserver::track_armor(autoaim_interfaces::msg::Armors armors)
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
        for (int i = 0; i < 3; i++)
        {
          // Get  predictions
          double pre_yaw = target_state_(3);
          double angle = pre_yaw + 2 * M_PI / 3.0 * i;
          autoaim_interfaces::msg::Armor armor;
          armor.pose.position.x = target_state_(0) - radius_ * std::cos(angle);
          armor.pose.position.y = target_state_(1) - radius_ * std::sin(angle);
          armor.pose.position.z = target_state_(2);
          tf2::Quaternion tf_q;
          tf_q.setRPY(0, angles::from_degrees(-15.0), angle);
          armor.pose.orientation = tf2::toMsg(tf_q);
          standard_armor.armors.emplace_back(armor);
        }
        Eigen::MatrixXd score = getScoreMat(same_id_armor.armors, standard_armor.armors);
        armor_match_ = getMatch(score, m_score_tolerance, 3);
        if (armor_match_.size() == 1)
        {
          int num = armor_match_.begin()->first;
          measurement.resize(4);
          measurement << same_id_armor.armors[num].pose.position.x, same_id_armor.armors[num].pose.position.y,
              same_id_armor.armors[num].pose.position.z, orientation2yaw(same_id_armor.armors[num].pose.orientation);
          matched = true;
          target_state_ = ekf_.Correct(measurement);
        }
        else
        {
          RCLCPP_WARN(logger_, "no matched armor found");
        }
      }
    }
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
      if (lost_cnt_ > params_.max_lost)
      {
        RCLCPP_WARN(logger_, "Target lost! %d", params_.max_lost);
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

void OutpostObserver::reset_kalman()
{
  RCLCPP_DEBUG(logger_, "Kalman Refreshed!");
  // reset kalman
  double armor_x = tracking_armor_.pose.position.x;
  double armor_y = tracking_armor_.pose.position.y;
  double armor_z = tracking_armor_.pose.position.z;
  Eigen::VectorXd target(5);
  double yaw = orientation2yaw(tracking_armor_.pose.orientation);
  double car_center_x = armor_x + radius_ * cos(yaw);
  double car_center_y = armor_y + radius_ * sin(yaw);
  double car_center_z = armor_z;
  target << car_center_x, car_center_y, car_center_z, yaw, 0;
  target_state_ = target;
  ekf_.setState(target_state_);
}

Eigen::Vector3d OutpostObserver::state2position(const Eigen::VectorXd& state)
{
  double car_center_x = state(0);
  double car_center_y = state(1);
  double car_center_z = state(2);
  double yaw = state(3);
  double armor_x = car_center_x - radius_ * std::cos(yaw);
  double armor_y = car_center_y - radius_ * std::sin(yaw);
  return Eigen::Vector3d(armor_x, armor_y, car_center_z);
}

}  // namespace helios_cv