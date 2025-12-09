// created by liuhan, Yechenyuzhu on 2023/1/16
// rebuild on 2025/11/28
// Submodule of HeliosRobotSystem
// for more see document:
// https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "StandardObserver.hpp"
#include "BaseObserver.hpp"
#include <autoaim_interfaces/msg/detail/pre_trajectory__struct.hpp>
#include <autoaim_utilities/BulletTrajectory.hpp>

namespace helios_cv
{
StandardObserver::StandardObserver(const StandardObserverParams& params) : params_(params)
{
  find_state_ = LOST;
  // init kalman filter
  double dt = BaseObserver::dt_;
  ekf_ = set_ekf(dt);
}

ExtendedKalmanFilter StandardObserver::set_ekf(double dt)
{
  fixed_dt_ = dt;
  auto f = [&, this](const Eigen::VectorXd& x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * fixed_dt_;
    x_new(2) += x(3) * fixed_dt_;
    x_new(8) += x(9) * fixed_dt_;
    return x_new;
  };
  auto j_f = [&, this](const Eigen::VectorXd&) {
    Eigen::MatrixXd f(10, 10);
    // clang-format off
        //     xc vxc          yc vyc          z1 z2 r1 r2 yaw vyaw
        f <<   1, fixed_dt_,   0, 0,           0, 0, 0, 0, 0, 0,
               0, 1,           0, 0,           0, 0, 0, 0, 0, 0,
               0, 0,           1, fixed_dt_,   0, 0, 0, 0, 0, 0,
               0, 0,           0, 1,           0, 0, 0, 0, 0, 0,
               0, 0,           0, 0,           1, 0, 0, 0, 0, 0,
               0, 0,           0, 0,           0, 1, 0, 0, 0, 0,
               0, 0,           0, 0,           0, 0, 1, 0, 0, 0,
               0, 0,           0, 0,           0, 0, 0, 1, 0, 0,
               0, 0,           0, 0,           0, 0, 0, 0, 1, fixed_dt_,
               0, 0,           0, 0,           0, 0, 0, 0, 0, 1;
    // clang-format on
    return f;
  };
  auto h = [&](const Eigen::VectorXd& x) {
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
  auto update_Q = [&](const Eigen::VectorXd& X) -> Eigen::MatrixXd {
    double t = dt, x = params_.ekf_params.sigma2_q_xyz, y = params_.ekf_params.sigma2_q_yaw,
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
  auto update_R = [&](const Eigen::VectorXd& z) -> Eigen::MatrixXd {
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

  auto funcs = EkfFuncs{f, j_f, h, j_h, update_Q, update_R, measurement_diff, p0};
  
  return ExtendedKalmanFilter{
      funcs.f, funcs.h, funcs.j_f, funcs.j_h,
      funcs.update_Q, funcs.update_R, 
      funcs.measurement_diff, funcs.p0
  };


}

void StandardObserver::set_params(void* params)
{
  params_ = *static_cast<StandardObserverParams*>(params);
}

Eigen::Matrix<double, 4, HORIZON> StandardObserver::get_trajectory()
{
    Eigen::Matrix<double, 4, HORIZON> pretraj_matrix;
    pretraj_matrix.setZero();
    std::vector<std::vector<double>> yaw_pitch_results(HORIZON, std::vector<double>(2, 0.0));
    Eigen::VectorXd state = target_state_; // 已经考虑系统的延迟
    Trajectory tmp_traj;

    auto getYP = [&](const Eigen::VectorXd& st) -> std::pair<double, double> {
        tmp_traj.set(choose_aim_point(st), bullet_speed_);
        // if (!tmp_traj.solvable()) throw std::runtime_error("Unsolvable bullet trajectory!");
        if (!tmp_traj.solvable()) {
            RCLCPP_ERROR(logger_, "Unsolvable bullet trajectory!, yaw: %f, pitch: %f", tmp_traj.yaw(), tmp_traj.pitch());
        }
        return {tmp_traj.yaw(), tmp_traj.pitch()};
    };

    if (bullet_speed_ < 10 || bullet_speed_ > 30) bullet_speed_ = 28;

    const double yaw0 = getYP(state).first;
    yaw0_ = yaw0;


    // 生成 -DT * (HALF_HORIZON + 1) 和 -DT * HALF_HORIZON 以供中心差分
    ExtendedKalmanFilter ekf_back = set_ekf(-DT * (HALF_HORIZON + 1));
    ekf_back.setState(state); state = ekf_back.Predict(); 
    auto yaw_pitch_last = getYP(state);

    ExtendedKalmanFilter ekf_fwd = set_ekf(DT);
    ekf_fwd.setState(state);  state = ekf_fwd.Predict();
    auto yaw_pitch = getYP(state);

    for (int i = 0; i < HORIZON; i++) {
        ekf_fwd.setState(state);  state = ekf_fwd.Predict();
        auto yaw_pitch_next = getYP(state);

        double yaw_vel = (yaw_pitch_next.first - yaw_pitch_last.first) / (2.0 * DT);
        double pitch_vel = (yaw_pitch_next.second - yaw_pitch_last.second) / (2.0 * DT);
        
        pretraj_matrix.col(i) << yaw_pitch.first - yaw0, yaw_vel, yaw_pitch.second, pitch_vel;
        // RCLCPP_INFO(logger_, "---yaw: %f, yaw_vel: %f, pitch: %f, pitch_vel: %f", 
        //     yaw_pitch.first - yaw0, yaw_vel, yaw_pitch.second, pitch_vel);

        yaw_pitch_last = yaw_pitch;
        yaw_pitch = yaw_pitch_next;
    }

    return pretraj_matrix;
}

std::vector<double> StandardObserver::choose_aim_point(const Eigen::VectorXd& state)
{
    int armors_num = 4;
    
    double current_yaw_normalized = angles::normalize_angle(state(8));
    double car_center_yaw = math::xyz2ypd(state(0), state(2), state(4))[0];

    double yaw_diff_min = 1e9;
    double best_armor_yaw = 0.f;
    int best_armor_index = 0;

    for (int i = 0; i < armors_num; i++) {
        double tmp_yaw = current_yaw_normalized + i * (2.0 * M_PI / armors_num);
        tmp_yaw = angles::normalize_angle(tmp_yaw);

        double temp_yaw_diff = fabs(angles::shortest_angular_distance(gimbal_yaw_, tmp_yaw));
        if (temp_yaw_diff < yaw_diff_min) {
            yaw_diff_min = temp_yaw_diff;
            best_armor_yaw = tmp_yaw;
            best_armor_index = i; 
        }
    }

    // 偶数(0,2) -> state(6) [r1]
    // 奇数(1,3) -> state(7) [r2]
    bool is_even = (best_armor_index % 2 == 0);
    double r = is_even ? state(6) : state(7);

    // 偶数(0,2) -> state(4) [z1]
    // 奇数(1,3) -> state(5) [z2]
    double target_z = is_even ? state(4) : state(5);

    std::vector<double> aim_point{
        state(0) - r * std::cos(best_armor_yaw),
        state(2) - r * std::sin(best_armor_yaw),
        target_z
    };
    
    return aim_point;
}

autoaim_interfaces::msg::Target StandardObserver::predict_target(autoaim_interfaces::msg::Armors armors, double dt, double yaw, double bullet_speed)
{
  dt_ = dt; // dt 为上一次预测消耗的时间
  autoaim_interfaces::msg::Target target;
  target.header.frame_id = params_.target_frame;
  target.header.stamp = armors.header.stamp;
  gimbal_yaw_ = yaw;
  bullet_speed_ = bullet_speed;

  // RCLCPP_ERROR(logger_, "gimbal_yaw: %f, find_state: %s", yaw, TRACKER_STATE_STR[find_state_].c_str());

  autoaim_interfaces::msg::PreTrajectory pretraj;
  target.yaw0 = 0.0f;
  bool no_armor = false;

  if (find_state_ == LOST) {
    target.tracking = false;
    if (armors.armors.empty())  return target;

    if (params_.is_sentry) 
      same_priority_armor_count_ = find_priority_armor(armors, params_.priority_sequence);
    else 
      same_priority_armor_count_ = find_priority_armor(armors);

    // if there is not armor in the priority sequence, just jump
    if (same_priority_armor_count_ != -1) {
      tracking_armor_ = armors.armors[0];
    } else {
      find_state_ = LOST;
      RCLCPP_ERROR(logger_, "no priority armor found");
      no_armor = true;
      return target;
    }

    armor_type_ = ARMOR_TYPE_STR[tracking_armor_.type];
    reset_kalman();
    tracking_number_ = tracking_armor_.number;
    find_state_ = DETECTING;
    update_target_type(tracking_armor_);
  } else {
    // get observation
    track_armor(armors);
    if (find_state_ == TRACKING || find_state_ == TEMP_LOST) {
      target.position.x = choose_aim_point(target_state_)[0];
      target.position.y = choose_aim_point(target_state_)[1];
      target.position.z = choose_aim_point(target_state_)[2]; 
      // 获得基于当前时刻的理想轨迹
      Eigen::Matrix<double, 4, HORIZON> pretraj_matrix = get_trajectory();
      for(int i = 0; i < HORIZON; i++) {
        pretraj.yaw = pretraj_matrix.col(i)(0);
        pretraj.yaw_vel = pretraj_matrix.col(i)(1);
        pretraj.pitch = pretraj_matrix.col(i)(2);
        pretraj.pitch_vel = pretraj_matrix.col(i)(3);
        target.pretraj.push_back(pretraj);
      }
      target.yaw0 = yaw0_;
    } else {
      // target.tracking = false;
      return target;
    }
    // Update threshold of temp lost
    params_.max_lost = std::max(static_cast<int>(params_.lost_time_thresh / dt_), 5);
  }
  RCLCPP_ERROR(logger_, "car_yaw: %f", target_state_(8));
  return target;
}

void StandardObserver::track_armor(autoaim_interfaces::msg::Armors armors)
{
  bool matched = false;
  Eigen::VectorXd measurement;
  target_state_ = ekf_.Predict();

  // 结合观测修正
  if (!armors.armors.empty()){
      autoaim_interfaces::msg::Armors same_id_armor;
      autoaim_interfaces::msg::Armors standard_armor;

      armor_type_ = ARMOR_TYPE_STR[tracking_armor_.type];

      // find armor of the highest priority
      if (params_.is_sentry)
        same_priority_armor_count_ = find_priority_armor(armors, params_.priority_sequence);
      else
        same_priority_armor_count_ = find_priority_armor(armors);
      
      // if tracking armor is not armor of highest priority or
      // not in the priority sequence,
      // reset kalman and track armor of the highest priority
      if (same_priority_armor_count_ == -1) {
        // RCLCPP_WARN(logger_, "no priority armor found");
        find_state_ = TEMP_LOST;
      } else if (armors.armors[0].number != tracking_number_) {
        find_state_ = TEMP_LOST;
      } else {
        for (const auto& armor : armors.armors) {
          if (armor.number == tracking_number_) 
            same_id_armor.armors.emplace_back(armor);
        }

        if (!same_id_armor.armors.empty()) {
          for (int i = 0; i < 4; i++) {
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
                    
          if (armor_match_.size() == 1) {
            int num = armor_match_.begin()->first;
            measurement.resize(4);
            measurement << same_id_armor.armors[num].pose.position.x, 
                           same_id_armor.armors[num].pose.position.y,
                           same_id_armor.armors[num].pose.position.z, 
                           orientation2yaw(same_id_armor.armors[num].pose.orientation);
            matched = true;
            target_state_ = ekf_.Correct(measurement);
          } else if (armor_match_.size() == 2) {
            int num1 = armor_match_.begin()->first;
            int num2 = (++armor_match_.begin())->first;
            measurement.resize(8);
            measurement << same_id_armor.armors[num1].pose.position.x, 
                           same_id_armor.armors[num1].pose.position.y,
                           same_id_armor.armors[num1].pose.position.z, 
                           orientation2yaw(same_id_armor.armors[num1].pose.orientation),
                           same_id_armor.armors[num2].pose.position.x, 
                           same_id_armor.armors[num2].pose.position.y, 
                           same_id_armor.armors[num2].pose.position.z, 
                           orientation2yaw(same_id_armor.armors[num2].pose.orientation);
            matched = true;
            target_state_ = ekf_.Correct(measurement);
          } else {
            RCLCPP_WARN(logger_, "no matched armor found! matched armor num: %ld", armor_match_.size());
          }
        }
      }
  }

  // Prevent radius from spreading
  if (target_state_(6) < 0.2) {
    target_state_(6) = 0.2;
    ekf_.setState(target_state_);
  } else if (target_state_(6) > 0.4) {
    target_state_(6) = 0.4;
    ekf_.setState(target_state_);
  } 
  
  if (target_state_(7) < 0.2) {
    target_state_(7) = 0.2;
    ekf_.setState(target_state_);
  } else if (target_state_(7) > 0.4) {
    target_state_(7) = 0.4;
    ekf_.setState(target_state_);
  }

  // Update state machine
  update_state_machine(matched);

}

void StandardObserver::update_state_machine(bool matched)
{
  if (find_state_ == DETECTING) {
    if (matched) {
      detect_cnt_++;
      if (detect_cnt_ > params_.max_detect) {
        detect_cnt_ = 0;
        find_state_ = TRACKING;
      }
    } else {
      detect_cnt_ = 0;
      find_state_ = LOST;
    }
  } else if (find_state_ == TRACKING) {
    if (!matched) {
      find_state_ = TEMP_LOST;
      lost_cnt_++;
    }
  } else if (find_state_ == TEMP_LOST) {
    if (!matched) {
      lost_cnt_++;
      // RCLCPP_WARN(logger_, "max lost %d, lost_cnt %d", params_.max_lost, lost_cnt_);
      
      if (lost_cnt_ > params_.max_lost) {
        RCLCPP_WARN(logger_, "Target lost! %d %d", lost_cnt_, params_.max_lost);
        find_state_ = LOST;
        lost_cnt_ = 0;
      }
    } else {
      find_state_ = TRACKING;
      lost_cnt_ = 0;
    }
  }
}

void StandardObserver::reset_kalman()
{
    const auto& armor = tracking_armor_;
    double yaw = orientation2yaw(armor.pose.orientation);
    double r = 0.26;
    
    Eigen::VectorXd state(10);
    state << 
        armor.pose.position.x + r * cos(yaw), 0,        // xc, vxc
        armor.pose.position.y + r * sin(yaw), 0,        // yc, vyc  
        armor.pose.position.z, armor.pose.position.z,   // z1, z2
        r, r, yaw, 0;                                   // r1, r2, yaw, vyaw
    
    ekf_.setState(state);
    target_state_ = state;
}

}  // namespace helios_cv