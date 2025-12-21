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
const double STATIC_YAW_VEL_THRESH = 0.008;  // rad/s
const double TOP_YAW_VEL_THRESH = 1e9; // 夏天的

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
      diff << z.segment(0, 3) - input_z.segment(0, 3), -math::get_angle_diff(z(3), input_z(3)),
          z.segment(4, 3) - input_z.segment(4, 3), -math::get_angle_diff(z(7), input_z(7));
      return diff;
    }
    else
    {
      Eigen::VectorXd diff(4);
      diff << z.segment(0, 3) - input_z.segment(0, 3), -math::get_angle_diff(z(3), input_z(3));
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
    Eigen::VectorXd state = target_state_; 


    Trajectory tmp_traj;
    auto getYP = [&](const Eigen::VectorXd& st) -> std::pair<double, double> {
        tmp_traj.set(choose_aim_point(st), bullet_speed_);
        if (!tmp_traj.solvable()) {
            return {0.0, 0.0};
        }
        return {tmp_traj.yaw(), tmp_traj.pitch()};
    };

    if (bullet_speed_ < 10 || bullet_speed_ > 30) bullet_speed_ = 28;

    const double yaw0 = getYP(state).first;
    yaw0_ = yaw0;
    
    Eigen::VectorXd temp_state = state;
    ExtendedKalmanFilter ekf_fwd = set_ekf(DT);
    
    // 中央差分
    ExtendedKalmanFilter ekf_back = set_ekf(-DT);
    ekf_back.setState(state); 
    Eigen::VectorXd state_prev = ekf_back.Predict();
    auto yaw_pitch_last = getYP(state_prev);
    
    auto yaw_pitch_curr = getYP(state);

    ekf_fwd.setState(state);

    for (int i = 0; i < HORIZON; i++) {
        temp_state = ekf_fwd.Predict();
        
        // 注意：在 TOP 模式下，
        // choose_aim_point 返回的是相对于车体中心的固定点（修正了半径）
        // 所以即使 temp_state 中的 yaw 在旋转，
        // 计算出的 aim_point 也是稳定的（随着车体平动而平动）
        
        auto yaw_pitch_next = getYP(temp_state);

        double pred_yaw_vel = (yaw_pitch_next.first - yaw_pitch_last.first) / (2.0 * DT);
        double pred_pitch_vel = (yaw_pitch_next.second - yaw_pitch_last.second) / (2.0 * DT);
        
        pretraj_matrix.col(i) << yaw_pitch_curr.first - yaw0, pred_yaw_vel, yaw_pitch_curr.second, pred_pitch_vel;

        yaw_pitch_last = yaw_pitch_curr;
        yaw_pitch_curr = yaw_pitch_next;
    }

    return pretraj_matrix;
}


std::vector<double> StandardObserver::choose_aim_point(const Eigen::VectorXd& state)
{
    // state: xc, vxc, yc, vyc, z1, z2, r1, r2, yaw, vyaw
    int armors_num = 4;

    double current_yaw = angles::normalize_angle(state(8));
    double yaw_diff_min = 1e3;
    double best_armor_yaw = 0.f;
    int best_armor_index = 0;

    for (int i = 0; i < armors_num; i++) {
        double tmp_yaw = current_yaw + i * (2.0 * M_PI / armors_num);
        tmp_yaw = angles::normalize_angle(tmp_yaw);

        double temp_yaw_diff = fabs(math::get_angle_diff(gimbal_yaw_, tmp_yaw));
        if (temp_yaw_diff < yaw_diff_min) {
            yaw_diff_min = temp_yaw_diff;
            best_armor_yaw = tmp_yaw;
            best_armor_index = i; 
        }
    }

    bool is_even = (best_armor_index % 2 == 0);
    double r = is_even ? state(6) : state(7);
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
  dt_ = dt; 
  autoaim_interfaces::msg::Target target;
  target.header.frame_id = params_.target_frame;
  target.header.stamp = armors.header.stamp;
  gimbal_yaw_ = yaw;
  bullet_speed_ = bullet_speed;

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
    track_armor(armors);

    if (find_state_ == TRACKING || find_state_ == TEMP_LOST) {
      target.tracking = true;
      target.id = tracking_number_; 
      target.armors_num = 4;

      auto curr_aim = choose_aim_point(target_state_);
      target.position.x = curr_aim[0];
      target.position.y = curr_aim[1];
      target.position.z = curr_aim[2]; 
      
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
      return target;
    }
    params_.max_lost = std::max(static_cast<int>(params_.lost_time_thresh / dt_), 5);
  }
  return target;
}

void StandardObserver::track_armor(autoaim_interfaces::msg::Armors armors)
{
  
  bool matched = false;
  Eigen::VectorXd measurement;
  yaw_vel_prev_ = target_state_(8);
  target_state_ = ekf_.Predict();

  if (!armors.armors.empty()){
      autoaim_interfaces::msg::Armors same_id_armor;
      autoaim_interfaces::msg::Armors standard_armor;

      armor_type_ = ARMOR_TYPE_STR[tracking_armor_.type];

      if (params_.is_sentry)
        same_priority_armor_count_ = find_priority_armor(armors, params_.priority_sequence);
      else
        same_priority_armor_count_ = find_priority_armor(armors);
      
      if (same_priority_armor_count_ == -1) {
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


          double pred_car_yaw = target_state_(8); 
          double yaw_vel = target_state_(9);


          for (auto& match : armor_match_) {
              int obs_idx = match.first; 
              int geo_sec = match.second;
              
              double obs_armor_yaw = orientation2yaw(same_id_armor.armors[obs_idx].pose.orientation);
              
              // RCLCPP_INFO(logger_ ,"id:%d, geo_sec:%d, obs_armor_yaw:%.2f", obs_idx, geo_sec, obs_armor_yaw);

              // 结合观测的yaw，判断基于EKF的相位是否合理
              double theoretical_yaw_geo = pred_car_yaw + geo_sec * M_PI / 2.0;
              double diff_yaw = std::abs(angles::shortest_angular_distance(theoretical_yaw_geo, obs_armor_yaw));

              // 如果不合理，结合当前观测的yaw，重新判断相位
              if (diff_yaw > 0.5) { 
                  
                  int best_sec = geo_sec;
                  double min_diff = 1e9;

                  for (int k = 0; k < 4; k++) {
                      double theoretical_yaw_k = pred_car_yaw + k * M_PI / 2.0;
                      double diff = std::abs(angles::shortest_angular_distance(theoretical_yaw_k, obs_armor_yaw));
                      
                      if (diff < min_diff) {
                          min_diff = diff;
                          best_sec = k;
                      }
                  }
                  
                  if (best_sec != geo_sec) {
                      if (min_diff < diff_yaw - 0.1) {
                           RCLCPP_WARN(logger_, "[Spinning] Jump! %d -> %d. Diff: %.2f -> %.2f", 
                                      geo_sec, best_sec, diff_yaw, min_diff);
                           match.second = best_sec; 
                      }
                  }
              } 

              RCLCPP_INFO(logger_, "id:%d, final_sec:%d, obs_yaw:%.2f", obs_idx, match.second, obs_armor_yaw);
          }
          RCLCPP_INFO(logger_, " ");

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

  // double yaw_vel_dt = angles::shortest_angular_distance(target_state_(8), yaw_vel_prev_)/dt_;
  // is_spinning = std::abs(yaw_vel_dt) > TOP_YAW_VEL_THRESH;
  // is_static = std::abs(yaw_vel_dt) < STATIC_YAW_VEL_THRESH;
  // auto getName = [&]() {
  //               if(is_spinning) return "spinning";
  //               if(is_static) return "static";
  //               return "normal";
  //         };
  // RCLCPP_INFO(logger_, "yaw_vel_dt:%.2f, %s", yaw_vel_dt, getName());

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