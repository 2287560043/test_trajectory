// created by liuhan, Yechenyuzhu on 2023/1/16
// rebuild on 2025/11/28
// Submodule of HeliosRobotSystem
// Modified to implement Visual Center Priority (Jiaolong Style)

#include "StandardObserver.hpp"
#include "BaseObserver.hpp"
#include <autoaim_interfaces/msg/detail/pre_trajectory__struct.hpp>
#include <autoaim_utilities/BulletTrajectory.hpp>
#include <autoaim_utilities/Math.hpp>

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
  last_switch_time_ = rclcpp::Time(0);
  target_armor_id_ = -1;
}

ExtendedKalmanFilter StandardObserver::set_ekf(double dt)
{
  // fixed_dt_ = dt;
  auto f = [dt, this](const Eigen::VectorXd& x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt;
    x_new(2) += x(3) * dt;
    x_new(4) += x(5) * dt;
    x_new(8) += x(9) * dt;
    return x_new;
  };
  auto j_f = [dt, this](const Eigen::VectorXd&) {
    Eigen::MatrixXd f(11, 11);
    // clang-format off
        //     xc vxc          yc vyc          z1 vz  r1 r2 yaw vyaw dz
        f <<   1, dt,          0, 0,           0, 0,  0, 0, 0,  0,   0,
               0, 1,           0, 0,           0, 0,  0, 0, 0,  0,   0,
               0, 0,           1, dt,          0, 0,  0, 0, 0,  0,   0,
               0, 0,           0, 1,           0, 0,  0, 0, 0,  0,   0,
               0, 0,           0, 0,           1, dt, 0, 0, 0,  0,   0,
               0, 0,           0, 0,           0, 1,  0, 0, 0,  0,   0,
               0, 0,           0, 0,           0, 0,  1, 0, 0,  0,   0,
               0, 0,           0, 0,           0, 0,  0, 1, 0,  0,   0,
               0, 0,           0, 0,           0, 0,  0, 0, 1,  dt,  0,
               0, 0,           0, 0,           0, 0,  0, 0, 0,  1,   0,
               0, 0,           0, 0,           0, 0,  0, 0, 0,  0,   1;
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
      z(0) = xc - r1 * cos(yaw1);             // xa1
      z(1) = yc - r1 * sin(yaw1);             // ya1
      z(2) = x(4) + (sec1 % 2) ? 0 : x(10);   // za1
      z(3) = yaw1;                            // yaw1
      z(4) = xc - r2 * cos(yaw2);             // xa2
      z(5) = yc - r2 * sin(yaw2);             // ya2
      z(6) = x(4) + (sec2 % 2) ? 0 : x(10);   // za2
      z(7) = yaw2;                            // yaw2
      return z;
    }
    else
    {
      int sec = armor_match_.begin()->second;
      Eigen::VectorXd z(4);
      double xc = x(0), yc = x(2), yaw = x(8) + sec * M_PI_2, r = x(6 + sec % 2);
      z(0) = xc - r * cos(yaw);             // xa
      z(1) = yc - r * sin(yaw);             // ya
      z(2) = x(4) + (sec % 2) ? 0 : x(10);  // za
      z(3) = yaw;                           // yaw
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
            //  0.   1.   2.    3.   4.            
            //  xc   vxc  yc    vyc  z1              vz        r1                            r2                        yaw                   vyaw   dz
            h <<1,   0,   0,    0,   0,              0,        -((sec1 + 1) % 2) * cos(yaw1), -(sec1 % 2) * cos(yaw1), r1 * sin(yaw1),       0,     0,
                0,   0,   1,    0,   0,              0,        -((sec1 + 1) % 2) * sin(yaw1), -(sec1 % 2) * sin(yaw1), -r1 * cos(yaw1),      0,     0,
                0,   0,   0,    0,   (sec1 + 1) % 2, sec1 % 2, 0,                             0,                       0,                    0,     0,   
                0,   0,   0,    0,   0,              0,        0,                             0,                       1,                    0,     0,
                1,   0,   0,    0,   0,              0,        -((sec2 + 1) % 2) * cos(yaw2), -(sec2 % 2) * cos(yaw2), r2 * sin(yaw2),       0,     0,
                0,   0,   1,    0,   0,              0,        -((sec2 + 1) % 2) * sin(yaw2), -(sec2 % 2) * sin(yaw2), -r2 * cos(yaw2),      0,     0,
                0,   0,   0,    0,   (sec2 + 1) % 2, sec2 % 2, 0,                             0,                       0,                    0,     0, 
                0,   0,   0,    0,   0,              0,        0,                             0,                       1,                    0,     0;
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
  auto update_Q = [this, dt](const Eigen::VectorXd& X) -> Eigen::MatrixXd {
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

// [add] 根据 ID 获取特定装甲板的 3D 位置
Eigen::Vector3d StandardObserver::get_armor_position(const Eigen::VectorXd& state, int armor_index)
{
    double xc = state(0), yc = state(2), z = state(4 + armor_index % 2);
    double r = state(6 + armor_index % 2);
    double car_yaw = state(8);
    
    // 计算该装甲板的世界系 Yaw
    double armor_yaw = car_yaw + armor_index * (M_PI / 2.0);
    
    return Eigen::Vector3d(
        xc - r * cos(armor_yaw),
        yc - r * sin(armor_yaw),
        z
    );
}

// [add] 返回最优的 armor_id (0-3)
int StandardObserver::select_best_armor_id(const Eigen::VectorXd& state, double fly_time)
{
    double pred_x = state(0) + state(1) * fly_time;
    double pred_y = state(2) + state(3) * fly_time;
    double pred_yaw = state(8) + state(9) * fly_time;

    double angle_enemy_to_camera = std::atan2(-pred_y, -pred_x);

    int best_id = -1;
    double max_score = -1e9;

    const double LOCK_BONUS = 0.3; // temp
    RCLCPP_INFO(logger_, "");
    for (int i = 0; i < 4; i++) {
        double armor_yaw = pred_yaw + i * (M_PI / 2.0);
        armor_yaw = angles::normalize_angle(armor_yaw);

        // gimbal 和 armor 的朝向角差
        double yaw_diff = std::abs(angles::shortest_angular_distance(angle_enemy_to_camera, armor_yaw + M_PI));
        RCLCPP_INFO(logger_, "armor %d: yaw_diff:%f", i, yaw_diff);
        if (yaw_diff > (60.0 * M_PI / 180.0)) {
            continue;
        }
        
        // cos()先用着
        double score = std::cos(yaw_diff);

        
        // 如果上一次选中的 armor 看不见了则不做加分
        if (i == last_armor_id_) {
            score += LOCK_BONUS;
        }

        if (score > max_score) {
            max_score = score;
            best_id = i;
        }
    }

    if (best_id != -1) {
        last_armor_id_ = best_id; 
        return best_id;
    }
    return (last_armor_id_ != -1) ? last_armor_id_ : 0;
}

// [重构] 轨迹生成函数
Eigen::Matrix<double, 4, HORIZON> StandardObserver::get_trajectory()
{
    Eigen::Matrix<double, 4, HORIZON> pretraj_matrix;
    pretraj_matrix.setZero();
    Eigen::VectorXd state = target_state_; 

    if (bullet_speed_ < 10 || bullet_speed_ > 30) bullet_speed_ = 28;

    // 1. 估算飞行时间 (简单估算用于选择目标)
    double dist = std::sqrt(state(0)*state(0) + state(2)*state(2));
    double fly_time = dist / bullet_speed_;

    // 2. [关键修改] 在生成轨迹前，确定这一轮我们要打哪个 ID
    //    Jiaolong 逻辑：一旦锁定一个 ID，就在这段轨迹中一直跟踪它，
    //    除非它完全不可见。这里我们每帧做一次决策。
    target_armor_id_ = select_best_armor_id(state, fly_time);
    
    // 如果处于平动模式（小陀螺未激活），可能需要稍微调整逻辑（选最近的即可）
    // 但上述 orientation 逻辑在平动下也是通用的（平动时只有一个板面对相机）

    Trajectory tmp_traj;
    // Helper: 计算针对特定 ID 的 Yaw/Pitch
    auto getYP = [&](const Eigen::VectorXd& st, int id) -> std::pair<double, double> {
        // 获取该 ID 在该状态下的 3D 坐标
        Eigen::Vector3d pos = get_armor_position(st, id);
        std::vector<double> aim_pt = {pos.x(), pos.y(), pos.z()};
        
        tmp_traj.set(aim_pt, bullet_speed_);
        if (!tmp_traj.solvable()) {
            return {0.0, 0.0};
        }
        return {tmp_traj.yaw(), tmp_traj.pitch()};
    };

    // 初始 Yaw/Pitch
    const auto yp0 = getYP(state, target_armor_id_);
    yaw0_ = yp0.first; // 记录 T=0 时刻的目标 Yaw
    
    // 3. 生成轨迹 (EKF 预测)
    // 使用 target_armor_id 贯穿整个时域，保证轨迹平滑追踪同一个物理装甲板
    Eigen::VectorXd temp_state = state;
    ExtendedKalmanFilter ekf_fwd = set_ekf(DT);
    
    // 计算上一时刻状态用于计算速度 (Central Difference init)
    ExtendedKalmanFilter ekf_back = set_ekf(-DT);
    ekf_back.setState(state); 
    Eigen::VectorXd state_prev = ekf_back.Predict();
    auto yp_last = getYP(state_prev, target_armor_id_);
    auto yp_curr = yp0;

    ekf_fwd.setState(state);

    for (int i = 0; i < HORIZON; i++) {
        temp_state = ekf_fwd.Predict();
        
        // 关键：在预测的未来时刻，依然追踪同一个 target_armor_id
        // 即使它转过去了，我们也给出轨迹（让决策层决定是否发弹，或者在 PreAim 模式下跟随）
        auto yp_next = getYP(temp_state, target_armor_id_);

        double pred_yaw_vel = (yp_next.first - yp_last.first) / (2.0 * DT);
        double pred_pitch_vel = (yp_next.second - yp_last.second) / (2.0 * DT);
        
        // 填充相对角度和角速度
        pretraj_matrix.col(i) << yp_curr.first - yaw0_, pred_yaw_vel, yp_curr.second, pred_pitch_vel;

        yp_last = yp_curr;
        yp_curr = yp_next;
    }

    // 可以在这里更新 debug 信息，显示选中的 ID
    // RCLCPP_INFO(logger_, "Selected Armor ID: %d", target_armor_id);

    return pretraj_matrix;
}

autoaim_interfaces::msg::Target StandardObserver::predict_target(autoaim_interfaces::msg::Armors armors, double dt, double yaw, double bullet_speed)
{
  dt_ = dt; 
  autoaim_interfaces::msg::Target target;
  target.header.frame_id = params_.target_frame;
  target.header.stamp = armors.header.stamp;
  gimbal_yaw_ = yaw;
  bullet_speed_ = bullet_speed;

  rclcpp::Time now = armors.header.stamp;

  autoaim_interfaces::msg::PreTrajectory pretraj;
  target.yaw0 = 0.0f;

  if (armors.armors.empty()) {
      update_state_machine(false); 
      target.tracking = false;
      return target;
  }

  std::vector<autoaim_interfaces::msg::Armor> candidates = armors.armors;
  
  std::sort(candidates.begin(), candidates.end(), 
      [&](const autoaim_interfaces::msg::Armor& a, const autoaim_interfaces::msg::Armor& b) {
          return math::get_distance(a.pose.position) < math::get_distance(b.pose.position);
      });

  auto best_armor = candidates.front();
  bool should_switch = false;

  if (find_state_ == LOST) {
      should_switch = true;
  } else {
      // TRACKING or TEMP_LOST
      if (best_armor.number == tracking_number_) {
          should_switch = false;
      } else {
          bool time_expired = ((now - last_switch_time_).seconds()> SWITCH_LOCK_DT);

          bool last_target_exists = false;
          for (const auto& a : armors.armors) {
              if (a.number == tracking_number_) {
                  last_target_exists = true;
                  break;
              }
          }

          if (time_expired || !last_target_exists) {
              should_switch = true;
          } else {
              should_switch = false;
          }
      }
  }

  if (should_switch) {
      tracking_armor_ = best_armor;
      tracking_number_ = best_armor.number;
      armor_type_ = ARMOR_TYPE_STR[tracking_armor_.type];
      
      reset_kalman();
      update_target_type(tracking_armor_);
      
      last_switch_time_ = now;
      find_state_ = DETECTING;
  } else {
    // 保持旧目标
  }


  track_armor(armors);


  if (find_state_ == TRACKING || find_state_ == TEMP_LOST) {
    target.tracking = true;
    target.id = tracking_number_; 
    target.armors_num = 4;

    target.position.x = target_state_(0);
    target.position.y = target_state_(2);
    target.position.z = target_state_(4); 
    target.dz = target_state_(5);
    target.radius_1 = target_state_(6);
    target.radius_2 = target_state_(7);
    target.armor_id = target_armor_id_;

    target.yaw = target_state_(8);


    double dist = std::sqrt(target_state_(0)*target_state_(0) + target_state_(2)*target_state_(2));
    double fly_time = dist / bullet_speed_;
    target_armor_id_ = select_best_armor_id(target_state_, fly_time);
    
    // Eigen::Matrix<double, 4, HORIZON> pretraj_matrix = get_trajectory();
    
    // for(int i = 0; i < HORIZON; i++) {
    //   autoaim_interfaces::msg::PreTrajectory pt; 
    //   pt.yaw = pretraj_matrix.col(i)(0);
    //   pt.yaw_vel = pretraj_matrix.col(i)(1);
    //   pt.pitch = pretraj_matrix.col(i)(2);
    //   pt.pitch_vel = pretraj_matrix.col(i)(3);
    //   target.pretraj.push_back(pt);
    // }
    target.yaw0 = yaw0_;
  } else {
    target.tracking = false;
  }
  
  // 更新最大丢失时间阈值
  params_.max_lost = std::max(static_cast<int>(params_.lost_time_thresh / dt_), 5);

  return target;
}

void StandardObserver::track_armor(autoaim_interfaces::msg::Armor armor)
{
  
  bool matched = false;
  Eigen::VectorXd measurement;
  yaw_vel_prev_ = target_state_(8);
  target_state_ = ekf_.Predict();


      autoaim_interfaces::msg::Armors same_id_armor;
      autoaim_interfaces::msg::Armors standard_armor;

      armor_type_ = ARMOR_TYPE_STR[tracking_armor_.type];

      
          if (armor.number == tracking_number_) 
            same_id_armor.armors.emplace_back(armor);
      

      if (same_id_armor.armors.empty()) {
        find_state_ = TEMP_LOST;
      } else {
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

              // RCLCPP_INFO(logger_, "id:%d, final_sec:%d, obs_yaw:%.2f", obs_idx, match.second, obs_armor_yaw);
          }
          // RCLCPP_INFO(logger_, " ");

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
            RCLCPP_ERROR(logger_, "two armors");
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

      for (const auto& armor : armors.armors) {
          if (armor.number == tracking_number_) 
            same_id_armor.armors.emplace_back(armor);
      }

      if (same_id_armor.armors.empty()) {
        find_state_ = TEMP_LOST;
      } else {
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

              // RCLCPP_INFO(logger_, "id:%d, final_sec:%d, obs_yaw:%.2f", obs_idx, match.second, obs_armor_yaw);
          }
          // RCLCPP_INFO(logger_, " ");

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
            RCLCPP_ERROR(logger_, "two armors");
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