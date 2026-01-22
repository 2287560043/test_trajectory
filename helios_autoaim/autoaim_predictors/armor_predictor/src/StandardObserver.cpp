#include "armor_predictor/StandardObserver.hpp"
#include <angles/angles.h>
#include <autoaim_utilities/Target.hpp>

namespace helios_cv
{

StandardObserver::StandardObserver(const StandardObserverParams& params) 
    : params_(params), logger_(rclcpp::get_logger("StandardObserver"))
{
    reset_kalman();
}

void StandardObserver::reset_kalman()
{
    trackers_map_.clear();
    find_state_ = LOST;
    tracking_number_ = "";
}

Eigen::Vector4d get_observation(const geometry_msgs::msg::Pose& pose) {
    Eigen::Vector3d pos(pose.position.x, pose.position.y, pose.position.z);
    
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double r, p, yaw_orient;
    m.getRPY(r, p, yaw_orient);

    // 球坐标
    double d_xy = std::sqrt(pos.x()*pos.x() + pos.y()*pos.y());
    double d = std::sqrt(d_xy*d_xy + pos.z()*pos.z());
    double yaw_pos = std::atan2(pos.y(), pos.x());
    double pitch_pos = std::atan2(pos.z(), d_xy);

    yaw_orient = angles::normalize_angle(yaw_orient + M_PI);

    return Eigen::Vector4d(yaw_pos, pitch_pos, d, yaw_orient);
}

void StandardObserver::update_trackers(const autoaim_interfaces::msg::Armors& armors, double current_time) {
    // 清理超时 Tracker
    for (auto it = trackers_map_.begin(); it != trackers_map_.end();) {
        if (current_time - it->second.last_timestamp > LOST_TIME_THRESH) {
            it = trackers_map_.erase(it);
        } else {
            ++it;
        }
    }

    // 更新/添加 Tracker
    for (const auto& armor : armors.armors) {
        std::string id = armor.number;
        Eigen::Vector4d z = get_observation(armor.pose);
        Eigen::Vector3d pos_cart(armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);
        
        if (trackers_map_.find(id) == trackers_map_.end()) {
            ArmorTracker tracker;
            tracker.init(pos_cart, z(3), current_time);
            trackers_map_[id] = tracker;
            continue;
        }

        ArmorTracker& tracker = trackers_map_[id];
        double dt = current_time - tracker.last_timestamp;
        tracker.last_timestamp = current_time;

        // 预测
        Eigen::Matrix<double, 9, 9> Q = Eigen::Matrix<double, 9, 9>::Identity();
        // [TODO] 从 params_ 中读取这些参数
        double q_pos = 0.05 * dt;
        double q_vel = 5.0 * dt; 
        double q_yaw = 0.1 * dt;
        double q_vyaw = 20.0 * dt;
        Q.diagonal() << q_pos, q_vel, q_pos, q_vel, q_pos/10.0, q_pos, q_yaw, q_vyaw, 0.001;

        tracker.ekf.predict(dt, Q);

        // 保证 ekf 中的 yaw 连续
        double pred_yaw = tracker.ekf.get_state()(6);
        double obs_yaw = z(3);
        double yaw_diff = angles::normalize_angle(obs_yaw - pred_yaw);

        if (std::abs(yaw_diff) > 0.8) { 
             Eigen::VectorXd x = tracker.ekf.get_state();
             x(6) = obs_yaw; 
             tracker.ekf.set_state(x);
        }

        // 更新
        Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Identity();
        R.diagonal() << 0.01, 0.01, 0.05, 0.02; // yaw_p, pitch_p, dist, yaw_orient
        
        tracker.ekf.update(z, R);
    }
}

autoaim_interfaces::msg::Target StandardObserver::predict_target(
    autoaim_interfaces::msg::Armors armors, double dt, double gimbal_yaw, double bullet_speed)
{
    current_time_ = rclcpp::Time(armors.header.stamp).seconds();
    update_trackers(armors, current_time_);

    // [temp] 老遗产，后续会重写
    std::string best_id = "";
    if (find_state_ == LOST) {
        best_id = select_best_target();
        if (!best_id.empty()) {
            find_state_ = DETECTING;
            tracking_number_ = best_id;
        }
    } else {
        if (trackers_map_.count(tracking_number_)) {
            best_id = tracking_number_;
            find_state_ = TRACKING;
        } else {
            find_state_ = LOST;
        }
    }

    autoaim_interfaces::msg::Target target_msg;
    target_msg.header.stamp = armors.header.stamp;
    target_msg.header.frame_id = params_.target_frame;
    target_msg.tracking = (find_state_ != LOST);

    if (target_msg.tracking) {
        target_msg.id = best_id;
        ArmorTracker& tracker = trackers_map_[best_id];
        Eigen::VectorXd state = tracker.ekf.get_state(); 

        // [xc, vxc, yc, vyc, za, vza, yaw, v_yaw, r]
        double xc = state(0), vxc = state(1);
        double yc = state(2), vyc = state(3);
        double za = state(4), vza = state(5);
        double yaw = state(6), v_yaw = state(7);
        double r = state(8);

        double pred_xc = xc + vxc * dt;
        double pred_yc = yc + vyc * dt;
        double pred_za = za + vza * dt;
        double pred_yaw = yaw + v_yaw * dt;

        target_msg.position.x = pred_xc + r * std::cos(pred_yaw);
        target_msg.position.y = pred_yc + r * std::sin(pred_yaw);
        target_msg.position.z = pred_za;

        // V_armor = V_center + V_spin
        // V_spin 的切向速度大小为 r * omega
        target_msg.velocity.x = vxc - r * std::sin(pred_yaw) * v_yaw;
        target_msg.velocity.y = vyc + r * std::cos(pred_yaw) * v_yaw;
        target_msg.velocity.z = vza;
        
        target_msg.yaw = pred_yaw;
        target_msg.v_yaw = v_yaw; 
        target_msg.radius_1 = r;
        target_msg.radius_2 = r;
        target_msg.dz = 0;
    }

    return target_msg;
}

// 重写了所以这里先简单选最近的
std::string StandardObserver::select_best_target()
{
    std::string best_id = "";
    double min_dist = 1e9;
    for (const auto& pair : trackers_map_) {
        
        Eigen::VectorXd state = pair.second.ekf.get_state();

        double dist = state.head(3).norm(); 
        if (dist < min_dist) {
            min_dist = dist;
            best_id = pair.first;
        }
    }
    return best_id;
}

} // namespace helios_cv