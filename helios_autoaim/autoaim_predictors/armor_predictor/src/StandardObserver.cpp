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

autoaim_interfaces::msg::Target StandardObserver::predict_target(
    autoaim_interfaces::msg::Armors armors, double dt, double gimbal_yaw, double bullet_speed)
{
    current_time_ = rclcpp::Time(armors.header.stamp).seconds();
    
    update_trackers(armors, current_time_);


    // [temp] 暂时为了兼容接口，后续会重写
    // [状态机桥接] 根据当前状态和追踪结果，决定输出
    // BaseObserver 成员: find_state_, tracking_number_, lost_count_, etc
    std::string best_id = "";
    bool is_tracking = false;

    if (find_state_ == LOST) {
        // 如果处于丢失状态，尝试寻找一个新的最佳目标
        best_id = select_best_target();
        if (!best_id.empty()) {
            find_state_ = DETECTING; // 或者直接 TRACKING 也可以
            tracking_number_ = best_id;
            is_tracking = true;
        }
    } 
    else {
        // 如果处于 DETECTING 或 TRACKING 状态
        // 检查当前锁定的 tracking_number_ 是否还在追踪器列表中
        if (trackers_map_.count(tracking_number_)) {
            best_id = tracking_number_;
            is_tracking = true;
            find_state_ = TRACKING; 
        } else {
            find_state_ = LOST;
        }
    }


    autoaim_interfaces::msg::Target target_msg;
    target_msg.header.stamp = armors.header.stamp;
    target_msg.header.frame_id = params_.target_frame;
    target_msg.tracking = is_tracking;

    if (is_tracking) {
        target_msg.id = best_id;
        ArmorTracker& tracker = trackers_map_[best_id];
        // [x, vx, y, vy, z, vz]
        Eigen::VectorXd state = tracker.ekf.get_state(); 

        target_msg.position.x = state(0);
        target_msg.position.y = state(2);
        target_msg.position.z = state(4);
        
        target_msg.velocity.x = state(1);
        target_msg.velocity.y = state(3);
        target_msg.velocity.z = state(5);
        
        target_msg.yaw = std::atan2(state(2), state(0));
        
        // 角速度估计: (x*vy - y*vx) / r^2
        // 推导如下：
        // 位置向量 r = (x, y) , 速度向量 v = (vx, vy)
        // ｜r x v = (x*vy - y*vx) ｜ 
        // = 线速度切向分量 * |r| = （w*r） * r
        // 化简即可得到 wr = (x*vy - y*vx) / |r|^2
        double r2 = state(0)*state(0) + state(2)*state(2);
        if (r2 > 1e-3) {
            target_msg.v_yaw = (state(0)*state(3) - state(2)*state(1)) / r2;
        } else {
            target_msg.v_yaw = 0;
        }

        // 单板无法求解半径
        target_msg.radius_1 = 0.0;
        target_msg.radius_2 = 0.0;
        target_msg.dz = 0.0;
    }

    return target_msg;
}

void StandardObserver::update_trackers(const autoaim_interfaces::msg::Armors& armors, double current_time)
{
    // measurement update
    for (const auto& armor : armors.armors) {
        std::string id = armor.number;
        
        // odoom frame
        Eigen::Vector3d pos_xyz(armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);

        // 如果是新目标则初始化
        if (trackers_map_.find(id) == trackers_map_.end()) {
            ArmorTracker tracker;
            tracker.init(pos_xyz, current_time);
            trackers_map_[id] = tracker;
            continue;
        }

        ArmorTracker& tracker = trackers_map_[id];
        double dt = current_time - tracker.last_timestamp;
        tracker.last_timestamp = current_time;

        // Q
        ArmorEkf::MatrixXX Q = ArmorEkf::MatrixXX::Identity();
        double s2qxyz = params_.ekf_params.sigma2_q_xyz;
        // clang-format 
        Q << s2qxyz, 0,      0,      0,    0,       0,
             0,      s2qxyz, 0,      0,    0,       0,
             0,      0,      s2qxyz, 0,    0,       0,
             0,      0,      0,      10.0, 0,       0,
             0,      0,      0,      0,    s2qxyz,  0,
             0,      0,      0,      0,    0,       10.0;
        Q *= dt;
        // clang-format off

        tracker.ekf.predict_forward(ArmorPredictFunctor(dt), Q);

        // calculate obs ypd
        double r = std::sqrt(pos_xyz.x()*pos_xyz.x() + pos_xyz.y()*pos_xyz.y());
        double obs_yaw = std::atan2(pos_xyz.y(), pos_xyz.x());
        double obs_pitch = std::atan2(pos_xyz.z(), r);
        double obs_dist = std::sqrt(r*r + pos_xyz.z()*pos_xyz.z());
        
        Eigen::Vector3d obs_ypd(obs_yaw, obs_pitch, obs_dist);

        // 保证 obs_yaw 连续
        Eigen::VectorXd x_pred = tracker.ekf.get_state();
        double pred_yaw = std::atan2(x_pred(2), x_pred(0));
        double yaw_diff = angles::shortest_angular_distance(pred_yaw, obs_yaw);
        obs_ypd(0) = pred_yaw + yaw_diff; 

        // R
        ArmorEkf::MatrixYY R = ArmorEkf::MatrixYY::Identity();
        double r_yaw = params_.ekf_params.r_yaw;
        double r_xyz = params_.ekf_params.r_xyz_factor;
        // clang-format
        R << r_yaw, 0,      0,
             0,     r_yaw,  0,
             0,     0,      r_xyz * pow(obs_dist, 2) + 0.01;
        // clang-format off

        tracker.ekf.update_forward(ArmorMeasureFunctor(), obs_ypd, R);
    }

    // 对于这一帧没有匹配到的追踪器，也要 ekf 以维持状态，或者超时删除
    for (auto it = trackers_map_.begin(); it != trackers_map_.end(); it++) {
        bool is_updated_this_frame = false;
        // 检查追踪器是否更新
        if (std::abs(it->second.last_timestamp - current_time) < 1e-5) {
            is_updated_this_frame = true;
        }

        if (!is_updated_this_frame) {
            double dt = current_time - it->second.last_timestamp;
            // 超时删除
            if (dt > LOST_TIME_THRESH) {
                it = trackers_map_.erase(it);
                continue;
            } else {
                // TEMP_LOST
                // 不更新观测让 Q 变大，以增加 P 的不确定性
                ArmorEkf::MatrixXX Q = ArmorEkf::MatrixXX::Identity() * dt;
                it->second.ekf.predict_forward(ArmorPredictFunctor(dt), Q);
            }
        }
    }
}

std::string StandardObserver::select_best_target()
{
    std::string best_id = "";
    double min_dist = 1e9;

    // 遍历所有存活的追踪器
    for (const auto& pair : trackers_map_) {
        const auto& id = pair.first;
        const auto& tracker = pair.second;

        // 简单的选择策略：选距离最近的
        Eigen::VectorXd state = tracker.ekf.get_state();
        double dist = state.head(3).norm(); // 距离原点的距离

        // 也可以加上优先级逻辑 (Priority from params)
        // int priority = get_armor_priority(id); ...

        if (dist < min_dist) {
            min_dist = dist;
            best_id = id;
        }
    }
    return best_id;
}

} // namespace helios_cv