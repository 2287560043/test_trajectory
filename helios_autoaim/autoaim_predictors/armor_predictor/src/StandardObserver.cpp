#include "armor_predictor/StandardObserver.hpp"

#include <angles/angles.h>
#include <autoaim_utilities/Target.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

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

// 辅助函数：将 ROS Pose 转换为观测向量 [yaw, pitch, dist, yaw_orient]
// 注意：这里需要与你的 AntiSpinEKF 定义的观测矩阵 H 对应
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

// [核心入口] 预测并返回 Target 消息
autoaim_interfaces::msg::Target StandardObserver::predict_target(autoaim_interfaces::msg::Armors armors, double dt, double gimbal_yaw, double bullet_speed)
{
    autoaim_interfaces::msg::Target target_msg;
    target_msg.header.frame_id = "odoom"; // 假设在 odoom 系
    target_msg.header.stamp = armors.header.stamp;

    RCLCPP_INFO(logger_, "armors size: %ld", armors.armors.size());

    // 1. 如果没有识别到装甲板，视为 LOST
    if (armors.armors.empty()) {
        find_state_ = LOST;
        target_msg.tracking = false;
        return target_msg;
    }

    // 2. 选板逻辑：简单选择距离图像中心最近的 (distance_to_image_center)
    // 假设 upstream 已经做了初步筛选，这里取第一个
    auto best_armor = armors.armors[0];
    double min_dist = best_armor.distance_to_image_center;
    for (const auto& armor : armors.armors) {
        if (armor.distance_to_image_center < min_dist) {
            min_dist = armor.distance_to_image_center;
            best_armor = armor;
        }
    }

    // 更新当前正在追踪的 ID
    tracking_number_ = best_armor.number;

    // 3. [关键] 调用核心追踪逻辑更新状态机
    // 为了兼容接口，track_armor 接收整个消息，但在内部我们只用 tracking_number_ 对应的数据
    track_armor(armors);

    // 4. 再次检查状态，如果 track_armor 更新成功
    if (trackers_map_.find(tracking_number_) == trackers_map_.end()) {
        target_msg.tracking = false;
        RCLCPP_ERROR(logger_, "tracker not found for number: %s", tracking_number_.c_str());
        return target_msg;
    }

    auto& tracker = trackers_map_[tracking_number_];
    target_msg.tracking = true;
    target_msg.id = tracking_number_;
    target_msg.armors_num = tracker.last_armors_num;

    // --- 数据填充开始 ---

    // 4.1 填充公用数据 (Yaw, V_yaw, Radius)
    // 根据当前活跃轴决定使用哪组数据
    AntiSpinEKF& output_ekf = (tracker.active_axis == LONG_AXIS) ? tracker.long_ekf : tracker.short_ekf;
    
    target_msg.yaw = output_ekf.x(6);
    target_msg.v_yaw = output_ekf.x(7);
    target_msg.radius_1 = tracker.long_ekf.x(8);  // r_long
    target_msg.radius_2 = tracker.short_ekf.x(8); // r_short
    target_msg.axis_type = tracker.active_axis;
    target_msg.dz = std::abs(tracker.long_ekf.x(4) - tracker.short_ekf.x(4));

    // 4.2 填充独立的轴心状态 (新增接口)
    // // 长轴中心
    // target_msg.l_x  = tracker.long_ekf.x(0);
    // target_msg.l_vx = tracker.long_ekf.x(1);
    // target_msg.l_y  = tracker.long_ekf.x(2);
    // target_msg.l_vy = tracker.long_ekf.x(3);
    // target_msg.l_z  = tracker.long_ekf.x(4);
    // target_msg.l_vz = tracker.long_ekf.x(5);

    // // 短轴中心
    // target_msg.s_x  = tracker.short_ekf.x(0);
    // target_msg.s_vx = tracker.short_ekf.x(1);
    // target_msg.s_y  = tracker.short_ekf.x(2);
    // target_msg.s_vy = tracker.short_ekf.x(3);
    // target_msg.s_z  = tracker.short_ekf.x(4);
    // target_msg.s_vz = tracker.short_ekf.x(5);

    // 4.3 击打点预测 (Main Aiming Point)
    // 使用 active_ekf 进行外推
    // 注意：dt 参数由外部传入（包含延时补偿和飞行时间预估）
    
    // 预测状态向量 (临时变量)
    double pred_xc  = output_ekf.x(0) + output_ekf.x(1) * dt;
    double pred_yc  = output_ekf.x(2) + output_ekf.x(3) * dt;
    double pred_za  = output_ekf.x(4) + output_ekf.x(5) * dt;
    double pred_yaw = output_ekf.x(6) + output_ekf.x(7) * dt;
    double r        = output_ekf.x(8);

    // 计算预测的装甲板空间坐标
    target_msg.position.x = pred_xc + r * std::cos(pred_yaw);
    target_msg.position.y = pred_yc + r * std::sin(pred_yaw);
    target_msg.position.z = pred_za;

    // 计算预测的速度矢量 (切向速度 = 质心速度 + 自旋切向)
    target_msg.velocity.x = output_ekf.x(1) - r * std::sin(pred_yaw) * output_ekf.x(7);
    target_msg.velocity.y = output_ekf.x(3) + r * std::cos(pred_yaw) * output_ekf.x(7);
    target_msg.velocity.z = output_ekf.x(5);

    return target_msg;
}

// [核心逻辑实现]
void StandardObserver::track_armor(autoaim_interfaces::msg::Armors armors)
{
    // 1. 找到之前选定的装甲板
    auto target_armor_it = std::find_if(armors.armors.begin(), armors.armors.end(),
        [this](const auto& armor) { return armor.number == this->tracking_number_; });

    if (target_armor_it == armors.armors.end()) {
        return; // 理论上不会发生，因为 predict_target 已经判空
    }
    const auto& target_armor = *target_armor_it;

    double timestamp = (double)armors.header.stamp.sec + (double)armors.header.stamp.nanosec * 1e-9;
    std::string id = tracking_number_;

    // 2. 初始化 Tracker
    if (trackers_map_.find(id) == trackers_map_.end()) {
        ArmorTracker tracker;
        
        Eigen::Vector3d pos(target_armor.pose.position.x, target_armor.pose.position.y, target_armor.pose.position.z);
        double yaw = std::atan2(pos.y(), pos.x());
        
        // 关键：初始化时我们不知道是长轴还是短轴
        // 策略：给 r 赋不同的初值，引导它们向不同方向收敛
        // 假设长轴 r~0.25, 短轴 r~0.20
        tracker.long_ekf.init(pos, yaw, 0.25); 
        tracker.short_ekf.init(pos, yaw, 0.20);
        
        tracker.last_timestamp = timestamp;
        tracker.active_axis = LONG_AXIS; // 默认先给长轴，后续纠正
        tracker.last_armors_num = armors.armors.size();
        tracker.is_active = true;
        
        trackers_map_[id] = tracker;
    }

    auto& tracker = trackers_map_[id];
    
    // 3. 计算 dt 并执行双重预测 (Dual Prediction)
    double dt = timestamp - tracker.last_timestamp;
    tracker.last_timestamp = timestamp;

    Eigen::Matrix<double, 9, 9> Q = Eigen::Matrix<double, 9, 9>::Identity();
    // [TODO] 从 params_ 中读取这些参数
    double q_pos = 0.05 * dt;
    double q_vel = 5.0 * dt; 
    double q_yaw = 0.1 * dt;
    double q_vyaw = 20.0 * dt;
    Q.diagonal() << q_pos, q_vel, q_pos, q_vel, q_pos/10.0, 0, q_yaw, q_vyaw, 0.0001;

    // 两个 EKF 都要推演时间步，保持状态同步
    tracker.long_ekf.predict(dt, Q);
    tracker.short_ekf.predict(dt, Q);

    // 4. 观测向量构建
    Eigen::Vector4d z_k = get_observation(target_armor.pose);
    Eigen::Vector3d obs_pos(target_armor.pose.position.x, target_armor.pose.position.y, target_armor.pose.position.z);

    // 5. 轴切换判定 (Switching Logic)
    // determine_active_axis(tracker, obs_pos, armors.armors.size());

    // 6. 更新 (Update)
    // 哪一个轴 Active，就更新哪一个
    Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Identity();
    R.diagonal() << 0.01, 0.01, 0.05, 0.02; // yaw_p, pitch_p, dist, yaw_orient

    if (tracker.active_axis == LONG_AXIS) {
        tracker.long_ekf.update(z_k, R);
        // short_ekf 不更新观测，只靠 predict 和下面的 sync
    } else {
        tracker.short_ekf.update(z_k, R);
        // long_ekf 不更新观测
    }

    // 7. 状态同步 (Sync)
    // 这是双轴模型收敛的关键：共享刚体运动状态
    sync_yaw_states(tracker);

    // 记录历史
    tracker.last_armors_num = armors.armors.size();
    find_state_ = DETECTING;

    // RCLCPP_ERROR(logger_, "active_axis: %s", tracker.active_axis == LONG_AXIS ? "LONG_AXIS" : "SHORT_AXIS");
}

void StandardObserver::determine_active_axis(ArmorTracker& tracker, const Eigen::Vector3d& armor_pos, int current_armors_num)
{
    // 逻辑：
    // 当装甲板数量发生变化时（例如 1->2 或 2->1），是判断长短轴的最佳时机。
    // 长轴和短轴的装甲板在车体坐标系下的高度 z 可能不同（取决于车辆结构），或者半径 r 明显不同。
    
    // bool num_changed = (current_armors_num != tracker.last_armors_num);
    bool num_changed = true;
    
    // 获取两个 EKF 预测的 Z 高度
    double long_z = tracker.long_ekf.x(4);
    double short_z = tracker.short_ekf.x(4);
    double obs_z = armor_pos.z();
    RCLCPP_ERROR(logger_, "obs_z: %f, long_z: %f, short_z: %f", obs_z, long_z, short_z);
    
    // 计算 Z 轴残差
    double err_long = std::abs(obs_z - long_z);
    double err_short = std::abs(obs_z - short_z);

    if (num_changed) {
        // 如果有数量变化，信任 Z 轴匹配度
        // 也可以结合 r 的观测值（如果 PNP 解算的 r 足够准）
        if (err_long < err_short) {
            tracker.active_axis = LONG_AXIS;
            // RCLCPP_WARN(logger_, "switch to LONG_AXIS");
        } else {
            tracker.active_axis = SHORT_AXIS;
            // RCLCPP_WARN(logger_, "switch to SHORT_AXIS");
        }
    } else {
        // 如果数量没变，保持惯性，或者当误差差异极大时才强行切换
        // 这里添加一个迟滞系数 0.7，防止在临界点反复横跳
        if (err_long < err_short * 0.7) {
            tracker.active_axis = LONG_AXIS;
        } else if (err_short < err_long * 0.7) {
            tracker.active_axis = SHORT_AXIS;
        }
    }

    RCLCPP_WARN(logger_, "active_axis: %s", tracker.active_axis == LONG_AXIS ? "LONG_AXIS" : "SHORT_AXIS");
}

void StandardObserver::sync_yaw_states(ArmorTracker& tracker)
{
    // x 向量定义: [xc, vxc, yc, vyc, za, vza, yaw, v_yaw, r]
    // 索引: 6 -> yaw, 7 -> v_yaw
    
    if (tracker.active_axis == LONG_AXIS) {
        // 长轴是准的，把 Yaw 强推给短轴
        tracker.short_ekf.x(6) = tracker.long_ekf.x(6); // Sync Yaw
        tracker.short_ekf.x(7) = tracker.long_ekf.x(7); // Sync V_yaw
        
        // 也可以同步中心点位置 (xc, yc)，因为物理上中心是同一个
        // 但考虑到长短轴观测误差不同，通常保留各自的 xc, yc 会更鲁棒，或者做加权融合
        tracker.short_ekf.x(0) = tracker.long_ekf.x(0); // xc
        tracker.short_ekf.x(1) = tracker.long_ekf.x(1); // vxc
        tracker.short_ekf.x(2) = tracker.long_ekf.x(2); // yc
        tracker.short_ekf.x(3) = tracker.long_ekf.x(3); // vyc

    } else {
        // 短轴是准的，推给长轴
        tracker.long_ekf.x(6) = tracker.short_ekf.x(6);
        tracker.long_ekf.x(7) = tracker.short_ekf.x(7);
        
        tracker.long_ekf.x(0) = tracker.short_ekf.x(0);
        tracker.long_ekf.x(1) = tracker.short_ekf.x(1);
        tracker.long_ekf.x(2) = tracker.short_ekf.x(2);
        tracker.long_ekf.x(3) = tracker.short_ekf.x(3);
    }
}

} // namespace helios_cv