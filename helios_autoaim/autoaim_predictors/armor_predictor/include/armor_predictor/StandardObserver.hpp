// rebuild on 2026/01/19
// 极坐标观测
#pragma once

#include "BaseObserver.hpp"
#include "autoaim_utilities/AntiSpinEKF.hpp"

#include <map>
#include <string>

namespace helios_cv
{
const double LOST_TIME_THRESH = 0.5;

// [temp] 暂时为了兼容接口，后续会重写
typedef struct StandardObserverParams : public BaseObserverParams
{
  typedef struct DDMParams
  {
    double sigma2_q_xyz;
    double sigma2_q_yaw;
    double sigma2_q_r;
    double sigma2_q_z;
    double r_xyz_factor;
    double r_yaw;
    double r_z_factor;
  } DDMParams;
  DDMParams ekf_params;

  StandardObserverParams() = default;

  StandardObserverParams(int max_lost, int max_detect,
                         double lost_time_thresh, std::string target_frame, DDMParams ekf_params, bool is_sentry,
                         std::vector<std::string> priority_sequence)
    : BaseObserverParams(max_lost, max_detect, lost_time_thresh,
                         std::move(target_frame), is_sentry, priority_sequence)
    , ekf_params(ekf_params)
  {
  }
} StandardObserverParams;


// 定义轴类型
enum AxisType { LONG_AXIS = 0, SHORT_AXIS = 1 };

struct ArmorTracker
{
    // 双 EKF 实例
    AntiSpinEKF long_ekf;   // 观测长轴中心
    AntiSpinEKF short_ekf;  // 观测短轴中心
    
    // 当前激活的轴（即当前正在观测的轴）
    AxisType active_axis; 
    
    double last_timestamp;
    bool is_active;
    
    // 用于逻辑判断的历史数据
    int last_armors_num;
    double last_yaw;

    ArmorTracker() : active_axis(LONG_AXIS), last_timestamp(0), is_active(false), last_armors_num(0), last_yaw(0) {}
    void init(const Eigen::Vector3d& armor_pos, double armor_yaw, double t) {
        // 关键策略：给予不同的初始半径猜测，引导 EKF 分化
        // 假设长轴半径 ~0.35m-0.45m，短轴半径 ~0.20m-0.25m
        long_ekf.init(armor_pos, armor_yaw, 0.40); 
        short_ekf.init(armor_pos, armor_yaw, 0.20);
        
        last_timestamp = t;
        active_axis = LONG_AXIS; // 初始默认认为在长轴（或根据装甲板数量辅助判断）
        is_active = true;
        last_yaw = armor_yaw;
    }
};

class StandardObserver : public BaseObserver
{
public:
    StandardObserver(const StandardObserverParams& params);
    StandardObserver() = default;
    ~StandardObserver() = default;

    autoaim_interfaces::msg::Target predict_target(autoaim_interfaces::msg::Armors armors, double dt, double gimbal_yaw, double bullet_speed) override;
    
    void reset_kalman() override;
    void track_armor(autoaim_interfaces::msg::Armors armors) override;
    void set_params(void* params) override {return;};

protected:
    std::map<std::string, ArmorTracker> trackers_map_;
    void update_trackers(const autoaim_interfaces::msg::Armors& armors, double current_time);
    std::string select_best_target();
    // 辅助函数：根据 Z 轴高度和装甲板数量变化判断当前轴
    void determine_active_axis(ArmorTracker& tracker, const Eigen::Vector3d& armor_pos, int current_armors_num);
    
    // 辅助函数：同步 Yaw 和 V_yaw (刚体约束)
    void sync_yaw_states(ArmorTracker& tracker);
    
    double current_time_;

    // [temp] 暂时为了兼容接口，后续会重写
    std::map<int, int> armor_match_;
    ExtendedKalmanFilter ekf_;
    StandardObserverParams params_;
    rclcpp::Logger logger_ = rclcpp::get_logger("StandardObserver");
};

} // namespace helios_cv