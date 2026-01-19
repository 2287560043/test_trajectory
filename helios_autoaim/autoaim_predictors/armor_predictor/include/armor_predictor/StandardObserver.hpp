// rebuild on 2026/01/19
// 放弃了整车估计
// 转而对每个 armor 配置一个 tracker
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


struct ArmorTracker {
    AntiSpinEKF ekf; 
    double last_timestamp;
    bool is_active;
    
    void init(const Eigen::Vector3d& xyz_armor, double yaw_armor, double t) {
        double r0 = 0.20;

        // 反推车体中心
        double xc = xyz_armor.x() - r0 * std::cos(yaw_armor);
        double yc = xyz_armor.y() - r0 * std::sin(yaw_armor);

        Eigen::Matrix<double, 9, 1> x0;
        x0.setZero();
        // [xc, vxc, yc, vyc, za, vza, yaw, v_yaw, r]
        x0 << xc, 0, yc, 0, xyz_armor.z(), 0, yaw_armor, 0, r0;
        
        Eigen::Matrix<double, 9, 9> P0 = Eigen::Matrix<double, 9, 9>::Identity();
        // 初始协方差
        P0.diagonal() << 10, 10, 10, 10, 1, 10, 0.1, 100, 0.1;
        
        ekf.init(x0, P0);
        last_timestamp = t;
        is_active = true;
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
    void track_armor(autoaim_interfaces::msg::Armors armors) override {return;};
    void set_params(void* params) override {return;};

protected:
    std::map<std::string, ArmorTracker> trackers_map_;
    void update_trackers(const autoaim_interfaces::msg::Armors& armors, double current_time);
    std::string select_best_target();
    
    double current_time_;

    // [temp] 暂时为了兼容接口，后续会重写
    std::map<int, int> armor_match_;
    ExtendedKalmanFilter ekf_;
    StandardObserverParams params_;
    rclcpp::Logger logger_ = rclcpp::get_logger("StandardObserver");
};

} // namespace helios_cv