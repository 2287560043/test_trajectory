// rebuild on 2026/01/19
// 放弃了整车预测，转而对每个 armor 建立单独的 ArmorTracker
#pragma once

#include "BaseObserver.hpp"
#include "autoaim_utilities/Models.hpp"

// #include <autoaim_interfaces/msg/detail/armors__struct.hpp>


namespace helios_cv
{
const double SWITCH_LOCK_DT = 1.0;         // 锁定时间(秒),xx秒内禁止因距离优势切换目标
const double MAX_ORIENTATION_ANGLE = M_PI_2;
const double LOST_TIME_THRESH = 0.2;       // 丢失时间阈值(秒),超过此时间未检测到装甲板则认为丢失

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
    ArmorEkf ekf;
    double last_timestamp;
    bool is_active;
    
    void init(const Eigen::Vector3d& xyz, double t) {
        ArmorEkf::MatrixX1 x0;
        // 初始化位置为观测值，速度为0
        //    x       vx  y       vy  z       vz
        x0 << xyz(0), 0,  xyz(1), 0,  xyz(2), 0;
        
        ArmorEkf::MatrixXX P0 = ArmorEkf::MatrixXX::Identity();
        P0.diagonal() << 0.1, 10, 0.1, 10, 0.1, 10;
        
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

    // [temp] 暂时为了兼容接口，后续会重写
    autoaim_interfaces::msg::Target predict_target(autoaim_interfaces::msg::Armors armors, double dt, double yaw, double bullet_speed) override;
    
    void reset_kalman() override;

    void track_armor(autoaim_interfaces::msg::Armors armors) override {
        return;
    };

    void set_params(void* params) override {
        return;
    }
    // [temp] 暂时为了兼容接口，后续会重写


protected:
    // Armor Number (string, e.g., "1", "2", "outpost")
    std::map<std::string, ArmorTracker> trackers_map_;

    void update_trackers(const autoaim_interfaces::msg::Armors& armors, double current_time);
  
    // 返回最佳目标的 number
    std::string select_best_target();
    
    double current_time_;

    // [temp] 暂时为了兼容接口，后续会重写
    std::map<int, int> armor_match_;
    ExtendedKalmanFilter ekf_;

private:
    StandardObserverParams params_;
    rclcpp::Logger logger_ = rclcpp::get_logger("StandardObserver");
};

} // namespace helios_cv