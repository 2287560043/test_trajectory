// created by liuhan, Yechenyuzhu on 2024/1/16
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once

#include "BaseObserver.hpp"
#include "autoaim_utilities/BulletTrajectory.hpp"

namespace helios_cv
{

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

class StandardObserver : public BaseObserver
{
public:
  StandardObserver(const StandardObserverParams& params);

  autoaim_interfaces::msg::Target predict_target(autoaim_interfaces::msg::Armors armors, double dt) override;
  
  // 基于当前状态（目标命中时刻），用于预测dt秒后的目标状态，内部不处理选板逻辑
  Eigen::Matrix<double, 4, HORIZON> get_trajectory();

  EkfFuncs setup_ekf_funcs(double dt, const StandardObserverParams& params);
  // void setup_update_ekf(const StandardObserverParams& params);

  void reset_kalman() override;

  void set_params(void* params) override;

  // Trajectory get_trajectory(double yaw0, double bullet_speed);

protected:
  StandardObserver() = default;

  void track_armor(autoaim_interfaces::msg::Armors armors) override;

  Eigen::Vector3d state2position(const Eigen::VectorXd& state) override;

  // kalman utilities
  ExtendedKalmanFilter ekf_;
  ExtendedKalmanFilter update_ekf_;
  std::map<int, int> armor_match_;
  double bullet_speed_;
  double yaw0;

private:
  // Params
  StandardObserverParams params_;
  // Logger
  rclcpp::Logger logger_ = rclcpp::get_logger("StandardObserver");
};

}  // namespace helios_cv