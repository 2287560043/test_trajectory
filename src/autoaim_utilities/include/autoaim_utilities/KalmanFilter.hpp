// created by xu runze, tian yiyang, liu han on 2022/3/4
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once
#include <functional>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

namespace helios_cv
{

class EigenKalmanFilter
{
public:
  EigenKalmanFilter() = default;

  EigenKalmanFilter(const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& TransMat,
                    const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& MeasureMat,
                    const std::function<Eigen::MatrixXd()>& update_Q,
                    const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& update_R, const Eigen::MatrixXd& P);

  void set_state(const Eigen::VectorXd& state);

  Eigen::MatrixXd predict();
  Eigen::MatrixXd correct(const Eigen::VectorXd& measurement);

  std::function<Eigen::MatrixXd()> update_Q_;
  std::function<Eigen::MatrixXd(const Eigen::MatrixXd&)> update_R_;
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> trans_mat_;
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> measure_mat_;

  Eigen::MatrixXd F_;  //状态转移矩阵
  Eigen::MatrixXd H_;  //测量转移矩阵
  Eigen::MatrixXd Q_;  //过程激励噪声协方差矩阵
  Eigen::MatrixXd R_;  //测量噪声协方差矩阵
  Eigen::MatrixXd P_pre_;
  Eigen::MatrixXd P_post_;
  Eigen::MatrixXd gain_;        //卡尔曼增益系数
  Eigen::VectorXd state_pre_;   // k时刻先验估计
  Eigen::VectorXd state_post_;  // k时刻后验估计
};

}  // namespace helios_cv
