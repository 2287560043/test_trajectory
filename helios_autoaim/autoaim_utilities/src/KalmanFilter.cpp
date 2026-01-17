// created by xu runze, tian yiyang, liu han on 2022/3/4
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "KalmanFilter.hpp"

#include <Eigen/src/Core/Matrix.h>

#include <functional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace helios_cv
{

EigenKalmanFilter::EigenKalmanFilter(const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& TransMat,
                                     const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& MeasureMat,
                                     const std::function<Eigen::MatrixXd()>& update_Q,
                                     const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& update_R,
                                     const Eigen::MatrixXd& P)
  : trans_mat_(TransMat), measure_mat_(MeasureMat), update_Q_(update_Q), update_R_(update_R), P_post_(P)
{
  state_pre_ = Eigen::VectorXd::Zero(P.rows());
  state_post_ = Eigen::VectorXd::Zero(P.rows());
}

Eigen::MatrixXd EigenKalmanFilter::predict()
{
  // X_k_bar(先验估计) = A(状态转移矩阵) * X_k-1(后验估计)
  F_ = trans_mat_(state_post_);
  state_pre_ = F_ * state_post_;
  // state_pre_ = trans_mat_ * state_post_;
  Q_ = update_Q_();
  // p_k_bar(误差协方差先验矩阵) = A(状态转移矩阵) * p_k-1(误差协方差后验矩阵) * A_T(状态转移矩阵转置) +
  // Q(过程噪音协方差矩阵)
  P_pre_ = F_ * P_post_ * F_.transpose() + Q_;
  state_post_ = state_pre_;
  P_post_ = P_pre_;
  return state_pre_;
}

Eigen::MatrixXd EigenKalmanFilter::correct(const Eigen::VectorXd& measurement)
{
  //计算高斯增益
  //               P_k_bar(误差协方差先验矩阵) * H_T(测量转移矩阵的转置)
  // K_k(高斯增益) = ------------------------------------------------------------------------
  //               H(测量转移矩阵) * P_k_bar(误差协方差先验矩阵) * H_T(测量转移矩阵的转置) + R(测量噪音协方差矩阵)
  R_ = update_R_(measurement);
  H_ = measure_mat_(state_pre_);
  Eigen::MatrixXd S = H_ * P_pre_ * H_.transpose() + R_;
  gain_ = P_pre_ * H_.transpose() * (H_ * P_pre_ * H_.transpose() + R_).inverse();  // K
  // X_k(后验估计) = X_k_bar(先验估计) + K_k(高斯增益) * (Z_k(测量量) - H(测量转移矩阵) * X_k_bar(先验估计矩阵))
  state_post_ = state_pre_ + gain_ * (measurement - H_ * state_pre_);
  // P_k(误差协方差后验估计) = P_k_bar(误差协方差先验估计) - K_k(卡尔曼增益) * H(测量转移矩阵) *
  // P_k_bar(误差协方差先验估计)
  P_post_ = P_pre_ - gain_ * H_ * P_pre_;
  return state_post_;
}

void EigenKalmanFilter::set_state(const Eigen::VectorXd& state)
{
  state_post_ = state;
}

}  // namespace helios_cv
