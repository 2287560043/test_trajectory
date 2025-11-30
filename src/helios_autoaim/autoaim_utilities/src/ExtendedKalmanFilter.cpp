// created by liuhan on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "ExtendedKalmanFilter.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "rclcpp/rclcpp.hpp"
#include "Armor.hpp"

namespace helios_cv
{

ExtendedKalmanFilter::ExtendedKalmanFilter(
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& F,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& H,
    const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& Jf,
    const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& Jh,
    const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& UpdateQ,
    const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& UpdateR,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& measurement_diff,
    const Eigen::MatrixXd& P)
  : f(F), h(H), jf(Jf), jh(Jh), update_Q(UpdateQ), update_R(UpdateR), P_post_(P), measurement_diff_(measurement_diff)
{
  state_pre_ = Eigen::VectorXd::Zero(P.rows());
  state_post_ = Eigen::VectorXd::Zero(P.rows());
}

Eigen::MatrixXd ExtendedKalmanFilter::Predict()
{
  F = jf(state_post_);
  Q = update_Q(state_post_);
  state_pre_ = f(state_post_);
  P_pre_ = lambda * F * P_post_ * F.transpose() + Q;
  state_post_ = state_pre_;
  P_post_ = P_pre_;
  return state_pre_;
}

Eigen::MatrixXd ExtendedKalmanFilter::Correct(const Eigen::VectorXd& z)
{
  static bool c_is = 1;
  Eigen::MatrixXd C_new ;
  y_old = y;
  R = update_R(z);
  H = jh(state_pre_);
  Eigen::MatrixXd S = H * P_pre_ * H.transpose() + R;
  y = measurement_diff_(z, h(state_pre_));
  if (c_is || y.rows()!= C_prev.rows()) {
    y_old = y;
    C_prev = y * y.transpose();
  }else {
    C_prev = y_old * y_old.transpose();
  }
  Eigen::MatrixXd C_prev_new = (reo * C_prev  +  y * y.transpose()) / (1 + reo);
  Eigen::MatrixXd N = C_prev_new - H * Q * H.transpose() - beta * R;
  if (N.trace() <= 0) {
    N = H * Q * H.transpose() +  R;
  }
  Eigen::MatrixXd M = H * F * P_pre_ * F.transpose() * H.transpose();
  if (M.trace() != 0) {
    lambda = std::max(1.0, N.trace() / M.trace());
  }
  c_is = 0;
  C_prev = C_prev_new;           
  Eigen::MatrixXd K = P_pre_ * H.transpose() * S.inverse();
  state_post_ = state_pre_ + K * y;
  P_post_ = (Eigen::MatrixXd::Identity(P_pre_.rows(), P_pre_.cols()) - K * H) * P_pre_;
  return state_post_;
}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd& state)
{
  state_post_ = state;
}

};  // namespace helios_cv