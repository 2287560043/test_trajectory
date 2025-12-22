// created by liu han on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <map>

namespace helios_cv
{

class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter() = default;

  ExtendedKalmanFilter(
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& F,
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& H,
      const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& Jf,
      const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& Jh,
      const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& UpdateQ,
      const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& UpdateR,
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& measurement_diff,
      const Eigen::MatrixXd& P);

  Eigen::MatrixXd Predict();

  Eigen::MatrixXd Correct(const Eigen::VectorXd& z);

  void setState(const Eigen::VectorXd& state);

  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h;
  std::function<Eigen::MatrixXd(const Eigen::MatrixXd&)> jf;
  std::function<Eigen::MatrixXd(const Eigen::MatrixXd&)> jh;
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> update_Q;
  std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> update_R;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> measurement_diff_;
  Eigen::MatrixXd F;
  Eigen::MatrixXd H;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::VectorXd state_pre_;
  Eigen::VectorXd state_post_;
  Eigen::MatrixXd P_pre_;
  Eigen::MatrixXd P_post_;
  Eigen::MatrixXd Gain_;
  double lambda = 1.0;   // 渐消因子   
  double reo = 0.95;
  double beta = 1;
  Eigen::VectorXd y;
  Eigen::VectorXd y_old;
  Eigen::MatrixXd C_prev; 
};

}  // namespace helios_cv