// created by lijunqi on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "LeastSquaresNode.hpp"
#include <fstream>

namespace helios_cv
{

LeastSquaresNode::LeastSquaresNode(const rclcpp::NodeOptions& options) : rclcpp::Node("least_squares_node", options)
{
  filter_omega_.clear();
  t_.clear();
  a_ = 0.9125;
  w_ = 1.942;
  phi_ = 3.141592;
  // b_ = 1.1775;
  params_ = {a_, w_, phi_};
  isSolve_ = false;
  refresh_ = false;
  problem = std::make_shared<ceres::Problem>();
  ceres_pub =
      this->create_publisher<autoaim_interfaces::msg::Omega>("/predictor/solve_answer", rclcpp::SensorDataQoS());
  ceres_sub = this->create_subscription<autoaim_interfaces::msg::Omega>(
      "/predictor/solve_problem", rclcpp::SensorDataQoS(),
      std::bind(&LeastSquaresNode::estimate, this, std::placeholders::_1));

  RCLCPP_ERROR(logger_, "least square start!");
}

void LeastSquaresNode::estimate(autoaim_interfaces::msg::Omega::SharedPtr msg)
{
  // Handle input
  set_sub_parameter(*msg);
  // Solve
  if (isSolve_) 
  {
    // RCLCPP_INFO(logger_, "solve start");
    problem->AddParameterBlock(params_.data(), params_.size());

    for (int i = st_; i < filter_omega_.size(); i++) 
    {
      // ceres::CostFunction* const_func = new ceres::AutoDiffCostFunction<SinResidual, 1, 1, 1, 1>(
      //     new SinResidual(t_[i], filter_omega_[i]));
      // problem->AddResidualBlock(const_func, NULL, &a_, &w_, &phi_);
      ceres::CostFunction *costFunction = new CostFunctor2(t_[i], filter_omega_[i]);
      ceres::LossFunction *lossFunction = new ceres::SoftLOneLoss(0.1);
      problem->AddResidualBlock(costFunction, lossFunction, params_.data());
    }

    problem->SetParameterLowerBound(params_.data(), 0, 0.78);     // a 0.78
    problem->SetParameterUpperBound(params_.data(), 0, 1.045);    // a 1.045
    problem->SetParameterLowerBound(params_.data(), 1, 1.884);    // w 1.884
    problem->SetParameterUpperBound(params_.data(), 1, 2.0);      // w 2.0
    problem->SetParameterLowerBound(params_.data(), 2, -M_PI);  // phi -CV_PI
    problem->SetParameterUpperBound(params_.data(), 2, M_PI);   // phi CV_PI
    // problem->SetParameterLowerBound(params_.data(), 3, 0.8);   // b_ 的下界 1.045
    // problem->SetParameterUpperBound(params_.data(), 3, 1.5);   // b_ 的上界 1.31

    //a,w,phi的拟合权重
    std::array<double, 3> omega = {0.3, 0.5, 0.1};

    //对每个关键参数分别添加惩罚项
    ceres::CostFunction *costFunction1 = new CostFunctor1(params_[0], 0);
    ceres::LossFunction *lossFunction1 =
        new ceres::ScaledLoss(new ceres::HuberLoss(0.1), omega[0], ceres::TAKE_OWNERSHIP);
    problem->AddResidualBlock(costFunction1, lossFunction1, params_.data());
    ceres::CostFunction *costFunction2 = new CostFunctor1(params_[1], 1);
    ceres::LossFunction *lossFunction2 =
        new ceres::ScaledLoss(new ceres::HuberLoss(0.1), omega[1], ceres::TAKE_OWNERSHIP);
    problem->AddResidualBlock(costFunction2, lossFunction2, params_.data());
    ceres::CostFunction *costFunction3 = new CostFunctor1(params_[2], 2);
    ceres::LossFunction *lossFunction3 =
        new ceres::ScaledLoss(new ceres::HuberLoss(0.1), omega[2], ceres::TAKE_OWNERSHIP);
    problem->AddResidualBlock(costFunction3, lossFunction3, params_.data());

    RCLCPP_WARN(logger_, "Initial a: %f, w: %f, phi: %f", params_[0], params_[1], params_[2]);
    ceres::Solver::Options options;
    options.max_num_iterations = 70;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;  
    options.minimizer_type = ceres::TRUST_REGION;  
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem.get(), &summary);
    RCLCPP_WARN(logger_, "Final a: %f, w: %f, phi: %f", params_[0], params_[1], params_[2]);
    problem.reset(new ceres::Problem);


    // std::string filename = "/home/gqf74/least_square_result/2025-05/2025_05.txt";
    // std::ofstream outfile(filename, std::ios::app);  // 追加模式
    // if (outfile.is_open()) {
    //     outfile << "Final a: " << params_[0] << " " 
    //             << "w: "<< params_[1] << " " 
    //             << "phi: "<< params_[2] <<" "
    //             << "filter_omega_.size:  "<< filter_omega_.size() <<"\n";      
    //     outfile.close();
    //     RCLCPP_INFO(logger_, "Parameters saved to %s", filename.c_str());
    // } else {
    //     RCLCPP_ERROR(logger_, "Unable to open file %s", filename.c_str());
    // }

    // set output
    set_pub_parameter();
  }
}

void LeastSquaresNode::set_sub_parameter(autoaim_interfaces::msg::Omega omega)
{
  refresh_ = omega.refresh;
  if (refresh_)
  {
    least_squares_refresh();
  }
  filter_omega_.push_back(omega.omega);
  a_ = params_[0] = omega.a;
  w_ = params_[1] = omega.w;
  phi_ = params_[2] = omega.phi;
  // b_ = params_[3] = omega.b;
  t_.push_back(omega.t);
  isSolve_ = omega.solve;
  st_ = (omega.st > 0) ? omega.st : 0;
}

void LeastSquaresNode:: set_pub_parameter()
{
  autoaim_interfaces::msg::Omega pub_omega;
  pub_omega.a = params_[0];
  pub_omega.w = params_[1];
  pub_omega.phi = params_[2];
  // pub_omega.b = params_[3];

  ceres_pub->publish(pub_omega);
}

void LeastSquaresNode::least_squares_refresh()
{
  problem.reset(new ceres::Problem);
  filter_omega_.clear();
  t_.clear();
  // b_ = 1.1775;
  phi_ = 3.141592;
  w_ = 1.942;
  a_ = 0.9125;
  isSolve_ = false;
  refresh_ = false;
  RCLCPP_WARN(logger_, "least square refreshed!");
}

}  // namespace helios_cv

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::LeastSquaresNode);
