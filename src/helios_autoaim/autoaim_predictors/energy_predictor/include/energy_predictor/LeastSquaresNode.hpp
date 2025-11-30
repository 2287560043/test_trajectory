#pragma once
// ros
#include <autoaim_interfaces/msg/detail/omega__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoaim_interfaces/msg/omega.hpp"
// custom
#include "Omega.hpp"
// eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// ceres-solver
#include <ceres/ceres.h>
#include <ceres/problem.h>
// opencv
#include <opencv2/core.hpp>
// STL
#include <queue>
#include <thread>
#include <tuple>

namespace helios_cv
{

class LeastSquaresNode : public rclcpp::Node
{
public:
  LeastSquaresNode(const rclcpp::NodeOptions& options);

  void estimate(autoaim_interfaces::msg::Omega::SharedPtr msg);

private:
  rclcpp::Subscription<autoaim_interfaces::msg::Omega>::SharedPtr ceres_sub;
  rclcpp::Publisher<autoaim_interfaces::msg::Omega>::SharedPtr ceres_pub;

  struct SinResidual
  {
    SinResidual(double t, double omega) : omega_(omega), t_(t)
    {
    }

    template <class T>
    bool operator()(const T* const a, const T* const w, const T* phi, T* residual) const
    {  // const T* phi
      residual[0] = omega_ - (a[0] * sin(w[0] * t_ + phi[0]) + 2.09 - a[0]);
      return true;
    }

  private:
    const double omega_;
    const double t_;
  };
  std::shared_ptr<ceres::Problem> problem;

  void least_squares_refresh();

  void set_sub_parameter(autoaim_interfaces::msg::Omega omega);

  void set_pub_parameter();

  std::vector<double> filter_omega_;
  std::vector<double> t_;
  int st_;
  double phi_;
  double w_;
  double a_;
  double b_;
  //default parameter
  std::vector<double> params_;
  bool isSolve_;
  bool refresh_;
  // Logger
  rclcpp::Logger logger_ = rclcpp::get_logger("LeastSquaresNode");
};

/**
 * @brief 惩罚项，让拟合的参数更加贴近预设的参数
 */
class CostFunctor1 : public ceres::SizedCostFunction<1, 3> {
   public:
    CostFunctor1(double truth_, int id_) : truth(truth_), id(id_) {}
    virtual ~CostFunctor1(){};
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        double pre = parameters[0][id];
        residuals[0] = pre - truth;
        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) {
                for (int i = 0; i < 3; ++i) {
                    if (i == id) {
                        jacobians[0][i] = 1;
                    } else {
                        jacobians[0][i] = 0;
                    }
                }
            }
        }
        return true;
    }
    double truth;
    int id;
};

/**
 * @brief 拟合项
 */
class CostFunctor2 : public ceres::SizedCostFunction<1, 3> {
   public:
    CostFunctor2(double t_, double y_) : t(t_), y(y_) {}
    virtual ~CostFunctor2(){};
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        double a = parameters[0][0];
        double w = parameters[0][1];
        double phi = parameters[0][2];
        // double b = parameters[0][3];
        residuals[0] = ((a * sin(w * t + phi)) + 2.09 - a) - y;
        // RCLCPP_INFO(rclcpp::get_logger("residual"), "residuals: %f", residuals[0]);
        if (!std::isfinite(residuals[0])) {
          RCLCPP_ERROR(rclcpp::get_logger("optimizer"), "Non-finite residual!");
          return false;
        }
        if (jacobians != NULL) {
            if (jacobians[0] != NULL) {
            double sin_value = sin(w * t + phi);
            double cos_value = cos(w * t + phi);

            jacobians[0][0] = sin_value;          // 对 a 的偏导数
            jacobians[0][1] = a * cos_value * t;     // 对 w 的偏导数
            jacobians[0][2] = a * cos_value;          // 对 phi 的偏导数
            // jacobians[0][3] = 1;          // 对 b 的偏导数
            }
        }
        return true;
    }
    double t, y;
};


}  // namespace helios_cv
