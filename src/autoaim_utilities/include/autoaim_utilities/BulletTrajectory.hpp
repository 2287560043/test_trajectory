#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>

namespace helios_cv
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;
constexpr double K = 0.022928514188;
constexpr double G = 9.7833;

struct Trajectory
{
  bool unsolvable;
  double fly_time;
  double pitch;  // 抬头为正

  // 考虑空气阻力
  // v0 子弹初速度大小，单位：m/s
  // d 目标水平距离，单位：m
  // h 目标竖直高度，单位：m
  Trajectory(const double v0, const double d, const double h);
  Trajectory() = default;
  Eigen::Matrix<double, 3, 1> bullet_solve(double x, double y, double z, double bullet_speed);

};

}  // namespace helios_cv