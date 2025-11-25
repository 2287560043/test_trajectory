#include "autoaim_utilities/BulletTrajectory.hpp"

#include <cmath>

namespace helios_cv
{
constexpr double g = 9.7833;

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));

  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  fly_time = (t_1 < t_2) ? t_1 : t_2;
}
Eigen::Matrix<double, 3, 1> Trajectory::bullet_solve(double x, double y, double z, double bullet_speed)
{
    Eigen::Matrix<double, 3, 1> yaw_pitch_time;

    double yaw = std::atan2(y, x);

    double d = std::sqrt(x * x + y * y);
    double h = z;

    double d_ = d, h_ = h, dh = 1.0;
    double pitch = 0.0;
    double v_y0, v_z0;

    int iter = 0;
    const int MAX_ITER = 10;
    const double PREC = 1e-3;

    while (std::fabs(dh) > PREC)
    {
        iter++;

        pitch = std::atan2(h_, d_);
        double v0 = bullet_speed;

        v_y0 = v0 * std::cos(pitch);  // 水平初速
        v_z0 = v0 * std::sin(pitch);  // 垂直初速
        double fly_time = -std::log(1 - K * d_ / v_y0) / K;
        
        double temp_h =
            (K * v_z0 + G) * (K * d_) / (K * K * v_y0)
            + g * std::log(1.f - K * d_ / v_y0) / (K * K);

        dh = h - temp_h;
        h_ += dh;

        if (iter > MAX_ITER)
            break;
    }

    yaw_pitch_time << yaw, pitch, fly_time;
    return yaw_pitch_time;
}


}  // namespace helios_cv