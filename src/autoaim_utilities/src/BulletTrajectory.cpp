#include "autoaim_utilities/BulletTrajectory.hpp"

namespace helios_cv
{
void Trajectory::solve()
{
  solvable_ = true;
  fly_time_ = 0;
  pitch_ = 0;

  const double x = xyz_[0];
  const double y = xyz_[1];
  const double z = xyz_[2];

  yaw_ = std::atan2(y, x);

  const double d = std::sqrt(x * x + y * y);
  double h = z;

  const double v0 = bullet_speed_;

  double d_ = d, h_ = h, dh = 1e9;
  int iter = 0;

  while (std::fabs(dh) > PREC)
  {
      iter++;
      pitch_ = std::atan2(h_, d_);

      const double vy = v0 * std::cos(pitch_);
      const double vz = v0 * std::sin(pitch_);

      if (1.0 - K * d_ / vy <= 1e-6) {
          solvable_ = false;
          return;
      }

      fly_time_ = -std::log(1 - K * d_ / vy) / K;

      double temp_h =
          (K * vz + G) * (K * d_) / (K * K * vy)
          + G * std::log(1 - K * d_ / vy) / (K * K);

      dh = h - temp_h;
      h_ += dh;

      if (iter > MAX_ITER) {
          solvable_ = false;
          return;
      }
  }
}


}  // namespace helios_cv