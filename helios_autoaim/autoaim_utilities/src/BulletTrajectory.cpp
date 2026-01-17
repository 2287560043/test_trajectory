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

  const double d = std::sqrt(x * x + y * y);
  const double v0 = bullet_speed_;
  double h = z;
  double dh = 1;
  
  yaw_ = std::atan2(y, x);
  
  int cnt = 0;
  while (std::fabs(dh) > PREC)
  {
      cnt++;
      pitch_ = std::atan2(h, d);

      const double vy = v0 * std::cos(pitch_);
      const double vz = v0 * std::sin(pitch_);

      if (1.0 - K * d / vy <= 1e-6) {
          solvable_ = false;
          return;
      }

      fly_time_ = -std::log(1 - K * d / vy) / K;

      double temp_h =
          (K * vz + G) * (K * d) / (K * K * vy)
          + G * std::log(1 - K * d / vy) / (K * K);

      dh = h - temp_h;
      h += dh;

      if (cnt > MAX_ITER) {
        //   solvable_ = false;     // 是否需要视情况而定
          return;
      }
  }
}


}  // namespace helios_cv