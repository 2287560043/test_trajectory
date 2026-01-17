#pragma once
#include <cmath>
#include <vector>

namespace helios_cv
{
const double K = 0.022928514188;
const double G = 9.7833;
const double PREC = 0.01;
const int MAX_ITER = 10;

constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

class Trajectory
{
public:
    Trajectory() = default;

    Trajectory(const std::vector<double>& xyz, double bullet_speed)
    {
        set(xyz, bullet_speed);
    }

    void set(const std::vector<double>& xyz, double bullet_speed)
    {
        xyz_ = xyz;
        bullet_speed_ = bullet_speed;
        solve();
    }

    bool solvable() const { return solvable_; }
    double pitch() const { return pitch_; }
    double yaw() const { return yaw_; }
    double flyTime() const { return fly_time_; }

private:
    std::vector<double> xyz_;
    double bullet_speed_ = 0.0;

    bool solvable_ = false;
    double fly_time_ = 0.0;
    double pitch_ = 0.0;
    double yaw_ = 0.0;

    void solve();
};

} // namespace helios_cv