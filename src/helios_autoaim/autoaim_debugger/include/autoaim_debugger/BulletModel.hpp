// created by liuhan on 2024/5/1
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <queue>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

namespace helios_cv
{

class BulletModel
{
public:
  explicit BulletModel(int max_bullet_num, float air_coeff, float bullet_speed)
    : max_bullet_num_(max_bullet_num), air_coeff_(air_coeff), bullet_speed_(bullet_speed)
  {
    bullets_.reserve(max_bullet_num_);
  }

  void add_new_bullet(double yaw, double pitch, const Eigen::Vector3d& dead_position)
  {
    if (bullets_.size() < max_bullet_num_)
    {
      double dead_distance = std::sqrt(dead_position.x() * dead_position.x() + dead_position.y() * dead_position.y());
      double dead_time = (std::exp(air_coeff_ * dead_distance) - 1) / (air_coeff_ * bullet_speed_ * std::cos(pitch));
      bullets_.emplace_back(Eigen::Vector3d{ 0, 0, 0 },
                            Eigen::Vector3d{ bullet_speed_ * std::cos(pitch) * std::cos(yaw),
                                             bullet_speed_ * std::cos(pitch) * std::sin(yaw),
                                             bullet_speed_ * std::sin(pitch) },
                            dead_time, dead_distance);
    }
  }

  void update_bullets_status(double dt)
  {
    // only consider the air resistance in the horizontal direction
    // update the velocity of the bullet
    for (auto it = bullets_.begin(); it != bullets_.end();)
    {
      Bullet& bullet = *it;
      // remove the bullet if it is out of the dead time
      if (bullet.elapsed_time > bullet.dead_time)
      {
        it = bullets_.erase(it);
        continue;
      }
      else
      {
        /// update the position of the bullet
        bullet.elapsed_time += dt;
        // get the integral of velocity
        double vel_of_horizontal = bullet.vel.norm() * std::cos(std::asin(bullet.vel.z() / bullet.vel.norm()));
        double distance_elapsed = (1.0 / air_coeff_) * std::log(1 + air_coeff_ * vel_of_horizontal * dt);
        // caculate the new position of the bullet in x and y direction
        bullet.pos(0) += distance_elapsed * bullet.vel.x() / vel_of_horizontal;
        bullet.pos(1) += distance_elapsed * bullet.vel.y() / vel_of_horizontal;
        // caculate the new position of the bullet in z direction
        bullet.pos(2) += bullet.vel(2) * dt - 0.5 * g * dt * dt;
        /// update the velocity of the bullet
        bullet.vel(2) -= g * dt;
        // update the velocity of the bullet in x and y direction
        double vel_of_horizontal_new = vel_of_horizontal / (1 + air_coeff_ * vel_of_horizontal * dt);
        bullet.vel(0) = vel_of_horizontal_new * bullet.vel.x() / vel_of_horizontal;
        bullet.vel(1) = vel_of_horizontal_new * bullet.vel.y() / vel_of_horizontal;
        ++it;
      }
    }
  }

  std::vector<cv::Mat> get_bullet_positions_in_imu()
  {
    std::vector<cv::Mat> bullet_positions;
    for (const auto& bullet : bullets_)
    {
      cv::Mat bullet_position = (cv::Mat_<double>(3, 1) << bullet.pos.x(), bullet.pos.y(), bullet.pos.z());
      bullet_positions.emplace_back(bullet_position);
    }
    return bullet_positions;
  }

  void update_params(float air_coeff, float bullet_speed)
  {
    air_coeff_ = air_coeff;
    if (bullet_speed < 20 || bullet_speed > 30)
    {
      bullet_speed_ = 27;
    }
    else
    {
      bullet_speed_ = bullet_speed;
    }
  }

private:
  class Bullet
  {
  public:
    Bullet(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double dead_time, double dead_distance)
      : pos(pos), vel(vel), dead_time(dead_time), dead_distance(dead_distance)
    {
    }
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;

    double dead_time;
    double dead_distance;
    double elapsed_time = 0.0;
  };

  constexpr static double g = 9.81;

  std::vector<Bullet> bullets_;
  double bullet_speed_;
  double air_coeff_;
  int max_bullet_num_;

  // Logger
  rclcpp::Logger logger_ = rclcpp::get_logger("BulletModel");
};

}  // namespace helios_cv