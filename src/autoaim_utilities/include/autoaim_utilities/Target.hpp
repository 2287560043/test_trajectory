#pragma once

#include <Eigen/Core>
#include <optional>
#include <chrono>
#include "../../include/autoaim_utilities/Armor.hpp"
#include "../../include/autoaim_utilities/ExtendedKalmanFilter.hpp"


namespace helios_cv
{

typedef enum
{
  LOST,
  TEMP_LOST,
  TRACKING,
  DETECTING
} TrakerState;

enum AUTOAIM_MODE
{
  AUTOAIM = 0,
  SMALL_ENERGY = 1,
  BIG_ENERGY = 2,
};

// 强行兼容的权宜之计
enum TargetType
{
  OUTPOST,
  NORMAL
};


class Target
{
public:
  std::string armor_number;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}
