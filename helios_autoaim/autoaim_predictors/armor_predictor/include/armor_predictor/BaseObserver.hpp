// created by liuhan Yechenyuzhu on 2023/9/15
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once

// ros
#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

// standard header
#include <string>
#include <utility>
#include <vector>
#include <algorithm>
#include <pstl/glue_algorithm_defs.h>

// tf2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

// interface
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <autoaim_interfaces/msg/detail/armor__struct.hpp>
#include <autoaim_interfaces/msg/detail/armors__struct.hpp>

// custom header
#include "autoaim_interfaces/msg/armors.hpp"
#include "autoaim_interfaces/msg/target.hpp"
#include "autoaim_utilities/Armor.hpp"
#include "autoaim_utilities/Target.hpp"
#include "autoaim_utilities/BulletTrajectory.hpp"
#include "autoaim_utilities/Math.hpp"
#include "autoaim_utilities/ExtendedKalmanFilter.hpp"



namespace helios_cv
{

typedef struct BaseObserverParams
{
public:
  BaseObserverParams(int max_lost, int max_detect,
                     double lost_time_thresh, std::string target_frame, bool is_sentry,
                     std::vector<std::string> priority_sequence)
    : max_lost(max_lost)
    , max_detect(max_detect)
    , lost_time_thresh(lost_time_thresh)
    , target_frame(std::move(target_frame))
    , is_sentry(is_sentry)
    , priority_sequence(std::move(priority_sequence))
  {
  }
  BaseObserverParams() = default;
  int max_lost;
  int max_detect;
  double lost_time_thresh;
  std::string target_frame;
  bool is_sentry;
  std::vector<std::string> priority_sequence;
} BaseObserverParams;

class BaseObserver
{
public:
  virtual autoaim_interfaces::msg::Target predict_target(autoaim_interfaces::msg::Armors armors, double dt, double yaw, double bullet_speed) = 0;

  virtual void reset_kalman() = 0;

  virtual void set_params(void* params) = 0;

  TargetType target_type_ = TargetType::NORMAL;
  int find_state_;

protected:
  BaseObserver() = default;

  BaseObserver(const BaseObserver&) = default;

  virtual ~BaseObserver() = default;

  virtual void update_target_type(const autoaim_interfaces::msg::Armor& armor)
  {
    if (tracking_number_ == "outpost")
    {
      target_type_ = TargetType::OUTPOST;
    }
    else
    {
      target_type_ = TargetType::NORMAL;
    }
  }

  /**
   * @brief 获取矩阵中的一组位于互不相同的行和列的数的位置，且选取的这些数的和最小
   * @param matrix 输入的矩阵
   * @param n 匹配装甲板数
   * @return map<int, int> 行和列的位置
   */
  virtual std::map<int, int> getMatch(Eigen::MatrixXd matrix, double score_max, int m);

  /**
   * @brief 根据熵权法获取装甲板相关性矩阵，相关性指标为负向指标
   * @param armors 识别装甲板序列
   * @param status 当前运动状态
   * @return 相关性矩阵
   */
  virtual Eigen::MatrixXd getScoreMat(const autoaim_interfaces::msg::Armors::_armors_type& detect_armors,
                                      const autoaim_interfaces::msg::Armors::_armors_type& standard_armors);

  // virtual Eigen::Vector3d state2position(const Eigen::VectorXd& state) = 0;

  virtual double orientation2yaw(const geometry_msgs::msg::Quaternion& orientation);

  virtual void track_armor(autoaim_interfaces::msg::Armors armors) = 0;

  /**
   * @brief find the best armor in the armors sequence
   * @param armors armors got from detector, the best armor will be place at first position
   * @param priority_sequence priority sequence vector
   * @return same priority armor's number, -1 if not fit armor
   */
  int find_priority_armor(autoaim_interfaces::msg::Armors& armors, const std::vector<std::string>& priority_sequence);

  /**
   * @brief find the closest armor to image center
   * @param armors armors got from detector, the best armor will be place at first position
   * @return same id armor's number, -1 if not fit armor
   */
  int find_priority_armor(autoaim_interfaces::msg::Armors& armors);

  int same_priority_armor_count_ = 0;

  autoaim_interfaces::msg::Armor tracking_armor_;

  std::string armor_type_;

  std::string tracking_number_;

  int lost_cnt_ = 0;
  int detect_cnt_ = 0;

  double dt_ = 0.008f;

  double m_score_tolerance = 1.0;

  // target_state = [x, vx, y, vy, z, vz, r1, r2, yaw, vyaw, dz]
  //                 0  1   2  3   4  5   6   7   8    9     10    
  Eigen::VectorXd target_state_;

private:
  /**
   * @brief 求出在n个数中选取k个数，所有的组合结果，即C(n,k)的所有结果
   * @param input 一组数，由用户决定
   * @param tmp_v 存储中间结果
   * @param result C(n,k)结果
   * @param start 起始位置，应指定为0
   * @param k k个数，由用户决定
   */
  virtual void getCombinationsNumbers(std::vector<int>& input, std::vector<int>& tmp_v,
                                      std::vector<std::vector<int>>& result, int start, int k);

  virtual double getDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);

  double min = 0, tmp = 0;  // 最小值

  std::vector<int> col;    // 数列中的每个数代表矩阵的每一列
  std::vector<int> row;    // 数列中的每个数代表矩阵的每一行
  std::vector<int> tmp_v;  // 存储C(n,k)的中间结果

  std::vector<std::vector<int>> result;     // 存储C(n,k)的结果
  std::vector<std::vector<int>> nAfour;     // 存储A(n,4)的结果
  std::vector<std::vector<int>> fourAfour;  // 存储A(4,4)的结果

  std::map<int, int> row_col;  // 存储最终结果，row_col[i]=j表示矩阵的第i行第j列是要选取的数

  rclcpp::Logger logger_ = rclcpp::get_logger("BaseObserver");
};

}  // namespace helios_cv