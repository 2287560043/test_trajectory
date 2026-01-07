
// created by liu han on 2024/1/27
// Submodule of HeliosRobotSystem
// for more see document:
// https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once
// ROS2
#include <angles/angles.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>

// STL
#include <array>
#include <vector>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv2/imgproc.hpp>

// Ceres
#include <ceres/ceres.h>

#include <autoaim_armor_detector/Armor.hpp>

namespace helios_cv {

constexpr double small_armor_width = 135.0;
constexpr double small_armor_height = 55.0;
constexpr double large_armor_width = 225.0;
constexpr double large_armor_height = 55.0;

constexpr double energy_armor_height = 300.0;
constexpr double energy_armor_width_ = 300.0;
constexpr double energy_fan_width_ = 568.48 / 1000.0; // Unit: m

const double DETECTOR_ERROR_PIXEL_BY_SLOPE = 2.;

struct BaseTransformInfo {
  virtual ~BaseTransformInfo() = default;
};

struct ArmorTransformInfo : BaseTransformInfo {
  ArmorTransformInfo(cv::Quatd odom2cam, cv::Quatd cam2odom, double gimbal_yaw)
      : odom2cam_r(odom2cam), cam2odom_r(cam2odom) , gimbal_yaw(gimbal_yaw) {}

  cv::Quatd odom2cam_r;
  cv::Quatd cam2odom_r;
  double gimbal_yaw;
};

struct EnergyTransformInfo : BaseTransformInfo {
  EnergyTransformInfo(cv::Quatd odom2cam, cv::Quatd cam2odom, double gimbal_yaw)
      : odom2cam_r(odom2cam), cam2odom_r(cam2odom), gimbal_yaw(gimbal_yaw) {}

  cv::Quatd odom2cam_r;
  cv::Quatd cam2odom_r;
  double gimbal_yaw;
};

cv::Quatd ros2cv(const geometry_msgs::msg::Quaternion &ros_quat);

geometry_msgs::msg::Quaternion cv2ros(const cv::Quatd &cv_quat);

class PnPSolver {
public:
  PnPSolver(const std::array<double, 9> &camera_matrix,
            const std::vector<double> &distortion_coefficients);

  // Get 3d position
  virtual bool solve_pose(const Armor &armor, cv::Mat &rvec, cv::Mat &tvec);

  virtual void update_transform_info(BaseTransformInfo *transform_info){};

  virtual void draw_projection_points(cv::Mat &image){};

  // Calculate the distance between armor center and image center
  float calculateDistanceToCenter(const cv::Point2f &image_point);

  virtual void set_use_projection(bool use_projection) = 0;

  virtual bool use_projection() = 0;

protected:
  bool solve_pnp(const Armor &armor, cv::Mat &rvec, cv::Mat &tvec);

  // Four vertices of armor in 3d
  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
  std::vector<std::vector<cv::Point3f>> energy_fan_points_;
  std::vector<cv::Point2f> image_fans_points_;
  std::vector<cv::Point3f> object_points_;

  std::vector<cv::Point2f> image_armor_points_;
  std::vector<cv::Point3f> energy_armor_points_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("PnPSolver");
};

class ArmorProjectYaw : public PnPSolver {
public:
  explicit ArmorProjectYaw(const std::array<double, 9> &camera_matrix,
                           const std::vector<double> &dist_coeffs);

  bool solve_pose(const Armor &armor, cv::Mat &rvec, cv::Mat &tvec) override;

  void update_transform_info(BaseTransformInfo *transform_info) override;

  void draw_projection_points(cv::Mat &image) override;

  void set_use_projection(bool use_projection) override;

  bool use_projection() override;

private:
  bool use_projection_ = true;
  double gimbal_yaw;
  typedef struct CostFunctor {
  double incline;
    explicit CostFunctor(const double incline_) : incline(incline_) {};
    template <typename T>
    bool operator()(const T *const yaw, T *residual) const;

    template <typename T>
    std::vector<Eigen::Matrix<T, 2, 1>> cvProject(const std::vector<Eigen::Vector<T, 3>>& objectPoints,
      const Eigen::Matrix<T, 3, 3>& r_mat,  const Eigen::Matrix<T, 3, 1>& t_vec,
      const Eigen::Matrix<T, 3, 3>& camera_matrix,const std::vector<T>& distCoeffs) const;

    template<typename T>
    inline T sq(const T& x) const{return x * x;}
    template<typename T>
    inline Eigen::Matrix<T, 2, 1> cvPointToEigen(const cv::Point_<float>& pt) const{return Eigen::Matrix<T, 2, 1>(T(pt.x), T(pt.y));}
    template <typename T>
    inline T get_abs_angle(const Eigen::Matrix<T, 2, 1>& vec1,  const Eigen::Matrix<T, 2, 1>& vec2) const {
        const T eps = T(1e-10);
        T norm1 = vec1.norm();
        T norm2 = vec2.norm();
        if (ceres::abs(norm1) < eps || ceres::abs(norm2) < eps) {
            return T(0.0);
        }
        T dot_product = vec1.dot(vec2);
        T cos_theta = dot_product / (norm1 * norm2);
        cos_theta = Eigen::numext::mini(cos_theta, T(1.0));
        cos_theta = Eigen::numext::maxi(cos_theta, T(-1.0));
        return ceres::acos(cos_theta);
    }
  } CostFunctor;
  bool is_transform_info_updated_ = false;

  cv::Matx33d odom2cam_r_;
  cv::Matx33d cam2odom_r_;

  // self pointer to make self pointer accessible in ceres callback
  inline static ArmorProjectYaw *pthis_;
  // 限制到 -pi ~ pi
  inline double reduced_angle(const double& x) {
    return std::atan2(std::sin(x), std::cos(x));
  }

  void diff_function(double yaw);

  [[maybe_unused]] double phi_optimization(double left, double right,
                                           double eps);

  void get_rotation_matrix(double yaw, cv::Mat &rotation_mat) const;

  std::vector<cv::Point2f> projected_points_;
  cv::Mat tvec_;
  // The pitch and roll of armor are fixed for target
  double roll_ = 0, pitch_ = angles::from_degrees(15);
  double armor_angle_;

  rclcpp::Logger logger_ = rclcpp::get_logger("ArmorProjectYaw");
};

class EnergyProjectRoll : public PnPSolver {
public:
  explicit EnergyProjectRoll(const std::array<double, 9> &camera_matrix,
                             const std::vector<double> &dist_coeffs);

  bool solve_pose(const Armor &armor, cv::Mat &rvec, cv::Mat &tvec) override;

  void draw_projection_points(cv::Mat &image) override;

  void update_transform_info(BaseTransformInfo *transform_info) override;

  void set_use_projection(bool use_projection) override;

  bool use_projection() override;

private:
  bool use_projection_ = true;

  // self pointer to make self pointer accessible in ceres callback
  inline static EnergyProjectRoll *pthis_;
  double yaw_;

  struct CostFunctor {
    int num_points;  
    CostFunctor(int n) : num_points(n) {}
    template <typename T>
    bool operator()(const T *const roll, T *residual) const;
  };

  bool is_transform_info_updated_ = false;

  cv::Matx33d odom2cam_r_;
  cv::Matx33d cam2odom_r_;

  double pitch_ = 0;
  std::vector<cv::Point2f> projected_points_;
  cv::Mat tvec_;

  double diff_function(double roll);

  void get_rotation_matrix(double roll, cv::Mat &rotation_mat) const;

  rclcpp::Logger logger_ = rclcpp::get_logger("EnergyProjectRoll");
};

} // namespace helios_cv
