// created by lijunqi, liuhan on 2024/2/1
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

// ros2
#include <memory>
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
// opencv
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
// eigen
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
// interfaces
#include "autoaim_interfaces/msg/armor.hpp"
#include "autoaim_interfaces/msg/armors.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
// utilities
#include "BaseDetector.hpp"
#include "autoaim_utilities/Armor.hpp"

namespace helios_cv
{

struct TraditionalEnergyParams : public BaseParams
{
  double binary_thresh;
  int64_t white_mask_thres;
  double rgb_weight_r_1;
  double rgb_weight_r_2;
  double rgb_weight_r_3;
  double rgb_weight_b_1;
  double rgb_weight_b_2;
  double rgb_weight_b_3;
};

struct TraditionalEnergyDebugImages : public DebugImages
{
  cv::Mat binary_img;
  cv::Mat detect_img;
  cv::Mat prepro_img;
};

struct TraditionalEnergyDebugInfos : public DebugInfos
{
};

class TraditionalEnergyDetector : public BaseDetector
{
public:
  TraditionalEnergyDetector(const TraditionalEnergyParams& params);

  ~TraditionalEnergyDetector() = default;

  std::shared_ptr<Armors> detect_armors(std::shared_ptr<Image> image);

  void set_params(void* params) final;

  DebugImages* get_debug_images() final;

  DebugInfos* get_debug_infos() final;

  inline bool isEmpty(const cv::RotatedRect& rect);
  inline cv::Point2f normalize(const cv::Point2f& pt);
  // 计算图像中两向量的夹角
  double get_angle(const cv::Point v1, const cv::Point v2);
  inline void reset();

private:
  /**
   * @brief 画识别结果
   * @param img 原图
   */
  void draw_results(cv::Mat& img);
  /**
   * @brief 图像前处理
   *
   * @param src
   * @param isred
   * @return cv::Mat
   */
  cv::Mat preprocess(cv::Mat& src, bool isred);
  /**
   * @brief 寻找目标装甲板
   *
   * @param src
   * @param contours
   * @return true
   * @return false
   */
  bool find_target_flow(const cv::Mat& src, std::vector<std::vector<cv::Point>>& contours);
  /**
   * @brief 寻找能量机关中心的R标
   *
   * @param src
   * @return true
   * @return false
   */
  bool find_target_R(const cv::Mat& src, std::vector<cv::Point>& i);
  /**
   * @brief 寻找待击打扇叶的箭头灯条
   *
   * @param src
   * @return true
   * @return false
   */
  bool find_target_Arrows(const cv::Mat& src, std::vector<cv::Point>& i);
  /**
   * @brief 寻找已点亮扇叶的箭头灯条
   *
   * @param src
   * @return true
   * @return false
   */
  bool find_target_Fans(const cv::Mat& src, std::vector<cv::Point>& i);

  /**
   * @brief 筛选完候选目标后通过夹角进行总校验
   *
   * @return true
   * @return false
   */
  bool angle_exame(cv::RotatedRect& Arrow, cv::RotatedRect& R);

  /**
   * @brief 更新ROI
   */
  inline void refresh_global_Roi();

  /**
   * @brief 设置要发送的装甲板角点
   */
  inline void getPts();
  /**
   * @brief 设置要发送的已点亮扇叶角点
   */
  inline void get_Fan_Pts();

  /**
   * @brief 将关键点从ROI上的坐标变回原图上的坐标
   */
  inline void setPoint();

  /**
   * @brief 用掩膜计算旋转矩形区域亮暗比
   */
  float Rota_White_Ratio(const cv::RotatedRect& rota, const cv::Mat& src);

  int raw_width_ = 1280;
  int raw_height_ = 1024;

  // 最终发送的角点
  std::vector<cv::Point2f> target_pts_;   
  std::vector<cv::Point2f> fan_pts_;                      //目标装甲板坐标 01为长边
  cv::Point2f circle_center_point = cv::Point2f(0, 0);  //中心R标的坐标

  // debug visiualization
  TraditionalEnergyDebugImages debug_images_;
  TraditionalEnergyDebugInfos debug_infos_;
  // 轮廓（外+内）
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  // 候选目标
  std::vector<cv::RotatedRect> Arrows_, R_, Fans_;
  // 待击打扇页的两部分（圆环与箭头）的旋转矩形
  cv::RotatedRect rota_far_, rota_close_;
  // 中心R标的旋转矩形
  cv::RotatedRect rota_R_;
  // 待击打扇叶的圆形
  cv::Point2f target_center_ = cv::Point2f(0, 0);
  float target_long_radius_ = 0.0;
  float target_short_radius_ = 0.0;
  float target_area_ = 12000.0;
  // float target_angle_rad = 0.0;
  // 全局ROI与中心R标ROI，初始（默认）为整个图像
  cv::Rect global_roi_ = cv::Rect(0, 0, raw_width_, raw_height_); 

  std::vector<cv::Point> fan_approx_;

  bool find_target_ = false;

  TraditionalEnergyParams params_;

  rclcpp::Logger logger_ = rclcpp::get_logger("TraditionalEnergyDetector");
};
}  // namespace helios_cv