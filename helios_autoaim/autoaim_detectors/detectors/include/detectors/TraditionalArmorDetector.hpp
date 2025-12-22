// created by liuhan on 2024/2/1
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
#include <autoaim_utilities/NumberClassifier.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include "BaseDetector.hpp"
#include "autoaim_interfaces/msg/debug_armors.hpp"
#include "autoaim_interfaces/msg/debug_lights.hpp"
#include "autoaim_utilities/Armor.hpp"

namespace helios_cv
{

struct TraditionalArmorParams : public BaseParams
{
  double number_classifier_thresh;
  double binary_threshold;
  typedef struct LightParams
  {
    double min_ratio;
    double max_ratio;
    double max_angle;
  } LightParams;
  LightParams light_params;
  typedef struct ArmorParams
  {
    double min_light_ratio;
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    double max_angle;
  } ArmorParams;
  ArmorParams armor_params;

  TraditionalArmorParams() = default;
};

struct TraditionalArmorDebugImage : public DebugImages
{
  cv::Mat number_img;
  cv::Mat binary_img;
  cv::Mat result_img;
};

struct TraditionalArmorDebugInfo : public DebugInfos
{
  autoaim_interfaces::msg::DebugArmors debug_armors;
  autoaim_interfaces::msg::DebugLights debug_lights;
};

class TraditionalArmorDetector : public BaseDetector
{
public:
  TraditionalArmorDetector(const TraditionalArmorParams& params);

  ~TraditionalArmorDetector() = default;

  std::shared_ptr<Armors> detect_armors(std::shared_ptr<Image> image) final;

  void set_params(void* params) final;

  DebugImages* get_debug_images() final;

  DebugInfos* get_debug_infos() final;

private:
  cv::Mat preprocessImage(const cv::Mat& input);

  std::vector<Light> findLights(const cv::Mat& rbg_img, const cv::Mat& binary_img);

  std::vector<Armor> matchLights(const std::vector<Light>& lights);

  void draw_results(const std::vector<Armor>& armors);

  bool isLight(const Light& possible_light);

  bool containLight(const Light& light_1, const Light& light_2, const std::vector<Light>& lights);

  ArmorType isArmor(const Light& light_1, const Light& light_2);

  void get_all_number_images(const std::vector<Armor>& armors);

  TraditionalArmorParams params_;

  TraditionalArmorDebugImage debug_images_;
  TraditionalArmorDebugInfo debug_infos_;

  std::shared_ptr<NumberClassifier> number_classifier_;

  std::vector<Light> lights_;

  rclcpp::Logger logger_ = rclcpp::get_logger("TraditionalArmorDetector");
};

}  // namespace helios_cv