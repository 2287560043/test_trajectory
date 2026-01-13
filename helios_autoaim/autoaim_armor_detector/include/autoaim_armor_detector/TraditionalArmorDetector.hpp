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
#include "armor_detector_parameters.hpp"
#include "autoaim_armor_detector/BaseDetector.hpp"
#include "autoaim_interfaces/msg/detail/debug_armors__struct.hpp"
#include "autoaim_interfaces/msg/detail/debug_lights__struct.hpp"
#include "autoaim_utilities/Armor.hpp"
#include <autoaim_utilities/NumberClassifier.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>

namespace helios_cv {


struct TraditionalArmorDebugImage: public DebugImages {
    cv::Mat number_img;
    cv::Mat binary_img;
    cv::Mat result_img;
};

struct TraditionalArmorDebugInfo: public DebugInfos {
    autoaim_interfaces::msg::DebugArmors debug_armors;
    autoaim_interfaces::msg::DebugLights debug_lights;
};

class TraditionalArmorDetector: public BaseDetector {
public:
    explicit TraditionalArmorDetector(const detector_node::Params& params);

    ~TraditionalArmorDetector() = default;

    std::vector<Armor> detect_armors(cv::Mat image) final;

    void updateParams(const detector_node::Params& params) final;

    cv::Mat get_debug_image() final;

private:
    cv::Mat preprocessImage(const cv::Mat& input);

    std::vector<Light> findLights(const cv::Mat& rbg_img, const cv::Mat& binary_img);

    std::vector<Armor> matchLights(const std::vector<Light>& lights);

    void draw_results(const std::vector<Armor>& armors);

    bool isLight(const Light& possible_light);

    bool containLight(const Light& light_1, const Light& light_2, const std::vector<Light>& lights);

    ArmorType isArmor(const Light& light_1, const Light& light_2);

    void get_all_number_images(const std::vector<Armor>& armors);

     detector_node::Params params_;

    TraditionalArmorDebugImage debug_images_;
    TraditionalArmorDebugInfo debug_infos_;

    std::shared_ptr<NumberClassifier> number_classifier_;

    std::vector<Light> lights_;

    rclcpp::Logger logger_ = rclcpp::get_logger("TraditionalArmorDetector");
};

} // namespace helios_cv
