// created by liuhan on 2024/8/16
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */

#include "autoaim_armor_detector/ArmorDetectorFactory.hpp"

#include "autoaim_armor_detector/TraditionalArmorDetector.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logger.hpp>

namespace helios_cv {

std::unique_ptr<BaseDetector>
DetectorFactory::createTraditionalArmorDetector(const detector_node::Params& params) {
    return std::make_unique<TraditionalArmorDetector>(TraditionalArmorParams {
        BaseParams { .is_blue = static_cast<bool>(params.is_blue),
                     .autoaim_mode = static_cast<int>(params.autoaim_mode),
                     .debug = params.debug },
        params.armor.traditional.number_classifier_threshold,
        params.armor.traditional.binary_thres,
        TraditionalArmorParams::LightParams {
            .min_ratio = params.armor.traditional.light.min_ratio,
            .max_ratio = params.armor.traditional.light.max_ratio,
            .max_angle = params.armor.traditional.light.max_angle,
        },
        TraditionalArmorParams::ArmorParams {
            .min_light_ratio = params.armor.traditional.armor.min_light_ratio,
            .min_small_center_distance = params.armor.traditional.armor.min_small_center_distance,
            .max_small_center_distance = params.armor.traditional.armor.max_small_center_distance,
            .min_large_center_distance = params.armor.traditional.armor.min_large_center_distance,
            .max_large_center_distance = params.armor.traditional.armor.max_large_center_distance,
            .max_angle = params.armor.traditional.armor.max_angle,
        } });
}

std::unique_ptr<BaseDetector>
DetectorFactory::createOvnetArmorDetector(const detector_node::Params& params) {
    return std::make_unique<OVNetArmorEnergyDetector>(OVNetArmorEnergyDetectorParams {
        BaseParams { .is_blue = static_cast<bool>(params.is_blue),
                     .autoaim_mode = static_cast<int>(params.autoaim_mode),
                     .debug = params.debug },
        params.armor.net.net_classifier_threshold,
        params.armor.traditional.armor.min_large_center_distance,
        OVNetArmorEnergyDetectorParams::OVNetParams {
            .MODEL_PATH = ament_index_cpp::get_package_share_directory(
                              "detect"
                              "ors"
                          )
                + "/model/" + params.armor.net.model_path,
            .NUM_CLASS = static_cast<int>(params.armor.net.num_class),
            .NUM_COLORS = static_cast<int>(params.armor.net.num_colors),
            .NMS_THRESH = static_cast<float>(params.armor.net.nms_thresh),
            .NUM_APEX = static_cast<int>(params.armor.net.num_apex),
            .POOL_NUM = static_cast<int>(params.armor.net.pool_num) } });
}

void paramConvert(const detector_node::Params& params, TraditionalArmorParams& armor_params) {
    armor_params.is_blue = params.is_blue;
    armor_params.autoaim_mode = params.autoaim_mode;
    armor_params.debug = params.debug;
    armor_params.armor_params.max_angle = params.armor.traditional.armor.max_angle;
    armor_params.armor_params.max_large_center_distance =
        params.armor.traditional.armor.max_large_center_distance;
    armor_params.armor_params.max_small_center_distance =
        params.armor.traditional.armor.max_small_center_distance;
    armor_params.armor_params.min_large_center_distance =
        params.armor.traditional.armor.min_large_center_distance;
    armor_params.armor_params.min_light_ratio = params.armor.traditional.armor.min_light_ratio;
    armor_params.armor_params.min_small_center_distance =
        params.armor.traditional.armor.min_small_center_distance;
    armor_params.light_params.max_angle = params.armor.traditional.light.max_angle;
    armor_params.light_params.max_ratio = params.armor.traditional.light.max_ratio;
    armor_params.light_params.min_ratio = params.armor.traditional.light.min_ratio;
    armor_params.number_classifier_thresh = params.armor.traditional.number_classifier_threshold;
    armor_params.binary_threshold = params.armor.traditional.binary_thres;
}

void paramConvert(
    const detector_node::Params& params,
    OVNetArmorEnergyDetectorParams& armor_params
) {
    armor_params.is_blue = params.is_blue;
    armor_params.autoaim_mode = params.autoaim_mode;
    armor_params.debug = params.debug;
    armor_params.net_params.MODEL_PATH = ament_index_cpp::get_package_share_directory("detectors")
        + "/model/" + params.armor.net.model_path;
    armor_params.net_params.NMS_THRESH = params.armor.net.nms_thresh;
    armor_params.net_params.NUM_APEX = params.armor.net.num_apex;
    armor_params.net_params.NUM_CLASS = params.armor.net.num_class;
    armor_params.net_params.NUM_COLORS = params.armor.net.num_colors;
    armor_params.net_params.POOL_NUM = params.armor.net.pool_num;
    armor_params.classifier_threshold = params.armor.net.net_classifier_threshold;
    armor_params.min_large_center_distance =
        params.armor.traditional.armor.min_large_center_distance;
}

} // namespace helios_cv
