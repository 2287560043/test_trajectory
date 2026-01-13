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

std::unique_ptr<BaseDetector> createArmorDetector(const detector_node::Params& params) {
    if (params.use_traditional) {
        return std::make_unique<TraditionalArmorDetector>(params);
    } else {
        return std::make_unique<OVNetArmorEnergyDetector>(params);
    }
}

} // namespace helios_cv
