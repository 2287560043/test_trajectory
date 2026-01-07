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
#pragma once

#include "autoaim_armor_detector/TraditionalArmorDetector.hpp"
// #include "autoaim_armor_detector/BaseDetector.hpp"
#include "autoaim_armor_detector/OVNetArmorEnergyDetector.hpp"
// #include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <armor_detector_parameters.hpp>
namespace helios_cv {

class DetectorFactory {
public:
    std::unique_ptr<BaseDetector>
    createTraditionalArmorDetector(const detector_node::Params& params);

    std::unique_ptr<BaseDetector>
    createOvnetArmorDetector(const detector_node::Params& params);
};

void param_convert(const detector_node::Params& params, TraditionalArmorParams& armor_params);

void param_convert(
    const detector_node::Params& params,
    OVNetArmorEnergyDetectorParams& armor_params
);

template<typename DetectorParams>
void setDetectorParams(
    const detector_node::Params& params,
    std::shared_ptr<BaseDetector> detector
) {
    DetectorParams detector_params;
    param_convert(params, detector_params);
    detector->set_params(&detector_params);
}

} // namespace helios_cv
