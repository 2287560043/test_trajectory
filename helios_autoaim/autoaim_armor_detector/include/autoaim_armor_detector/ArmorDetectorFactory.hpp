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

#include "autoaim_armor_detector/BaseDetector.hpp"
#include "autoaim_armor_detector/OVNetArmorEnergyDetector.hpp"
#include "autoaim_armor_detector/TraditionalArmorDetector.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <armor_detector_parameters.hpp>
namespace helios_cv {

std::unique_ptr<BaseDetector> createArmorDetector(const detector_node::Params& params);

} // namespace helios_cv
