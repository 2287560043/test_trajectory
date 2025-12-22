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

#include <cstdint>
#include <memory>
#include "detectors/BaseDetector.hpp"
#include "detectors/TraditionalArmorDetector.hpp"
#include "detectors/OVNetArmorEnergyDetector.hpp"
#include "detectors/TraditionalEnergyDetector.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <detector_node/detector_node_parameters.hpp>

namespace helios_cv
{

class DetectorFactory
{
public:
  std::unique_ptr<BaseDetector> create_traditional_armor_detector(const detector_node::Params& params);

  std::unique_ptr<BaseDetector> create_ovnet_armor_energy_detector(const detector_node::Params& params);

  std::unique_ptr<BaseDetector> create_traditional_energy_detector(const detector_node::Params& params);
};

void param_convert(const detector_node::Params& params, TraditionalArmorParams& armor_params);

void param_convert(const detector_node::Params& params, OVNetArmorEnergyDetectorParams& armor_params);

void param_convert(const detector_node::Params& params, TraditionalEnergyParams& armor_params);

template <typename DetectorParams>
void set_detector_params(const detector_node::Params& params, std::shared_ptr<BaseDetector> detector)
{
  DetectorParams detector_params;
  param_convert(params, detector_params);
  detector->set_params(&detector_params);
}

}  // namespace helios_cv