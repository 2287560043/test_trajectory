// created by liuhan on 2024/4/6
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
#include <exception>
#include <memory>
#include <any>
#include <opencv2/core/quaternion.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "autoaim_interfaces/msg/armors.hpp"

#include "BaseDetector.hpp"
#include "OVNetArmorEnergyDetector.hpp"
#include "TraditionalArmorDetector.hpp"
#include "TraditionalEnergyDetector.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "autoaim_utilities/PnPSolver.hpp"

#define DECLARE_INTERFACE(interface)                                                                                   \
private:                                                                                                               \
  interface() = delete;                                                                                                \
  interface(const interface&) = delete;                                                                                \
  interface& operator=(const interface&) = delete;

namespace helios_cv
{

/**
 * @brief DetectStream is a class that can detect target from a stream of images.
 *        The reason this class is written is the process of detecting armors is same
 *        for armor and energy, and we may use this base class when we want to detect
 *        other new targets.
 */
class DetectStream
{
public:
  virtual ~DetectStream() = default;
};

class ArmorEnergyDetectStream : public DetectStream
{
  DECLARE_INTERFACE(ArmorEnergyDetectStream)
public:
  static autoaim_interfaces::msg::Armors detect(std::shared_ptr<BaseDetector> detector,
                                                std::shared_ptr<PnPSolver> pnp_solver, 
                                                std::shared_ptr<Image> image,
                                                void* extra_param = nullptr);

  template <typename DebugImagesType>
  static const DebugImagesType& get_debug_images(std::shared_ptr<BaseDetector> detector)
  {
    auto debug_images = detector->get_debug_images();
    if (debug_images)
    {
      return *dynamic_cast<DebugImagesType*>(debug_images);
    }
    else
    {
      DebugImagesType empty_debug_images;
      return empty_debug_images;
    }
  }

  template <typename DebugInfosType>
  static const DebugInfosType& get_debug_infos(std::shared_ptr<BaseDetector> detector)
  {
    auto debug_infos = detector->get_debug_infos();
    if (debug_infos)
    {
      return *dynamic_cast<DebugInfosType*>(debug_infos);
    }
    else
    {
      DebugInfosType empty_debug_infos;
      return empty_debug_infos;
    }
  }

private:
  static inline rclcpp::Logger logger_ = rclcpp::get_logger("ArmorEnergyDetectStream");
};

}  // namespace helios_cv
