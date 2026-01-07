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
#include <autoaim_interfaces/msg/detail/armors__struct.hpp>
#include <memory>
#include <opencv2/core/quaternion.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "BaseDetector.hpp"
#include "autoaim_utilities/PnPSolver.hpp"
namespace helios_cv {
class ArmorDetectStream {
public:
    ArmorDetectStream(
        std::unique_ptr<BaseDetector> detector,
        std::unique_ptr<PnPSolver> pnp_solver
    ):
        detector_(std::move(detector)),
        pnp_solver_(std::move(pnp_solver)) {}
    autoaim_interfaces::msg::Armors
    detect(std::shared_ptr<Image> image, void* extra_param = nullptr);

    cv::Mat get_debug_images() {
        return detector_->get_debug_image();
    }

private:
    std::unique_ptr<BaseDetector> detector_;
    std::unique_ptr<PnPSolver> pnp_solver_;
};

} // namespace helios_cv
