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
#include <utility>

#include "BaseDetector.hpp"
#include "armor_detector_parameters.hpp"
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
    autoaim_interfaces::msg::Armors detect(cv::Mat image, void* extra_param = nullptr);

    void changeDetector(std::unique_ptr<BaseDetector> detector) {
        detector_ = std::move(detector);
    }
    void updateParams(const detector_node::Params& params) {
        detector_->updateParams(params);
    }
    cv::Mat get_debug_images() {
        return detector_->get_debug_image();
    }

private:
    std::unique_ptr<BaseDetector> detector_;
    std::unique_ptr<PnPSolver> pnp_solver_;
};

} // namespace helios_cv
