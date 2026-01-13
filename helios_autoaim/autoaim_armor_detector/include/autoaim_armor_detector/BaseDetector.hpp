// created by liuhan on 2024/8/13
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
#include "autoaim_utilities/Armor.hpp"
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <utility>

namespace helios_cv {

struct BaseParams {
public:
    bool is_blue;
    int autoaim_mode;
    bool debug;

    // virtual ~BaseParams() = default;
};

struct Image {
    Image(cv::Mat&& image): image(std::forward<cv::Mat>(image)) {}

    Image() = default;

    cv::Mat image;

    virtual ~Image() = default;
};

struct DebugImages {
    using SharedPtr = std::shared_ptr<DebugImages>;

    virtual ~DebugImages() = default;
};

struct DebugInfos {
    using SharedPtr = std::shared_ptr<DebugInfos>;

    virtual ~DebugInfos() = default;
};

class BaseDetector {
public:
    virtual std::vector<Armor> detect_armors(cv::Mat image) = 0;

    virtual void updateParams(const detector_node::Params& params) = 0;

    virtual cv::Mat get_debug_image() = 0;
    virtual ~BaseDetector() = default;

protected:
    BaseDetector() = default;
    // RAII Class need to delete copy constructor and copy assignment operator
    // and move constructor
    BaseDetector(const BaseDetector&) = delete;

    BaseDetector& operator=(const BaseDetector&) = delete;

    BaseDetector(BaseDetector&&) = delete;
};

} // namespace helios_cv
