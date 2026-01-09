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

    virtual void set_params(void* params) = 0;

    virtual void set_cam_info(const sensor_msgs::msg::CameraInfo& cam_info) {
        cam_info_ = cam_info;
        cam_center_ = cv::Point2f(cam_info_.width / 2.0, cam_info_.height / 2.0);
    };

    virtual cv::Mat get_debug_image() = 0;

    virtual ~BaseDetector() = default;

protected:
    BaseDetector() = default;
    // RAII Class need to delete copy constructor and copy assignment operator
    // and move constructor
    BaseDetector(const BaseDetector&) = delete;

    BaseDetector& operator=(const BaseDetector&) = delete;

    BaseDetector(BaseDetector&&) = delete;

    sensor_msgs::msg::CameraInfo cam_info_;

    cv::Point2f cam_center_;
};

} // namespace helios_cv
