#include <CameraApi.h>
#include <CameraDefine.h>
#include <CameraStatus.h>
#include <atomic>
#include <camera_imu_bridge/Camera.hpp>
#include <camera_imu_bridge/CameraFrame.hpp>
#include <camera_imu_bridge/CameraParams.hpp>
#include <camera_imu_bridge/LogLevel.hpp>
#include <fmt/core.h>
#include <fmt/format.h>
#include <functional>
#include <memory>
#include <thread>
namespace helios_cv {
Camera::Camera(
    std::function<void(const LogLevel, const std::string&)> log_callback,
    std::function<void(std::shared_ptr<CameraFrame>)> frame_callback
):
    log_callback_(log_callback),
    frame_callback_(frame_callback) {
    log(LogLevel::Info, "Starting MVCameraNode");

    if (!initCamera()) {
        throw std::runtime_error("Camera initialization failed");
    }

    if (!startCamera()) {
        throw std::runtime_error("Failed to start camera");
    }
    log(LogLevel::Info, "Camera started successfully!");
}

void Camera::processFrame(std::shared_ptr<CameraFrame> camera_frame, BYTE* output) {
    CameraImageProcessEx(
        h_camera_,
        camera_frame->buffer,
        output,
        &camera_frame->head,
        CAMERA_MEDIA_TYPE_BGR8,
        0
    );
}

void Camera::setParams(const CameraParams& params) {
    CameraSetTriggerMode(h_camera_, 2);
    params_ = params;
    log(LogLevel::Info, "===  Set all params ===");
    {
        int val = clampToRange(
            "exposure_time",
            (int)params.exposure_time,
            (int)capability_.sExposeDesc.uiExposeTimeMin,
            (int)capability_.sExposeDesc.uiExposeTimeMax
        );
        auto status = CameraSetExposureTime(h_camera_, val);
        log(LogLevel::Info,
            "  exposure={}, status={}",
            val,
            status == CAMERA_STATUS_SUCCESS ? "OK" : CameraGetErrorString(status));
    }
    {
        int val = clampToRange(
            "analog_gain",
            (int)params.analog_gain,
            (int)capability_.sExposeDesc.uiAnalogGainMin,
            (int)capability_.sExposeDesc.uiAnalogGainMax
        );
        auto status = CameraSetAnalogGain(h_camera_, val);
        log(LogLevel::Info,
            "  gain={}, status={}",
            val,
            status == CAMERA_STATUS_SUCCESS ? "OK" : CameraGetErrorString(status));
    }
    {
        int val = clampToRange(
            "saturation",
            (int)params.saturation,
            capability_.sSaturationRange.iMin,
            capability_.sSaturationRange.iMax
        );
        auto status = CameraSetSaturation(h_camera_, val);
        log(LogLevel::Info,
            "  saturation={}, status={}",
            val,
            status == CAMERA_STATUS_SUCCESS ? "OK" : CameraGetErrorString(status));
    }
    {
        int val = clampToRange(
            "gamma",
            (int)params.gamma,
            capability_.sGammaRange.iMin,
            capability_.sGammaRange.iMax
        );
        auto status = CameraSetGamma(h_camera_, val);
        log(LogLevel::Info,
            "  gamma={}, status={}",
            val,
            status == CAMERA_STATUS_SUCCESS ? "OK" : CameraGetErrorString(status));
    }
    {
        int r = clampToRange(
            "rgb_r",
            (int)params.rgb_gain.r,
            capability_.sRgbGainRange.iRGainMin,
            capability_.sRgbGainRange.iRGainMax
        );
        int g = clampToRange(
            "rgb_g",
            (int)params.rgb_gain.g,
            capability_.sRgbGainRange.iGGainMin,
            capability_.sRgbGainRange.iGGainMax
        );
        int b = clampToRange(
            "rgb_b",
            (int)params.rgb_gain.b,
            capability_.sRgbGainRange.iBGainMin,
            capability_.sRgbGainRange.iBGainMax
        );
        auto status = CameraSetGain(h_camera_, r, g, b);
        log(LogLevel::Info,
            "  RGB=({},{},{}), status={}",
            r,
            g,
            b,
            status == CAMERA_STATUS_SUCCESS ? "OK" : CameraGetErrorString(status));
    }
    {
        bool val = params.rotate;
        int rot_val = val ? 2 : 0;
        auto status = CameraSetRotate(h_camera_, rot_val);
        log(LogLevel::Info,
            "  rotate={}, status={}",
            val ? "180°" : "0°",
            status == CAMERA_STATUS_SUCCESS ? "OK" : CameraGetErrorString(status));
    }
    {
        bool val = params.print_fps;
        if (val) {
            if (!run_fps_timer_) {
                setupFpsTimer();
            }
        } else {
            run_fps_timer_ = false;
            if (fps_timer_.joinable()) {
                fps_timer_.join();
            }
        }
    }
}

Camera::~Camera() {
    run_fps_timer_ = false;
    if (fps_timer_.joinable()) {
        fps_timer_.join();
    }
    if (h_camera_) {
        CameraStop(h_camera_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        CameraSetCallbackFunction(h_camera_, nullptr, nullptr, nullptr);
        CameraUnInit(h_camera_);
    }
}

bool Camera::initCamera() {
    CameraSdkInit(1);

    int count = 1;
    tSdkCameraDevInfo dev_info;
    if (CameraEnumerateDevice(&dev_info, &count) != CAMERA_STATUS_SUCCESS || count == 0) {
        log(LogLevel::Error, "No camera found");
        return false;
    }

    log(LogLevel::Info, "Found: {} [{}]", dev_info.acProductName, dev_info.acSn);

    if (auto status = CameraInit(&dev_info, PARAM_MODE_BY_SN, 0, &h_camera_);
        status != CAMERA_STATUS_SUCCESS)
    {
        log(LogLevel::Error, "Camera init failed: {}", CameraGetErrorString(status));
        return false;
    }

    CameraGetCapability(h_camera_, &capability_);

    tSdkImageResolution res;
    CameraGetImageResolution(h_camera_, &res);
    img_width_ = res.iWidth;
    img_height_ = res.iHeight;

    // 设置按SN码加载参数，SDK会自动查找对应配置文件
    CameraSetParameterMode(h_camera_, PARAM_MODE_BY_SN);
    log(LogLevel::Info, "Parameter mode: BY_SN (auto-load config based on SN)");

    CameraSetAeState(h_camera_, false);
    CameraSetTriggerMode(h_camera_, 0);

    log(LogLevel::Info,

        "Exposure range: [{}, {}]μs",
        capability_.sExposeDesc.uiExposeTimeMin,
        capability_.sExposeDesc.uiExposeTimeMax);
    log(LogLevel::Info,
        "Gain range: [{}, {}]",
        capability_.sExposeDesc.uiAnalogGainMin,
        capability_.sExposeDesc.uiAnalogGainMax);

    setParams(params_);
    double exp_time;
    int gain, r, g, b, sat, gam;
    CameraGetExposureTime(h_camera_, &exp_time);
    CameraGetAnalogGain(h_camera_, &gain);
    CameraGetGain(h_camera_, &r, &g, &b);
    CameraGetSaturation(h_camera_, &sat);
    CameraGetGamma(h_camera_, &gam);

    log(LogLevel::Info, "=== Active Camera Parameters ===");
    log(LogLevel::Info, "  Resolution: {}x{}", img_width_, img_height_);
    log(LogLevel::Info, "  Exposure: {} μs", exp_time);
    log(LogLevel::Info, "  Analog Gain: {}", gain);
    log(LogLevel::Info, "  RGB Gain: R={} G={} B={}", r, g, b);
    log(LogLevel::Info, "  Saturation: {}", sat);
    log(LogLevel::Info, "  Gamma: {}", gam);
    log(LogLevel::Info, "  Rotate: {}", params_.rotate ? "180°" : "0°");
    log(LogLevel::Info, "================================");

    log(LogLevel::Info, "Camera ready: {}x{}", img_width_, img_height_);
    return true;
}

template<typename T>
T Camera::clampToRange(const std::string& name, T val, T min, T max) {
    if (val < min) {
        log(LogLevel::Warn, "{}={} < min({}), clamped", name.c_str(), (int)val, (int)min);
        return min;
    }
    if (val > max) {
        log(LogLevel::Warn, "{}={} > max({}), clamped", name.c_str(), (int)val, (int)max);
        return max;
    }
    return val;
}

void Camera::setupFpsTimer() {
    run_fps_timer_ = true;
    fps_timer_ = std::thread { [this]() {
        while (run_fps_timer_) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            uint32_t cnt = frame_cnt_.exchange(0);
            log(LogLevel::Info, "FPS: {}", cnt / 2.0);
        }
    } };
}

bool Camera::startCamera() {
    if (CameraSetCallbackFunction(h_camera_, frameCallback, this, nullptr) != CAMERA_STATUS_SUCCESS)
    {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    return CameraPlay(h_camera_) == CAMERA_STATUS_SUCCESS;
}

void Camera::frameCallback(CameraHandle hCam, BYTE* buf, tSdkFrameHead* head, PVOID ctx) {
    auto* cam = static_cast<Camera*>(ctx);
    if (!cam || !buf || !head)
        return;

    auto now = std::chrono::system_clock::now();
    uint time_now_low, time_now_high;
    CameraGetDevTimeStamp(hCam, &time_now_low, &time_now_high);

    uint64_t time_now_us = ((uint64_t)time_now_high << 32) | time_now_low;

    uint64_t exposure_end_time_us = static_cast<uint64_t>(head->uiTimeStamp) * 100;
    int64_t time_since_exposure_end_us =
        static_cast<int64_t>(time_now_us) - static_cast<int64_t>(exposure_end_time_us);

    auto exposure_start_time =
        now - std::chrono::microseconds(time_since_exposure_end_us + head->uiExpTime);

    auto exposure_start_time_t = std::chrono::system_clock::to_time_t(exposure_start_time);

    std::tm exposure_start_time_utc;

    gmtime_r(&exposure_start_time_t, &exposure_start_time_utc);
    uint32_t seconds_of_day = exposure_start_time_utc.tm_hour * 3600
        + exposure_start_time_utc.tm_min * 60 + exposure_start_time_utc.tm_sec;
    uint32_t milliseconds_of_day = seconds_of_day * 1000;
    auto ms_part = std::chrono::duration_cast<std::chrono::milliseconds>(
                       exposure_start_time.time_since_epoch()
                   )
                       .count()
        % 1000;

    uint32_t frame_timestamp_ms = milliseconds_of_day + ms_part;

    if (frame_timestamp_ms >= 86400000) {
        frame_timestamp_ms = frame_timestamp_ms % 86400000;
    }
    cam->frame_callback_(std::make_shared<CameraFrame>(head, buf, frame_timestamp_ms));
    ++cam->frame_cnt_;
}

template<typename... Args>
void Camera::log(const LogLevel log_level, const std::string& fmt, Args... args) {
    if (log_callback_)
        log_callback_(log_level, fmt::format(fmt::runtime(fmt), args...));
}
} // namespace helios_cv
