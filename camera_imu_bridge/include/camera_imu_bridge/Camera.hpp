#pragma once
#include "CameraDefine.h"
#include <camera_imu_bridge/CameraFrame.hpp>
#include <camera_imu_bridge/CameraParams.hpp>
#include <camera_imu_bridge/LogLevel.hpp>
#include <fmt/format.h>
#include <functional>
#include <memory>
#include <string>
#include <thread>
namespace helios_cv {
    class Camera {
    public:
        explicit Camera(
            std::function<void(const LogLevel, const std::string&)> logCallback,
            std::function<void(std::shared_ptr<CameraFrame>)> frameCallback
        );
        void processFrame(std::shared_ptr<CameraFrame> cameraFrame, BYTE* output);
        void setParams(const cameraParams& params);
        ~Camera();

    private:
        bool initCamera();
        template<typename T>
        T clampToRange(const std::string& name, T val, T min, T max);
        void setupFPSTimer();
        bool startCamera();
        static void frameCallback(CameraHandle hCam, BYTE* buf, tSdkFrameHead* head, PVOID ctx);
        template<typename... Args>
        void log(const LogLevel logLevel, const std::string& fmt, Args... args);

        std::function<void(const LogLevel, const std::string&)> logCallback_;
        std::function<void(std::shared_ptr<CameraFrame>)> frameCallback_;
        int h_camera_ { 0 };
        tSdkCameraCapbility capability_;
        int img_width_ { 0 }, img_height_ { 0 };
        cameraParams params_;
        std::atomic<bool> run_fps_timer_ { 0 };
        std::thread fps_timer_;
        std::atomic<uint32_t> frame_cnt_ { 0 };
    };
}
