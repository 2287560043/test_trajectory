#pragma once
#include "camera_imu_bridge/LogLevel.hpp"
#include <atomic>
#include <camera_imu_bridge/CameraFrame.hpp>
#include <camera_imu_bridge/ImuFrame.hpp>
#include <camera_imu_bridge/RingArray.hpp>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
namespace helios_cv {
class Syncer {
public:
    Syncer(
        std::function<void(const LogLevel, const std::string&)> log_callback,
        std::function<void(std::shared_ptr<CameraFrame>, std::shared_ptr<ImuFrame>)>
            process_and_publish_synced_frame_callback
    );
    void onImuFrame(std::shared_ptr<ImuFrame> imu_frame, int frames_since_trigger);
    void onCameraFrame(std::shared_ptr<CameraFrame> camera_frame);
    ~Syncer();

private:
    template<typename... Args>
    void log(const LogLevel logLevel, const std::string& fmt, Args... args);
    void trySync();
    void checkStatus();
    void processAndPublishSyncedFrame(int cameraFrameIndex);
    void processAndPublishSyncedFrameLoop();
    uint64_t imu_frame_id_ { 0 };
    uint64_t camera_frame_id_ { 0 };
    RingArray<std::shared_ptr<ImuFrame>, 10> imu_frames_;
    RingArray<std::shared_ptr<CameraFrame>, 10> camera_frames_;
    int offset_ { -5 };
    std::atomic<bool> running_;
    std::atomic<bool> sync_ready_;
    std::atomic<uint16_t> camera_imu_synced_index_ { 65535 };
    std::thread process_and_publish_synced_frame_thread_;
    std::function<void(const LogLevel, const std::string&)> log_callback_;
    std::function<void(std::shared_ptr<CameraFrame>, std::shared_ptr<ImuFrame>)>
        process_and_publish_synced_frame_callback_;
};
} // namespace helios_cv
