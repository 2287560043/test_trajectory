#include "camera_imu_bridge/LogLevel.hpp"
#include <camera_imu_bridge/Syncer.hpp>
#include <fmt/format.h>
#include <thread>
namespace helios_cv {
Syncer::Syncer(
    std::function<void(const LogLevel, const std::string&)> log_callback,
    std::function<void(std::shared_ptr<CameraFrame>, std::shared_ptr<ImuFrame>)>
        process_and_publish_synced_frame_callback
):
    log_callback_(log_callback),
    process_and_publish_synced_frame_callback_(process_and_publish_synced_frame_callback) {
    process_and_publish_synced_frame_thread_ =
        std::thread { [this]() { processAndPublishSyncedFrameLoop(); } };
}

Syncer::~Syncer() {
    running_ = false;
    camera_imu_synced_index_ = 65534;
    camera_imu_synced_index_.notify_all();
    if (process_and_publish_synced_frame_thread_.joinable()) {
        process_and_publish_synced_frame_thread_.join();
    }
}

void Syncer::setParams(const SyncerParams& params) {
    params_ = params;
}

void Syncer::onImuFrame(std::shared_ptr<ImuFrame> imu_frame, int frames_since_trigger) {
    if (frames_since_trigger == params_.imu_frame_since_trigger) {
        imu_frame->id = imu_frame_id_;
        imu_frames_[imu_frame_id_] = imu_frame;
        ++imu_frame_id_;
    }
}

void Syncer::onCameraFrame(std::shared_ptr<CameraFrame> camera_frame) {
    camera_frame->id = camera_frame_id_;
    camera_frames_[camera_frame_id_] = camera_frame;
    if (sync_ready_) {
        if (camera_frame_id_ > 11) {
            trySync();
        }
    } else {
        processAndPublishSyncedFrame(camera_frame_id_ % 10);
    }
    ++camera_frame_id_;
    checkStatus();
}

template<typename... Args>
void Syncer::log(const LogLevel log_level, const std::string& fmt, Args... args) {
    if (log_callback_)
        log_callback_(log_level, fmt::format(fmt::runtime(fmt), args...));
}

void Syncer::trySync() {
    while (offset_ < 6) {
        int calculate_count { 0 };
        int time_sum { 0 };
        for (int i = 0; i < 10; ++i) {
            if (imu_frames_[camera_frames_[i]->id - offset_]
                && imu_frames_[camera_frames_[i]->id - offset_]->id
                    == camera_frames_[i]->id - offset_)
            {
                time_sum +=
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        camera_frames_[i]->time - imu_frames_[camera_frames_[i]->id - offset_]->time
                    )
                        .count();
                ++calculate_count;
            }
        }
        auto average_time = time_sum / calculate_count;
        if (calculate_count != 0 && average_time > params_.camera_imu_time_difference_us.min && average_time < params_.camera_imu_time_difference_us.max) {
            sync_ready_ = true;
            return;
        }
        ++offset_;
    }
    log(LogLevel::Warn, "No suitable offset matched");
}

void Syncer::checkStatus() {
    if (camera_frame_id_ > imu_frame_id_ && camera_frame_id_ - imu_frame_id_ > 5) {
        camera_frame_id_ = 0;
        imu_frame_id_ = 0;
        sync_ready_ = false;
        offset_ = -5;
        log(LogLevel::Warn, "Imu data missing");
        return;
    }
    if (camera_frame_id_ < imu_frame_id_ && imu_frame_id_ - camera_frame_id_ > 5) {
        camera_frame_id_ = 0;
        imu_frame_id_ = 0;
        sync_ready_ = false;
        offset_ = -5;
        log(LogLevel::Warn, "Imu data missing");
        return;
    }
    if (camera_frame_id_ % 100 == 0) {
        int calculate_count { 0 };
        int time_sum { 0 };
        for (int i = 0; i < 10; ++i) {
            if (imu_frames_[camera_frames_[i]->id - offset_]
                && imu_frames_[camera_frames_[i]->id - offset_]->id
                    == camera_frames_[i]->id - offset_)
            {
                time_sum +=
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        camera_frames_[i]->time - imu_frames_[camera_frames_[i]->id - offset_]->time
                    )
                        .count();
                ++calculate_count;
            }
        }
        auto average_time = time_sum / calculate_count;
        if (calculate_count != 0 && average_time > params_.camera_imu_time_difference_us.min && average_time < params_.camera_imu_time_difference_us.max) {
        } else {
            camera_frame_id_ = 0;
            imu_frame_id_ = 0;
            sync_ready_ = false;
            offset_ = -5;
            log(LogLevel::Warn, "Delay happened");
            return;
        }
    }
}

void Syncer::processAndPublishSyncedFrame(int cameraFrameIndex) {
    auto camera_id = camera_frames_[cameraFrameIndex]->id;
    auto imu_id = camera_id - offset_;
    if (camera_id == camera_frame_id_ && imu_frames_[imu_id]->id == camera_id - offset_) {
        auto imu_frame_index = imu_id % 10;
        camera_imu_synced_index_ = (static_cast<uint16_t>(cameraFrameIndex) << 8) | imu_frame_index;
        camera_imu_synced_index_.notify_one();
    }
};
void Syncer::processAndPublishSyncedFrameLoop() {
    uint16_t camera_imu_index_old = 65535;
    while (running_) {
        camera_imu_synced_index_.wait(camera_imu_index_old);
        if (sync_ready_) {
            camera_imu_index_old = camera_imu_synced_index_;
            auto camera_frame_index = (static_cast<uint8_t>(camera_imu_index_old >> 8) + 0) % 10;
            auto imu_frame_index = static_cast<uint8_t>(camera_imu_index_old & 0xFF);
            process_and_publish_synced_frame_callback_(
                camera_frames_[camera_frame_index],
                imu_frames_[imu_frame_index]
            );
        }
    }
}

} // namespace helios_cv
