#include <camera_imu_bridge/LogLevel.hpp>
#include <camera_imu_bridge/Syncer.hpp>
#include <fmt/format.h>
#include <thread>
#include <iomanip>

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
std::string ms_to_utc_time(uint32_t total_ms) {
    uint32_t total_seconds = total_ms / 1000;
    uint32_t ms_part = total_ms % 1000;

    uint8_t hours = (total_seconds / 3600) % 24;
    uint8_t minutes = (total_seconds % 3600) / 60;
    uint8_t seconds = total_seconds % 60;

    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << (int)hours << ":" << std::setw(2) << (int)minutes
        << ":" << std::setw(2) << (int)seconds << "." << std::setw(3) << ms_part;
    return oss.str();
}
void Syncer::onCameraFrame(std::shared_ptr<CameraFrame> camera_frame) {
    camera_frame->id = camera_frame_id_;
    camera_frames_[camera_frame_id_] = camera_frame;
    if (sync_ready_) {

processAndPublishSyncedFrame(camera_frame_id_ % 10);
    } else {
        if (camera_frame_id_ > 11) {
            trySync();
        }
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
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        camera_frames_[i]->time - imu_frames_[camera_frames_[i]->id - offset_]->time
                    )
                        .count();
                ++calculate_count;
            }
        }
        auto average_time = time_sum / calculate_count;
        log(LogLevel::Info, "average: {}", average_time);
        if (calculate_count != 0 && average_time > params_.camera_imu_time_difference_us.min
            && average_time < params_.camera_imu_time_difference_us.max)
        {
            sync_ready_ = true;
            return;
        }
        ++offset_;
    }
    log(LogLevel::Warn, "No suitable offset matched");
    log(LogLevel::Warn, "min: {}, max: {}", params_.camera_imu_time_difference_us.min,
        params_.camera_imu_time_difference_us.max);
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
        if (calculate_count != 0 && average_time > params_.camera_imu_time_difference_us.min
            && average_time < params_.camera_imu_time_difference_us.max)
        {
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

void Syncer::processAndPublishSyncedFrame(int camera_frame_index) {
    auto camera_id = camera_frames_[camera_frame_index]->id;
    auto imu_id = camera_id - offset_;
    if (camera_id == camera_frame_id_ && imu_frames_[imu_id]->id == camera_id - offset_) {
        auto imu_frame_index = imu_id % 10;
        camera_imu_synced_index_ = (static_cast<uint16_t>(camera_frame_index) << 8) | imu_frame_index;
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
log(LogLevel::Info, "IMU Time:       {}",ms_to_utc_time(imu_frames_[imu_frame_index]->system_time));
log(LogLevel::Info, "Camera Time:    {}", ms_to_utc_time(camera_frames_[camera_frame_index]->frame_timestamp_ms));
log(LogLevel::Info, "cameraId:       {}", camera_frames_[camera_frame_index]->id);
log(LogLevel::Info, "imuId:          {}", imu_frames_[imu_frame_index]->id);
log(LogLevel::Info, "offset:         {}", offset_);  
log(LogLevel::Info, "time diffrence: {}", std::chrono::duration_cast<std::chrono::microseconds>(camera_frames_[camera_frame_index]->time - imu_frames_[imu_frame_index]->time).count()/1000.0);
log(LogLevel::Info, "-----------------------------------");
        }
    }
}

} // namespace helios_cv
