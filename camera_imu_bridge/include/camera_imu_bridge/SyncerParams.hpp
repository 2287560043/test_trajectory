#pragma once
struct SyncerParams {
    struct CameraImuTimeDifferenceUs {
        int min { 4000 };
        int max { 8000 };
    };
    CameraImuTimeDifferenceUs camera_imu_time_difference_us;
    int imu_frame_since_trigger { 0 };
};
