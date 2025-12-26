#pragma once
#include <chrono>
#include <CameraDefine.h>
#include <cstring>
struct CameraFrame {
    std::chrono::high_resolution_clock::time_point time_;
    uint64_t id_;
    int64_t frame_timestamp_ms_;
    tSdkFrameHead head_;
    BYTE* buffer_;
    CameraFrame(tSdkFrameHead* head, BYTE* buffer, int64_t frame_timestamp_ms):
        time_(std::chrono::high_resolution_clock::now()),
        frame_timestamp_ms_(frame_timestamp_ms),
        head_(*head) {
        auto size = head->uBytes;
        buffer = new BYTE[size];
        std::memcpy(buffer_, buffer, size);
    }
    ~CameraFrame() {
        delete[] buffer_;
    }
};
