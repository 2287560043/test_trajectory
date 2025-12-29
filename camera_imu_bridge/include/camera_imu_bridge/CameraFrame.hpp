#pragma once
#include <CameraDefine.h>
#include <chrono>
#include <cstring>
struct CameraFrame {
    std::chrono::high_resolution_clock::time_point time;
    uint64_t id;
    int64_t frame_timestamp_ms;
    tSdkFrameHead head;
    BYTE* buffer;
    CameraFrame(tSdkFrameHead* head, BYTE* buffer, int64_t frame_timestamp_ms):
        time(std::chrono::high_resolution_clock::now()),
        frame_timestamp_ms(frame_timestamp_ms),
        head(*head) {
        auto size = head->uBytes;
        this->buffer = new BYTE[size];
        std::memcpy(this->buffer, buffer, size);
    }
    ~CameraFrame() {
        delete[] buffer;
    }
    CameraFrame(const CameraFrame&) = delete;
    CameraFrame& operator=(const CameraFrame&) = delete;
};
