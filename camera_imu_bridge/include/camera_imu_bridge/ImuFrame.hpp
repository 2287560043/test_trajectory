#pragma once
#include <chrono>
struct HI91Data {
    uint8_t tag;
    uint16_t status;
    int8_t temperature;
    float air_pressure;
    uint32_t system_time;
    float acc_b_x, acc_b_y, acc_b_z;
    float gyr_b_x, gyr_b_y, gyr_b_z;
    float mag_b_x, mag_b_y, mag_b_z;
    float roll, pitch, yaw;
    float q_w, q_x, q_y, q_z;
} __attribute__((packed));

struct ImuFrame {
    std::chrono::high_resolution_clock::time_point time;
    uint64_t id;
    uint8_t tag;
    uint16_t status;
    int8_t temperature;
    float air_pressure;
    uint32_t system_time;
    float acc_b_x, acc_b_y, acc_b_z;
    float gyr_b_x, gyr_b_y, gyr_b_z;
    float mag_b_x, mag_b_y, mag_b_z;
    float roll, pitch, yaw;
    float q_w, q_x, q_y, q_z;
    int64_t targetTime;
    ImuFrame(const HI91Data& _HI91Data):
        time(std::chrono::high_resolution_clock::now()),
        tag(_HI91Data.tag),
        status(_HI91Data.status),
        temperature(_HI91Data.temperature),
        air_pressure(_HI91Data.air_pressure),
        system_time(_HI91Data.system_time),
        acc_b_x(_HI91Data.acc_b_x),
        acc_b_y(_HI91Data.acc_b_y),
        acc_b_z(_HI91Data.acc_b_z),
        gyr_b_x(_HI91Data.gyr_b_x),
        gyr_b_y(_HI91Data.gyr_b_y),
        gyr_b_z(_HI91Data.gyr_b_z),
        mag_b_x(_HI91Data.mag_b_x),
        mag_b_y(_HI91Data.mag_b_y),
        mag_b_z(_HI91Data.mag_b_z),
        roll(_HI91Data.roll),
        pitch(_HI91Data.pitch),
        yaw(_HI91Data.yaw),
        q_w(_HI91Data.q_w),
        q_x(_HI91Data.q_x),
        q_y(_HI91Data.q_y),
        q_z(_HI91Data.q_z) {}
};
