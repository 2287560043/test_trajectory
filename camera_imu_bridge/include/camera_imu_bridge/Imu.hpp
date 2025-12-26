#pragma once
#include <camera_imu_bridge/ImuFrame.hpp>
#include <camera_imu_bridge/LogLevel.hpp>
#include <cstdint>
#include <functional>
#include <memory>
namespace helios_cv {
class Imu {
protected:
    Imu(std::function<void(const LogLevel, const std::string&)> logCallback,
        std::function<void(std::shared_ptr<ImuFrame>,int frames_since_trigger)> frameCallback){};
    virtual ~Imu() = 0;
};
inline Imu::~Imu() {}
} // namespace helios_cv
