#pragma once
#include <camera_imu_bridge/ImuFrame.hpp>
#include <camera_imu_bridge/LogLevel.hpp>
namespace helios_cv {
class Imu {
public:
    virtual ~Imu() = default;
};
} // namespace helios_cv
