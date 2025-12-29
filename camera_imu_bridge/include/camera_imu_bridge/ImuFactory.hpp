#pragma once
#include <camera_imu_bridge/Imu.hpp>
#include <functional>
#include <string>
#include <memory>
namespace helios_cv {
std::unique_ptr<Imu> createImu(
    std::string type,
    std::function<void(const LogLevel, const std::string&)> logCallback,
    std::function<void(std::shared_ptr<ImuFrame>, int frames_since_trigger)> frameCallback
);
}
