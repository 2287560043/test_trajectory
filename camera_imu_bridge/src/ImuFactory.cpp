#include <camera_imu_bridge/ImuFactory.hpp>
#include <camera_imu_bridge/ImuFtd2xx.hpp>
#include <camera_imu_bridge/ImuSerial.hpp>

namespace helios_cv {
std::unique_ptr<Imu> createImu(
    std::string type,
    std::function<void(const LogLevel, const std::string&)> logCallback,
    std::function<void(std::shared_ptr<ImuFrame>, int)> frameCallback
) {
    if (type == "ftd2xx") {
        return std::make_unique<ImuFtd2xx>(logCallback, frameCallback);
    }
    if (type == "serial") {
        return std::make_unique<ImuSerial>(logCallback, frameCallback);
    }
    return nullptr;
}

} // namespace helios_cv
