#pragma once
#include <atomic>
#include <camera_imu_bridge/Imu.hpp>
#include <ftd2xx.h>
#include <functional>
#include <memory>
#include <thread>
namespace helios_cv {
class ImuFtd2xx: public Imu {
public:
    ImuFtd2xx(
        std::function<void(const LogLevel, const std::string&)> log_callback,
        std::function<void(std::shared_ptr<ImuFrame>, int)> frame_callback
    );
    ~ImuFtd2xx();

private:
    template<typename... Args>
    void log(const LogLevel log_level, const std::string& fmt, Args... args);
    void watchdogLoop();
    void calculateCrc16(uint16_t* crc, const uint8_t* buf, uint32_t len);
    uint16_t readU16(const uint8_t* p);
    uint8_t calculateNmeaChecksum(const std::string& sentence);
    std::string generateGprmcTime(std::chrono::system_clock::time_point target_time);
    bool validateAndExtract(const uint8_t* frame, uint16_t payload_len);
    void parseData(const uint8_t* data, size_t len);
    bool deviceExists(const std::string& serial);
    void readLoop();
    void ppsSyncLoop();
    bool initDevice();
    bool tryReconnect();

    std::function<void(const LogLevel, const std::string&)> log_callback_;
    std::function<void(std::shared_ptr<ImuFrame>, int frames_since_trigger)> frame_callback_;
    std::atomic<size_t> frames_since_trigger_;
    FT_HANDLE ft_handle_ = nullptr;
    std::atomic<bool> running_ { true };
    std::atomic<bool> device_connected_ { false };
    std::vector<uint8_t> parse_buffer_;

    size_t parse_pos_ = 0;
    std::atomic<size_t> frame_count_ { 0 };
    std::atomic<bool> imu_ready_ { false };

    EVENT_HANDLE rx_event_;
    std::string target_serial_number_;
    std::thread read_th, pps_thread_, watchdog_thread_;
};
} // namespace helios_cv
