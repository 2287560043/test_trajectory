#pragma once
#include "Syncer.hpp"
#include <atomic>
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <cstdint>
class Syncer;
class Imu {
private:
    std::atomic<bool> sync_enabled { true };
    std::atomic<bool> utc_synced { false };
    std::atomic<bool> imu_ready { false }; // IMU就绪标志

    std::atomic<size_t> sync_count { 0 };
    std::chrono::high_resolution_clock::time_point lastTime;
    std::atomic<size_t> frameCount { 0 };

    boost::asio::io_context io_service;
    boost::asio::serial_port serial;

    std::thread io_thread;
    std::thread sync_thread;

    std::vector<uint8_t> read_buffer;
    std::vector<uint8_t> parse_buffer;
    size_t parse_pos = 0;
    std::atomic<size_t> frame_count { 0 };

    // 用于统计 trigger 间隔
    std::atomic<bool> last_was_trigger { false };
    std::atomic<size_t> frames_since_trigger { 0 };
    std::atomic<size_t> trigger_count { 0 };
    std::shared_ptr<Syncer> syncer;

    uint8_t calc_nmea_checksum(const std::string& sentence);

    static void calculate_crc16(uint16_t* crc, const uint8_t* buf, uint32_t len);
    static uint16_t read_u16(const uint8_t* p);
    void set_rts_physical(bool physical_high);
    std::string generate_gprmc();
    uint32_t get_utc_ms_of_day();
    std::string ms_to_utc_time(uint32_t total_ms);
    std::string generate_gprmc_for_time(std::chrono::system_clock::time_point target_time);
    void sync_loop();
    bool validate_and_extract(const uint8_t* frame, uint16_t payload_len);
    void parse_data(const uint8_t* data, size_t len);

public:
    Imu(std::shared_ptr<Syncer> _syncer);
    bool open(const std::string& port);
    void start_read();
    ~Imu();
};
