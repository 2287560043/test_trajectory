#include <atomic>
#include <boost/asio.hpp>
#include <camera_imu_bridge/Imu.hpp>
#include <camera_imu_bridge/ImuFrame.hpp>
#include <camera_imu_bridge/LogLevel.hpp>
#include <chrono>
namespace helios_cv {
class ImuSerial: public Imu {
public:
    ImuSerial(
        std::function<void(const LogLevel, const std::string&)> logCallback,
        std::function<void(std::shared_ptr<ImuFrame>,int frames_since_trigger)> frameCallback
    );
     ~ImuSerial() override;

private:
    bool open(const std::string& port);
    void start_read();
    template<typename... Args>
    void log(const LogLevel logLevel, const std::string& fmt, Args... args);
    uint8_t calc_nmea_checksum(const std::string& sentence);
    static void calculate_crc16(uint16_t* crc, const uint8_t* buf, uint32_t len);
    static uint16_t read_u16(const uint8_t* p);
    void set_rts_physical(bool physical_high);
    std::string generate_gprmc();
    uint32_t get_utc_ms_of_day();
    std::string generate_gprmc_for_time(std::chrono::system_clock::time_point target_time);
    void sync_loop();
    bool validate_and_extract(const uint8_t* frame, uint16_t payload_len);
    void parse_data(const uint8_t* data, size_t len);

    std::function<void(const LogLevel, const std::string&)> logCallback_;
    std::function<void(std::shared_ptr<ImuFrame>,int frames_since_trigger)> frameCallback_;
    std::atomic<bool> sync_enabled_ { true };
    std::atomic<bool> utc_synced_ { false };
    std::atomic<bool> imu_ready_ { false };

    boost::asio::io_context io_service_;
    boost::asio::serial_port serial_;

    std::thread io_thread_;
    std::thread sync_thread_;

    std::vector<uint8_t> read_buffer_;
    std::vector<uint8_t> parse_buffer_;
    size_t parse_pos = 0;

    std::atomic<bool> last_was_trigger_ { false };
    std::atomic<size_t> frames_since_trigger_ { 0 };
    std::atomic<size_t> trigger_count_ { 0 };
};
} // namespace helios_cv
