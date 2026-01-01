#include <camera_imu_bridge/ImuFrame.hpp>
#include <camera_imu_bridge/ImuSerial.hpp>
#include <camera_imu_bridge/LogLevel.hpp>
#include <fmt/format.h>
#include <iomanip>
#include <memory>
namespace helios_cv {
void ImuSerial::calculateCrc16(uint16_t* crc, const uint8_t* buf, uint32_t len) {
    uint32_t crc_val = *crc;
    for (uint32_t j = 0; j < len; ++j) {
        uint32_t byte = buf[j];
        crc_val ^= byte << 8;
        for (uint32_t i = 0; i < 8; ++i) {
            uint32_t temp = crc_val << 1;
            if (crc_val & 0x8000)
                temp ^= 0x1021;
            crc_val = temp;
        }
    }
    *crc = crc_val;
}

uint16_t ImuSerial::readU16(const uint8_t* p) {
    uint16_t u;
    std::memcpy(&u, p, 2);
    return u;
}

void ImuSerial::setRtsPhysical(bool physical_high) {
    int fd = serial_.native_handle();
    int status;
    ioctl(fd, TIOCMGET, &status);

    if (physical_high) {
        status &= ~TIOCM_RTS;
    } else {
        status |= TIOCM_RTS;
    }
    ioctl(fd, TIOCMSET, &status);
}

uint8_t ImuSerial::calculateNmeaChecksum(const std::string& sentence) {
    uint8_t checksum = 0;
    for (size_t i = 1; i < sentence.length(); ++i) {
        checksum ^= sentence[i];
    }
    return checksum;
}

uint32_t ImuSerial::getUtcMillisecondsOfDay() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_utc;
    gmtime_r(&time_t_now, &tm_utc);

    uint32_t ms_of_day = (tm_utc.tm_hour * 3600 + tm_utc.tm_min * 60 + tm_utc.tm_sec) * 1000;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    return ms_of_day + ms.count();
}

std::string ImuSerial::generateGprmcTime(std::chrono::system_clock::time_point target_time) {
    auto time_t_target = std::chrono::system_clock::to_time_t(target_time);
    std::tm tm_utc;
    gmtime_r(&time_t_target, &tm_utc);

    std::ostringstream nmea;
    nmea << "$GPRMC," << std::setfill('0') << std::setw(2) << tm_utc.tm_hour << std::setw(2)
         << tm_utc.tm_min << std::setw(2) << tm_utc.tm_sec << ".00" // 整秒时刻,毫秒部分为00
         << ",A,3955.1234,N,11620.5678,E,0.0,0.0," << std::setw(2) << tm_utc.tm_mday << std::setw(2)
         << (tm_utc.tm_mon + 1) << std::setw(2) << (tm_utc.tm_year % 100) << ",,,A";

    uint8_t checksum = calculateNmeaChecksum(nmea.str());
    nmea << "*" << std::hex << std::uppercase << std::setw(2) << static_cast<int>(checksum)
         << "\r\n";

    return nmea.str();
}

void ImuSerial::syncLoop() {
    while (!imu_ready_ && sync_enabled_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    log(LogLevel::Info, "✓ IMU已就绪,开始时间同步...");

    while (sync_enabled_) {
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());

        // 计算下一个整秒对应的UTC时间
        auto next_second_ms = ((now_ms.count() / 1000) + 1) * 1000;
        auto target_time_t =
            std::chrono::system_clock::time_point(std::chrono::milliseconds(next_second_ms));

        // 预先生成GPRMC (使用目标时间)
        std::string gprmc = generateGprmcTime(target_time_t);

        // 高精度等待
        auto wait_duration = std::chrono::milliseconds(next_second_ms) - now_ms;
        std::this_thread::sleep_for(wait_duration);

        if (!sync_enabled_)
            break;

        // 产生上升沿
        setRtsPhysical(false);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        setRtsPhysical(true);

        // 立即发送预先生成的GPRMC
        boost::asio::write(serial_, boost::asio::buffer(gprmc));

        // ... 打印信息

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        setRtsPhysical(false);
    }
}

bool ImuSerial::validateAndExtract(const uint8_t* frame, uint16_t payload_len) {
    uint16_t crc = 0;
    calculateCrc16(&crc, frame, 4);
    calculateCrc16(&crc, frame + 6, payload_len);
    if (crc != readU16(frame + 4))
        return false;

    const uint8_t* payload = frame + 6;
    for (size_t i = 0; i + sizeof(HI91Data) <= payload_len; ++i) {
        if (payload[i] == 0x91) {
            HI91Data data;
            std::memcpy(&data, payload + i, sizeof(HI91Data));

            // 收到第一帧数据，说明IMU已就绪
            if (!imu_ready_) {
                imu_ready_ = true;
            }

            // 收到数据包，立即获取PC本地UTC时间

            // 检查UTC_TIME标志位（第11位，0=已同步）
            bool utc_time_synced = ((data.status & (1 << 11)) == 0);
            utc_synced_ = utc_time_synced;
            ++frames_since_trigger_;
            // 检查SOUT_PULSE标志位（第12位）
            if (data.status & (1 << 12)) {
                frames_since_trigger_ = 0;
            }
            frame_callback_(std::make_shared<ImuFrame>(data), frames_since_trigger_);
            return true;
        }
    }
    return false;
}

void ImuSerial::parseData(const uint8_t* data, size_t len) {
    if (parse_pos_ + len > parse_buffer_.size()) {
        std::memmove(
            parse_buffer_.data(),
            parse_buffer_.data() + (parse_pos_ > 2048 ? parse_pos_ - 2048 : 0),
            parse_pos_ > 2048 ? 2048 : parse_pos_
        );
        parse_pos_ = parse_pos_ > 2048 ? 2048 : parse_pos_;
    }

    std::memcpy(parse_buffer_.data() + parse_pos_, data, len);
    parse_pos_ += len;

    size_t i = 0;
    while (i + 6 <= parse_pos_) {
        if (parse_buffer_[i] == 0x5A && parse_buffer_[i + 1] == 0xA5) {
            uint16_t payload_len = readU16(parse_buffer_.data() + i + 2);
            size_t frame_size = 6 + payload_len;

            if (payload_len > 1024 || i + frame_size > parse_pos_) {
                if (payload_len > 1024)
                    i++;
                break;
            }

            validateAndExtract(parse_buffer_.data() + i, payload_len);
            i += frame_size;
        } else {
            i++;
        }
    }

    if (i < parse_pos_) {
        std::memmove(parse_buffer_.data(), parse_buffer_.data() + i, parse_pos_ - i);
        parse_pos_ -= i;
    } else {
        parse_pos_ = 0;
    }
}

ImuSerial::ImuSerial(
    std::function<void(const LogLevel, const std::string&)> log_callback,
    std::function<void(std::shared_ptr<ImuFrame>, int frames_since_trigger)> frame_callback
):
    log_callback_(log_callback),
    frame_callback_(frame_callback),
    serial_(io_service_),
    read_buffer_(1024 * 1024),
    parse_buffer_(2 * 1024 * 1024) {
    open("/dev/ttyUSB0");
}

bool ImuSerial::open(const std::string& port) {
    try {
        serial_.open(port);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(921600));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(
            boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none)
        );
        serial_.set_option(
            boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one)
        );
        serial_.set_option(
            boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none
            )
        );

        setRtsPhysical(false);

        log(LogLevel::Info, "✓ 串口已打开");

        // 发送REBOOT命令重启IMU
        log(LogLevel::Info, "→ 发送REBOOT指令...");
        std::string reboot_cmd = "REBOOT\r\n";
        boost::asio::write(serial_, boost::asio::buffer(reboot_cmd));

        // 启动IO线程
        io_thread_ = std::thread([this]() { io_service_.run(); });
        startRead();

        // 启动时间同步线程
        sync_thread_ = std::thread([this]() { syncLoop(); });

        return true;
    } catch (const std::exception& e) {
        log(LogLevel::Error, "打开串口失败: {}\n", e.what());
        return false;
    }
}

void ImuSerial::startRead() {
    serial_.async_read_some(
        boost::asio::buffer(read_buffer_),
        [this](const boost::system::error_code& error, size_t bytes) {
            if (!error && bytes > 0) {
                parseData(read_buffer_.data(), bytes);
                startRead();
            }
        }
    );
}
template<typename... Args>
void ImuSerial::log(const LogLevel log_level, const std::string& fmt, Args... args) {
    if (log_callback_)
        log_callback_(log_level, fmt::format(fmt::runtime(fmt), args...));
}

ImuSerial::~ImuSerial() {
    sync_enabled_ = false;
    if (sync_thread_.joinable())
        sync_thread_.join();

    io_service_.stop();
    if (io_thread_.joinable())
        io_thread_.join();

    if (serial_.is_open()) {
        setRtsPhysical(false);
    }
}
} // namespace helios_cv
