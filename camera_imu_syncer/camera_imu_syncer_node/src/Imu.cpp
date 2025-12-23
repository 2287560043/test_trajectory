#include "Imu.hpp"
#include "Syncer.hpp"
#include <atomic>
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <cstring>
#include <thread>

// boost::asio::io_context io_service;
// boost::asio::serial_port serial;
// std::thread io_thread;
// std::thread sync_thread;
//
// std::vector<uint8_t> read_buffer;
// std::vector<uint8_t> parse_buffer;
// size_t parse_pos = 0;
// std::atomic<size_t> frame_count{0};
// std::atomic<size_t> sync_count{0};
// std::atomic<bool> sync_enabled{true};
// std::atomic<bool> utc_synced{false};
// std::atomic<bool> imu_ready{false};  // IMU就绪标志

void Imu::calculate_crc16(uint16_t* crc, const uint8_t* buf, uint32_t len) {
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

uint16_t Imu::read_u16(const uint8_t* p) {
    uint16_t u;
    std::memcpy(&u, p, 2);
    return u;
}

void Imu::set_rts_physical(bool physical_high) {
    int fd = serial.native_handle();
    int status;
    ioctl(fd, TIOCMGET, &status);

    if (physical_high) {
        status &= ~TIOCM_RTS;
    } else {
        status |= TIOCM_RTS;
    }
    ioctl(fd, TIOCMSET, &status);
}

uint8_t Imu::calc_nmea_checksum(const std::string& sentence) {
    uint8_t checksum = 0;
    for (size_t i = 1; i < sentence.length(); ++i) {
        checksum ^= sentence[i];
    }
    return checksum;
}

std::string Imu::generate_gprmc() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::tm tm_utc;
    gmtime_r(&time_t_now, &tm_utc);

    std::ostringstream nmea;
    nmea << "$GPRMC," << std::setfill('0') << std::setw(2) << tm_utc.tm_hour << std::setw(2)
         << tm_utc.tm_min << std::setw(2) << tm_utc.tm_sec << "." << std::setw(2)
         << (ms.count() / 10) << ",A,3955.1234,N,11620.5678,E,0.0,0.0," << std::setw(2)
         << tm_utc.tm_mday << std::setw(2) << (tm_utc.tm_mon + 1) << std::setw(2)
         << (tm_utc.tm_year % 100) << ",,,A";

    uint8_t checksum = calc_nmea_checksum(nmea.str());
    nmea << "*" << std::hex << std::uppercase << std::setw(2) << static_cast<int>(checksum)
         << "\r\n";

    return nmea.str();
}

uint32_t Imu::get_utc_ms_of_day() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_utc;
    gmtime_r(&time_t_now, &tm_utc);

    uint32_t ms_of_day = (tm_utc.tm_hour * 3600 + tm_utc.tm_min * 60 + tm_utc.tm_sec) * 1000;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    return ms_of_day + ms.count();
}

std::string Imu::ms_to_utc_time(uint32_t total_ms) {
    uint32_t total_seconds = total_ms / 1000;
    uint32_t ms_part = total_ms % 1000;

    uint8_t hours = (total_seconds / 3600) % 24;
    uint8_t minutes = (total_seconds % 3600) / 60;
    uint8_t seconds = total_seconds % 60;

    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << (int)hours << ":" << std::setw(2) << (int)minutes
        << ":" << std::setw(2) << (int)seconds << "." << std::setw(3) << ms_part;
    return oss.str();
}

std::string Imu::generate_gprmc_for_time(std::chrono::system_clock::time_point target_time) {
    auto time_t_target = std::chrono::system_clock::to_time_t(target_time);
    // auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(target_time.time_since_epoch())
    //     % 1000;
    //
    std::tm tm_utc;
    gmtime_r(&time_t_target, &tm_utc);

    std::ostringstream nmea;
    nmea << "$GPRMC," << std::setfill('0') << std::setw(2) << tm_utc.tm_hour << std::setw(2)
         << tm_utc.tm_min << std::setw(2) << tm_utc.tm_sec << ".00" // 整秒时刻,毫秒部分为00
         << ",A,3955.1234,N,11620.5678,E,0.0,0.0," << std::setw(2) << tm_utc.tm_mday << std::setw(2)
         << (tm_utc.tm_mon + 1) << std::setw(2) << (tm_utc.tm_year % 100) << ",,,A";

    uint8_t checksum = calc_nmea_checksum(nmea.str());
    nmea << "*" << std::hex << std::uppercase << std::setw(2) << static_cast<int>(checksum)
         << "\r\n";

    return nmea.str();
}

void Imu::sync_loop() {
    while (!imu_ready && sync_enabled) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "✓ IMU已就绪,开始时间同步...\n" << std::endl;

    while (sync_enabled) {
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());

        // 计算下一个整秒对应的UTC时间
        auto next_second_ms = ((now_ms.count() / 1000) + 1) * 1000;
        auto target_time_t =
            std::chrono::system_clock::time_point(std::chrono::milliseconds(next_second_ms));

        // 预先生成GPRMC (使用目标时间)
        std::string gprmc = generate_gprmc_for_time(target_time_t);

        // 高精度等待
        auto wait_duration = std::chrono::milliseconds(next_second_ms) - now_ms;
        std::this_thread::sleep_for(wait_duration);

        if (!sync_enabled)
            break;

        // 产生上升沿
        set_rts_physical(false);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        set_rts_physical(true);

        // 立即发送预先生成的GPRMC
        boost::asio::write(serial, boost::asio::buffer(gprmc));

        sync_count++;
        // ... 打印信息

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        set_rts_physical(false);
    }
}

bool Imu::validate_and_extract(const uint8_t* frame, uint16_t payload_len) {
    uint16_t crc = 0;
    calculate_crc16(&crc, frame, 4);
    calculate_crc16(&crc, frame + 6, payload_len);
    if (crc != read_u16(frame + 4))
        return false;

    const uint8_t* payload = frame + 6;
    for (size_t i = 0; i + sizeof(HI91Data) <= payload_len; ++i) {
        if (payload[i] == 0x91) {
            HI91Data data;
            std::memcpy(&data, payload + i, sizeof(HI91Data));

            // 收到第一帧数据，说明IMU已就绪
            if (!imu_ready) {
                imu_ready = true;
            }

            // 收到数据包，立即获取PC本地UTC时间
            uint32_t pc_recv_time_ms = get_utc_ms_of_day();

            // 检查UTC_TIME标志位（第11位，0=已同步）
            bool utc_time_synced = ((data.status & (1 << 11)) == 0);
            utc_synced = utc_time_synced;

            // 检查SOUT_PULSE标志位（第12位）
            if (data.status & (1 << 12)) {
                frame_count++;

                std::cout << "  ├─ SOUT #" << std::setw(4) << frame_count.load() - 1
                          << " | Status: 0x" << std::hex << data.status << std::dec
                          << " | Sync: " << (utc_time_synced ? "✓" : "✗");

                if (utc_time_synced) {
                    // 计算同步误差：IMU时间 - PC时间
                    int32_t sync_error = static_cast<int32_t>(data.system_time)
                        - static_cast<int32_t>(pc_recv_time_ms);

                    // 处理跨天的情况
                    if (sync_error > 43200000) {
                        sync_error -= 86400000;
                    } else if (sync_error < -43200000) {
                        sync_error += 86400000;
                    }

                    std::cout << "\n  ├─ IMU Time:  " << ms_to_utc_time(data.system_time)
                              << "\n  ├─ PC Time:   " << ms_to_utc_time(pc_recv_time_ms)
                              << "\n  └─ Error: " << std::showpos << sync_error << std::noshowpos
                              << " ms";

                } else {
                    std::cout << "\n  └─ Local Time: " << data.system_time << " ms (等待同步...)";
                }
                syncer->addIMUFrame(std::make_shared<Syncer::ImuInfo>(data));

                std::cout << std::endl;
            }
            return true;
        }
    }
    return false;
}

void Imu::parse_data(const uint8_t* data, size_t len) {
    if (parse_pos + len > parse_buffer.size()) {
        std::memmove(
            parse_buffer.data(),
            parse_buffer.data() + (parse_pos > 2048 ? parse_pos - 2048 : 0),
            parse_pos > 2048 ? 2048 : parse_pos
        );
        parse_pos = parse_pos > 2048 ? 2048 : parse_pos;
    }

    std::memcpy(parse_buffer.data() + parse_pos, data, len);
    parse_pos += len;

    size_t i = 0;
    while (i + 6 <= parse_pos) {
        if (parse_buffer[i] == 0x5A && parse_buffer[i + 1] == 0xA5) {
            uint16_t payload_len = read_u16(parse_buffer.data() + i + 2);
            size_t frame_size = 6 + payload_len;

            if (payload_len > 1024 || i + frame_size > parse_pos) {
                if (payload_len > 1024)
                    i++;
                break;
            }

            validate_and_extract(parse_buffer.data() + i, payload_len);
            i += frame_size;
        } else {
            i++;
        }
    }

    if (i < parse_pos) {
        std::memmove(parse_buffer.data(), parse_buffer.data() + i, parse_pos - i);
        parse_pos -= i;
    } else {
        parse_pos = 0;
    }
}

Imu::Imu(std::shared_ptr<Syncer> _syncer):
    serial(io_service),
    read_buffer(1024 * 1024),
    parse_buffer(2 * 1024 * 1024),
    syncer(_syncer) {}

bool Imu::open(const std::string& port) {
    try {
        serial.open(port);
        serial.set_option(boost::asio::serial_port_base::baud_rate(921600));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(
            boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none)
        );
        serial.set_option(
            boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one)
        );
        serial.set_option(
            boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none
            )
        );

        set_rts_physical(false);

        std::cout << "✓ 串口已打开" << std::endl;

        // 发送REBOOT命令重启IMU
        std::cout << "→ 发送REBOOT指令..." << std::endl;
        std::string reboot_cmd = "REBOOT\r\n";
        boost::asio::write(serial, boost::asio::buffer(reboot_cmd));

        // 启动IO线程
        io_thread = std::thread([this]() { io_service.run(); });
        start_read();

        // 启动时间同步线程
        sync_thread = std::thread([this]() { sync_loop(); });

        return true;
    } catch (const std::exception& e) {
        std::cerr << "打开串口失败: " << e.what() << std::endl;
        return false;
    }
}

void Imu::start_read() {
    serial.async_read_some(
        boost::asio::buffer(read_buffer),
        [this](const boost::system::error_code& error, size_t bytes) {
            if (!error && bytes > 0) {
                parse_data(read_buffer.data(), bytes);
                start_read();
            }
        }
    );
}

Imu::~Imu() {
    sync_enabled = false;
    if (sync_thread.joinable())
        sync_thread.join();

    io_service.stop();
    if (io_thread.joinable())
        io_thread.join();

    if (serial.is_open()) {
        set_rts_physical(false);
    }
}
