#include <atomic>
#include <camera_imu_bridge/ImuFrame.hpp>
#include <camera_imu_bridge/ImuFtd2xx.hpp>
#include <camera_imu_bridge/LogLevel.hpp>
#include <chrono>
#include <cstring>
#include <fmt/format.h>
#include <ftd2xx.h>
#include <iomanip>
#include <pthread.h>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <vector>

namespace helios_cv {
ImuFtd2xx::ImuFtd2xx(
    std::function<void(const LogLevel, const std::string&)> log_callback,
    std::function<void(std::shared_ptr<ImuFrame>, int frames_since_trigger)> frame_callback
):
    log_callback_(log_callback),
    frame_callback_(frame_callback),
    parse_buffer_(2 * 1024 * 1024) {
    pthread_mutex_init(&rx_event_.eMutex, NULL);
    pthread_cond_init(&rx_event_.eCondVar, NULL);

    if (!initDevice()) {
        log(LogLevel::Error, "初始化失败");
        return;
    }

    device_connected_ = true;
    read_th = std::thread { [this]() { readLoop(); } };

    pps_thread_ = std::thread { [this]() { ppsSyncLoop(); } };

    watchdog_thread_ = std::thread { [this]() { watchdogLoop(); } };
}
ImuFtd2xx::~ImuFtd2xx() {
    log(LogLevel::Info, "正在退出...");
    running_ = false;
    device_connected_ = false;

    pthread_cond_signal(&rx_event_.eCondVar);

    if (watchdog_thread_.joinable()) {
        watchdog_thread_.join();
    }
    if (read_th.joinable()) {
        read_th.join();
    }
    if (pps_thread_.joinable()) {
        pps_thread_.join();
    }

    if (ft_handle_ != nullptr) {
        FT_ClrRts(ft_handle_);
        FT_Close(ft_handle_);
    }

    pthread_mutex_destroy(&rx_event_.eMutex);
    pthread_cond_destroy(&rx_event_.eCondVar);

    log(LogLevel::Info, "程序退出");
}

template<typename... Args>
void ImuFtd2xx::log(const LogLevel log_level, const std::string& fmt, Args... args) {
    if (log_callback_)
        log_callback_(log_level, fmt::format(fmt::runtime(fmt), args...));
}

void ImuFtd2xx::watchdogLoop() {
    while (running_) {
        if (!device_connected_) {
            log(LogLevel::Info, "=== 设备断开，准备重连 ===");

            // 等待线程退出
            pthread_cond_signal(&rx_event_.eCondVar);
            if (read_th.joinable())
                read_th.join();
            if (pps_thread_.joinable())
                pps_thread_.join();

            // 关闭设备
            if (ft_handle_ != nullptr) {
                FT_Close(ft_handle_);
                ft_handle_ = nullptr;
            }

            // 重置状态
            imu_ready_ = false;
            parse_pos_ = 0;

            if (!tryReconnect()) {
                log(LogLevel::Error, "重连失败，程序退出");
                break;
            }

            device_connected_ = true;
            read_th = std::thread { [this]() { readLoop(); } };

            pps_thread_ = std::thread { [this]() { ppsSyncLoop(); } };

            log(LogLevel::Info, "=== 恢复运行 ===");
        }

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
void ImuFtd2xx::calculateCrc16(uint16_t* crc, const uint8_t* buf, uint32_t len) {
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

uint16_t ImuFtd2xx::readU16(const uint8_t* p) {
    uint16_t u;
    std::memcpy(&u, p, 2);
    return u;
}

uint8_t ImuFtd2xx::calculateNmeaChecksum(const std::string& sentence) {
    uint8_t checksum = 0;
    for (size_t i = 1; i < sentence.length(); ++i) {
        checksum ^= sentence[i];
    }
    return checksum;
}

std::string ImuFtd2xx::generateGprmcTime(std::chrono::system_clock::time_point target_time) {
    auto time_t_target = std::chrono::system_clock::to_time_t(target_time);
    std::tm tm_utc;
    gmtime_r(&time_t_target, &tm_utc);

    std::ostringstream nmea;
    nmea << "$GPRMC," << std::setfill('0') << std::setw(2) << tm_utc.tm_hour << std::setw(2)
         << tm_utc.tm_min << std::setw(2) << tm_utc.tm_sec << ".00"
         << ",A,3955.1234,N,11620.5678,E,0.0,0.0," << std::setw(2) << tm_utc.tm_mday << std::setw(2)
         << (tm_utc.tm_mon + 1) << std::setw(2) << (tm_utc.tm_year % 100) << ",,,A";

    uint8_t checksum = calculateNmeaChecksum(nmea.str());
    nmea << "*" << std::hex << std::uppercase << std::setw(2) << static_cast<int>(checksum)
         << "\r\n";

    return nmea.str();
}

bool ImuFtd2xx::validateAndExtract(const uint8_t* frame, uint16_t payload_len) {
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

            if (!imu_ready_) {
                imu_ready_ = true;
            }

            bool sout_pulse = (data.status & (1 << 12));

            ++frames_since_trigger_;
            if (sout_pulse) {
                frame_count_++;
                frames_since_trigger_ = 0;
            }
            frame_callback_(std::make_shared<ImuFrame>(data), frames_since_trigger_);
            return true;
        }
    }
    return false;
}

void ImuFtd2xx::parseData(const uint8_t* data, size_t len) {
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

bool ImuFtd2xx::deviceExists(const std::string& serial) {
    FT_Rescan();

    DWORD numDevs = 0;
    FT_STATUS status = FT_CreateDeviceInfoList(&numDevs);

    if (status != FT_OK || numDevs == 0) {
        return false;
    }

    FT_DEVICE_LIST_INFO_NODE* devInfo = new FT_DEVICE_LIST_INFO_NODE[numDevs];
    status = FT_GetDeviceInfoList(devInfo, &numDevs);

    bool found = false;
    if (status == FT_OK) {
        for (DWORD i = 0; i < numDevs; i++) {
            if (serial == devInfo[i].SerialNumber) {
                found = true;
                break;
            }
        }
    }

    delete[] devInfo;
    return found;
}

void ImuFtd2xx::readLoop() {
    std::vector<uint8_t> buffer(64 * 1024);
    log(LogLevel::Info, "[RX] 线程启动");

    while (running_ && device_connected_) {
        // 纯事件驱动，阻塞等待
        pthread_mutex_lock(&rx_event_.eMutex);

        struct timespec timeout;
        clock_gettime(CLOCK_REALTIME, &timeout);
        timeout.tv_sec += 1;

        pthread_cond_timedwait(&rx_event_.eCondVar, &rx_event_.eMutex, &timeout);
        pthread_mutex_unlock(&rx_event_.eMutex);

        if (!running_ || !device_connected_)
            break;

        DWORD EventDWord, RxBytes, TxBytes;
        FT_STATUS status = FT_GetStatus(ft_handle_, &RxBytes, &TxBytes, &EventDWord);

        if (status != FT_OK) {
            log(LogLevel::Warn, "[RX] 设备断开（FT_GetStatus失败）");
            device_connected_ = false;
            break;
        }

        if (EventDWord & FT_EVENT_RXCHAR && RxBytes > 0) {
            DWORD bytesRead = 0;
            DWORD toRead = std::min(RxBytes, (DWORD)buffer.size());
            status = FT_Read(ft_handle_, buffer.data(), toRead, &bytesRead);

            // 唯一的掉线检测点：FT_Read返回IO_ERROR
            if (status == FT_IO_ERROR) {
                log(LogLevel::Warn, "[RX] USB断开（FT_IO_ERROR）");
                device_connected_ = false;
                break;
            }

            if (status == FT_OK && bytesRead > 0) {
                parseData(buffer.data(), bytesRead);
            }
        }
    }

    log(LogLevel::Info, "[RX] 线程退出");
}

void ImuFtd2xx::ppsSyncLoop() {
    log(LogLevel::Info, "[PPS] 线程启动");

    // 等待IMU就绪
    while (!imu_ready_ && running_ && device_connected_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!running_ || !device_connected_)
        return;

    log(LogLevel::Info, "[PPS] ✓ IMU就绪，开始同步");

    FT_SetRts(ft_handle_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    while (running_ && device_connected_) {
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());

        auto next_second_ms = ((now_ms.count() / 1000) + 1) * 1000;
        auto target_time =
            std::chrono::system_clock::time_point(std::chrono::milliseconds(next_second_ms));

        // 提前生成GPRMC（移出临界区）
        std::string gprmc = generateGprmcTime(target_time);

        std::this_thread::sleep_until(target_time);

        if (!running_ || !device_connected_)
            break;

        FT_SetRts(ft_handle_);
        FT_ClrRts(ft_handle_);

        DWORD written = 0;
        FT_STATUS status = FT_Write(ft_handle_, (void*)gprmc.c_str(), gprmc.length(), &written);

        if (status != FT_OK) {
            log(LogLevel::Warn, "[PPS] 写入失败，设备断开");
            device_connected_ = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        FT_SetRts(ft_handle_);
    }

    log(LogLevel::Info, "[PPS] 线程退出");
}

bool ImuFtd2xx::initDevice() {
    log(LogLevel::Info, "[初始化] 开始...");

    FT_STATUS status;

    if (target_serial_number_.empty()) {
        // 首次打开
        status = FT_Open(0, &ft_handle_);

        if (status == FT_OK) {
            FT_DEVICE device_type;
            DWORD device_id;
            char serial[16];
            char desc[64];

            if (FT_GetDeviceInfo(ft_handle_, &device_type, &device_id, serial, desc, NULL) == FT_OK)
            {
                target_serial_number_ = serial;
                log(LogLevel::Info, "[初始化] 序列号: {}", target_serial_number_.c_str());
            }
        }
    } else {
        // 重连时使用序列号
        status =
            FT_OpenEx((PVOID)target_serial_number_.c_str(), FT_OPEN_BY_SERIAL_NUMBER, &ft_handle_);
    }

    if (status != FT_OK) {
        log(LogLevel::Error, "[初始化] 打开失败，错误: {}", status);
        return false;
    }

    // 配置设备
    FT_SetBaudRate(ft_handle_, 921600);
    FT_SetDataCharacteristics(ft_handle_, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
    FT_SetFlowControl(ft_handle_, FT_FLOW_NONE, 0, 0);
    FT_SetLatencyTimer(ft_handle_, 1); // 文档建议2ms，实际测试1ms
    FT_SetTimeouts(ft_handle_, 1, 1);
    FT_SetUSBParameters(ft_handle_, 4096, 0);

    UCHAR latency;
    FT_GetLatencyTimer(ft_handle_, &latency);
    log(LogLevel::Info, "[init] 延迟定时器: %i ms", (int)latency);

    FT_Purge(ft_handle_, FT_PURGE_RX | FT_PURGE_TX);

    // 设置事件通知
    DWORD event_mask = FT_EVENT_RXCHAR | FT_EVENT_MODEM_STATUS;
    status = FT_SetEventNotification(ft_handle_, event_mask, (PVOID)&rx_event_);

    if (status != FT_OK) {
        log(LogLevel::Error, "[init]  事件设置失败");
    }

    FT_ClrRts(ft_handle_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    log(LogLevel::Info, "[init] 完成");
    return true;
}

bool ImuFtd2xx::tryReconnect() {
    log(LogLevel::Info, "[重连] 等待设备...");

    int retry = 0;
    const int max_retry = 60;

    while (running_ && retry < max_retry) {
        retry++;

        // 使用官方API扫描设备
        if (deviceExists(target_serial_number_)) {
            log(LogLevel::Info, "[重连] 检测到设备，尝试连接...");

            if (initDevice()) {
                log(LogLevel::Info, "[重连] 重连成功！");
                return true;
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    log(LogLevel::Error, "[重连] 超时放弃");
    return false;
}
} // namespace helios_cv
