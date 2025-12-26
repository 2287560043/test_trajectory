#include <ftd2xx.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <vector>
#include <iomanip>
#include <sstream>
#include <pthread.h>
#include <unistd.h>
#include <camera_imu_bridge/ImuFrame.hpp>

FT_HANDLE ftHandle = nullptr;
std::atomic<bool> running{true};
std::atomic<bool> device_connected{false};
std::vector<uint8_t> parse_buffer(2 * 1024 * 1024);
size_t parse_pos = 0;
std::atomic<size_t> frame_count{0};
std::atomic<bool> imu_ready{false};

EVENT_HANDLE rx_event;
std::string target_serial_number;  // 保存序列号用于重连

void calculate_crc16(uint16_t* crc, const uint8_t* buf, uint32_t len) {
    uint32_t crc_val = *crc;
    for (uint32_t j = 0; j < len; ++j) {
        uint32_t byte = buf[j];
        crc_val ^= byte << 8;
        for (uint32_t i = 0; i < 8; ++i) {
            uint32_t temp = crc_val << 1;
            if (crc_val & 0x8000) temp ^= 0x1021;
            crc_val = temp;
        }
    }
    *crc = crc_val;
}

uint16_t read_u16(const uint8_t* p) {
    uint16_t u;
    std::memcpy(&u, p, 2);
    return u;
}

uint32_t get_utc_ms_of_day() {
    auto now = std::chrono::system_clock::now();
    auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ).count();
    return static_cast<uint32_t>(epoch_ms % 86400000);
}

std::string ms_to_utc_time(uint32_t total_ms) {
    uint32_t total_seconds = total_ms / 1000;
    uint32_t ms_part = total_ms % 1000;
    uint8_t hours = (total_seconds / 3600) % 24;
    uint8_t minutes = (total_seconds % 3600) / 60;
    uint8_t seconds = total_seconds % 60;
    
    char buf[32];
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d", hours, minutes, seconds, ms_part);
    return std::string(buf);
}

uint8_t calc_nmea_checksum(const std::string& sentence) {
    uint8_t checksum = 0;
    for (size_t i = 1; i < sentence.length(); ++i) {
        checksum ^= sentence[i];
    }
    return checksum;
}

std::string generate_gprmc_for_time(std::chrono::system_clock::time_point target_time) {
    auto time_t_target = std::chrono::system_clock::to_time_t(target_time);
    std::tm tm_utc;
    gmtime_r(&time_t_target, &tm_utc);
    
    std::ostringstream nmea;
    nmea << "$GPRMC,"
         << std::setfill('0') << std::setw(2) << tm_utc.tm_hour
         << std::setw(2) << tm_utc.tm_min
         << std::setw(2) << tm_utc.tm_sec
         << ".00"
         << ",A,3955.1234,N,11620.5678,E,0.0,0.0,"
         << std::setw(2) << tm_utc.tm_mday
         << std::setw(2) << (tm_utc.tm_mon + 1)
         << std::setw(2) << (tm_utc.tm_year % 100)
         << ",,,A";
    
    uint8_t checksum = calc_nmea_checksum(nmea.str());
    nmea << "*" << std::hex << std::uppercase << std::setw(2) 
         << static_cast<int>(checksum) << "\r\n";
    
    return nmea.str();
}

bool validate_and_extract(const uint8_t* frame, uint16_t payload_len) {
    uint16_t crc = 0;
    calculate_crc16(&crc, frame, 4);
    calculate_crc16(&crc, frame + 6, payload_len);
    if (crc != read_u16(frame + 4)) return false;

    const uint8_t* payload = frame + 6;
    for (size_t i = 0; i + sizeof(HI91Data) <= payload_len; ++i) {
        if (payload[i] == 0x91) {
            HI91Data data;
            std::memcpy(&data, payload + i, sizeof(HI91Data));
            
            if (!imu_ready) {
                imu_ready = true;
            }
            
            uint32_t pc_time_ms = get_utc_ms_of_day();
            bool utc_synced = ((data.status & (1 << 11)) == 0);
            bool sout_pulse = (data.status & (1 << 12));
            
            if (sout_pulse) {
                frame_count++;
                std::cout << "=== SOUT #" << frame_count.load() << " ===" << std::endl;
                std::cout << "Status: 0x" << std::hex << data.status << std::dec << std::endl;
                std::cout << "UTC同步: " << (utc_synced ? "是" : "否") << std::endl;
                
                if (utc_synced) {
                    int32_t delay = static_cast<int32_t>(pc_time_ms) 
                                  - static_cast<int32_t>(data.system_time);
                    
                    if (delay > 43200000) delay -= 86400000;
                    else if (delay < -43200000) delay += 86400000;
                    
                    std::cout << "IMU时间: " << ms_to_utc_time(data.system_time) << std::endl;
                    std::cout << "PC时间:  " << ms_to_utc_time(pc_time_ms) << std::endl;
                    std::cout << "延迟: " << delay << " ms" << std::endl;
                } else {
                    std::cout << "等待UTC同步..." << std::endl;
                }
                std::cout << std::endl;
            }
            return true;
        }
    }
    return false;
}

void parse_data(const uint8_t* data, size_t len) {
    if (parse_pos + len > parse_buffer.size()) {
        std::memmove(parse_buffer.data(), 
                     parse_buffer.data() + (parse_pos > 2048 ? parse_pos - 2048 : 0),
                     parse_pos > 2048 ? 2048 : parse_pos);
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
                if (payload_len > 1024) i++;
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


bool device_exists(const std::string& serial) {
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


void read_thread() {
    std::vector<uint8_t> buffer(64 * 1024);
    std::cout << "[RX] 线程启动" << std::endl;
    
    while (running && device_connected) {
        // 纯事件驱动，阻塞等待
        pthread_mutex_lock(&rx_event.eMutex);
        
        struct timespec timeout;
        clock_gettime(CLOCK_REALTIME, &timeout);
        timeout.tv_sec += 1;
        
        pthread_cond_timedwait(&rx_event.eCondVar, &rx_event.eMutex, &timeout);
        pthread_mutex_unlock(&rx_event.eMutex);
        
        if (!running || !device_connected) break;
        
        DWORD EventDWord, RxBytes, TxBytes;
        FT_STATUS status = FT_GetStatus(ftHandle, &RxBytes, &TxBytes, &EventDWord);
        
        if (status != FT_OK) {
            std::cerr << "[RX] 设备断开（FT_GetStatus失败）" << std::endl;
            device_connected = false;
            break;
        }
        
        if (EventDWord & FT_EVENT_RXCHAR && RxBytes > 0) {
            DWORD bytesRead = 0;
            DWORD toRead = std::min(RxBytes, (DWORD)buffer.size());
            status = FT_Read(ftHandle, buffer.data(), toRead, &bytesRead);
            
            // 唯一的掉线检测点：FT_Read返回IO_ERROR
            if (status == FT_IO_ERROR) {
                std::cerr << "[RX] USB断开（FT_IO_ERROR）" << std::endl;
                device_connected = false;
                break;
            }
            
            if (status == FT_OK && bytesRead > 0) {
                parse_data(buffer.data(), bytesRead);
            }
        }
    }
    
    std::cout << "[RX] 线程退出" << std::endl;
}


void pps_sync_thread() {
    std::cout << "[PPS] 线程启动" << std::endl;
    
    // 等待IMU就绪
    while (!imu_ready && running && device_connected) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    if (!running || !device_connected) return;
    
    std::cout << "[PPS] ✓ IMU就绪，开始同步" << std::endl;
    
    FT_SetRts(ftHandle);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    while (running && device_connected) {
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch());
        
        auto next_second_ms = ((now_ms.count() / 1000) + 1) * 1000;
        auto target_time = std::chrono::system_clock::time_point(
            std::chrono::milliseconds(next_second_ms));
        
        // 提前生成GPRMC（移出临界区）
        std::string gprmc = generate_gprmc_for_time(target_time);
        
        std::this_thread::sleep_until(target_time);
        
        if (!running || !device_connected) break;
        
        FT_SetRts(ftHandle);
        FT_ClrRts(ftHandle);
        
        DWORD written = 0;
        FT_STATUS status = FT_Write(ftHandle, (void*)gprmc.c_str(), gprmc.length(), &written);
        
        if (status != FT_OK) {
            std::cerr << "[PPS] 写入失败，设备断开" << std::endl;
            device_connected = false;
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        FT_SetRts(ftHandle);
    }
    
    std::cout << "[PPS] 线程退出" << std::endl;
}

bool init_device() {
    std::cout << "[初始化] 开始..." << std::endl;
    
    FT_STATUS status;
    
    if (target_serial_number.empty()) {
        // 首次打开
        status = FT_Open(0, &ftHandle);
        
        if (status == FT_OK) {
            FT_DEVICE device_type;
            DWORD device_id;
            char serial[16];
            char desc[64];
            
            if (FT_GetDeviceInfo(ftHandle, &device_type, &device_id, serial, desc, NULL) == FT_OK) {
                target_serial_number = serial;
                std::cout << "[初始化] 序列号: " << target_serial_number << std::endl;
            }
        }
    } else {
        // 重连时使用序列号
        status = FT_OpenEx((PVOID)target_serial_number.c_str(), 
                          FT_OPEN_BY_SERIAL_NUMBER, 
                          &ftHandle);
    }
    
    if (status != FT_OK) {
        std::cerr << "[初始化] 打开失败，错误: " << status << std::endl;
        return false;
    }
    
    // 配置设备
    FT_SetBaudRate(ftHandle, 921600);
    FT_SetDataCharacteristics(ftHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
    FT_SetFlowControl(ftHandle, FT_FLOW_NONE, 0, 0);
    FT_SetLatencyTimer(ftHandle, 1);  // 文档建议2ms，实际测试1ms
    FT_SetTimeouts(ftHandle, 1, 1);
    FT_SetUSBParameters(ftHandle, 4096, 0);
    
    UCHAR latency;
    FT_GetLatencyTimer(ftHandle, &latency);
    std::cout << "[init] 延迟定时器: " << (int)latency << " ms" << std::endl;
    
    FT_Purge(ftHandle, FT_PURGE_RX | FT_PURGE_TX);
    
    // 设置事件通知
    DWORD event_mask = FT_EVENT_RXCHAR | FT_EVENT_MODEM_STATUS;
    status = FT_SetEventNotification(ftHandle, event_mask, (PVOID)&rx_event);
    
    if (status != FT_OK) {
        std::cerr << "[init]  事件设置失败" << std::endl;
    }
    
    FT_ClrRts(ftHandle);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "[init] 完成" << std::endl;
    return true;
}


bool try_reconnect() {
    std::cout << "[重连] 等待设备..." << std::endl;
    
    int retry = 0;
    const int max_retry = 60;  
    
    while (running && retry < max_retry) {
        retry++;
        
        // 使用官方API扫描设备
        if (device_exists(target_serial_number)) {
            std::cout << "[重连] 检测到设备，尝试连接..." << std::endl;
            
            if (init_device()) {
                std::cout << "[重连] 重连成功！" << std::endl;
                return true;
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    
    std::cerr << "[重连] 超时放弃" << std::endl;
    return false;
}

int main() {

    pthread_mutex_init(&rx_event.eMutex, NULL);
    pthread_cond_init(&rx_event.eCondVar, NULL);
    

    if (!init_device()) {
        std::cerr << "初始化失败" << std::endl;
        return 1;
    }
    
    device_connected = true;
    

    std::thread read_th(read_thread);
    std::thread pps_th(pps_sync_thread);
    

    while (running) {
        if (!device_connected) {
            std::cout << "\n=== 设备断开，准备重连 ===" << std::endl;
            
            // 等待线程退出
            pthread_cond_signal(&rx_event.eCondVar);
            if (read_th.joinable()) read_th.join();
            if (pps_th.joinable()) pps_th.join();
            
            // 关闭设备
            if (ftHandle != nullptr) {
                FT_Close(ftHandle);
                ftHandle = nullptr;
            }
            
            // 重置状态
            imu_ready = false;
            parse_pos = 0;
            
            if (!try_reconnect()) {
                std::cerr << "重连失败，程序退出" << std::endl;
                break;
            }
            
            device_connected = true;
            read_th = std::thread(read_thread);
            pps_th = std::thread(pps_sync_thread);
            
            std::cout << "=== 恢复运行 ===\n" << std::endl;
        }
        
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        struct timeval tv = {0, 100000};  // 100ms超时
        
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) > 0) {
            std::cin.get();
            break;
        }
    }
    
    std::cout << "\n正在退出..." << std::endl;
    running = false;
    device_connected = false;
    
    pthread_cond_signal(&rx_event.eCondVar);
    
    if (read_th.joinable()) read_th.join();
    if (pps_th.joinable()) pps_th.join();
    
    if (ftHandle != nullptr) {
        FT_ClrRts(ftHandle);
        FT_Close(ftHandle);
    }
    
    pthread_mutex_destroy(&rx_event.eMutex);
    pthread_cond_destroy(&rx_event.eCondVar);
    
    std::cout << "程序退出" << std::endl;
    return 0;
}
