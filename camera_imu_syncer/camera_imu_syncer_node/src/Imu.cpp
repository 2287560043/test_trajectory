#include "Imu.hpp"
#include "Syncer.hpp"
#include <atomic>
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <cstring>
#include <thread>

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

static uint16_t read_u16(const uint8_t* p) {
    uint16_t u;
    std::memcpy(&u, p, 2);
    return u;
}

bool Imu::validate_and_extract(const uint8_t* frame, uint16_t payload_len) {
    // CRC校验
    uint16_t crc = 0;
    calculate_crc16(&crc, frame, 4);
    calculate_crc16(&crc, frame + 6, payload_len);
    if (crc != read_u16(frame + 4))
        return false;

    // 查找0x91
    const uint8_t* payload = frame + 6;
    for (size_t i = 0; i + sizeof(HI91Data) <= payload_len; ++i) {
        if (payload[i] == 0x91) {
            HI91Data data;
            std::memcpy(&data, payload + i, sizeof(HI91Data));

            // 检查 bit 12: SOUT_PULSE
            bool is_trigger = (data.status & (1 << 12)) != 0;

            ++frames_since_trigger;

            if (is_trigger) {
                frames_since_trigger.store(0);
                                // syncer->addIMUFrame(std::make_shared<Syncer::ImuInfo>(data));
            }
            if (frames_since_trigger == 1) {
                syncer->addIMUFrame(std::make_shared<Syncer::ImuInfo>(data));
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
    syncer(_syncer),
    serial(io_service),
    read_buffer(1024 * 1024),
    parse_buffer(2 * 1024 * 1024) {}

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

        io_thread = std::thread([this]() { io_service.run(); });
        start_read();
        return true;
    } catch (...) {
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
    io_service.stop();
    if (io_thread.joinable())
        io_thread.join();
}
