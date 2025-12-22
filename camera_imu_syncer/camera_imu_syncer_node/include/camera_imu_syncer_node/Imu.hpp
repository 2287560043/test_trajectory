#pragma once
#include "Syncer.hpp"
#include <atomic>
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <cstdint>
class Syncer;
class Imu {
private:
  std::shared_ptr<Syncer> syncer;
  std::chrono::high_resolution_clock::time_point lastTime;
  std::atomic<size_t> frameCount{0};
  boost::asio::io_context io_service;
  boost::asio::serial_port serial;
  std::thread io_thread;

  std::vector<uint8_t> read_buffer;
  std::vector<uint8_t> parse_buffer;
  size_t parse_pos = 0;
  std::atomic<size_t> frame_count{0};

  // 用于统计 trigger 间隔
  std::atomic<bool> last_was_trigger{false};
  std::atomic<size_t> frames_since_trigger{0};
  std::atomic<size_t> trigger_count{0};
  static void calculate_crc16(uint16_t *crc, const uint8_t *buf, uint32_t len);
  bool validate_and_extract(const uint8_t *frame, uint16_t payload_len);
  void parse_data(const uint8_t *data, size_t len);

public:
  Imu(std::shared_ptr<Syncer> _syncer);
  bool open(const std::string &port);
  void start_read();
  ~Imu();
};
