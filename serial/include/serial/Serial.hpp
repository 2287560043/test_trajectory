// created by liuhan on 2024/3/28
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */
#pragma once

#include <cstdint>
#include <functional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <serial_driver/serial_driver.hpp>
#include <io_context/io_context.hpp>
#include <rclcpp/rclcpp.hpp>

#include "CRC.h"

#include <string>
#include <queue>
#include <memory>

#define __packed __attribute__((__packed__))

namespace helios_cv
{

typedef struct __packed
{
  uint8_t sof = 0xA5;
  uint16_t data_length;
  uint8_t seq;
  uint8_t crc8;
  uint16_t cmd_id;
} FrameHeader;

struct MsgBase
{
  virtual ~MsgBase() = default;
};

template <typename T>
struct MsgImpl : MsgBase
{
  MsgImpl<T>(T data) : data(data)
  {
  }
  T data;
};

struct PublisherBase
{
  virtual void publish(std::shared_ptr<MsgBase> msg) = 0;
  virtual ~PublisherBase() = default;
};

template <typename MsgType>
struct PublisherImpl : PublisherBase
{
  std::shared_ptr<rclcpp::Publisher<MsgType>> publisher;

  template <typename... Ts>
  PublisherImpl(Ts&&... ts) : publisher{ std::forward<Ts>(ts)... } {};

  void publish(std::shared_ptr<MsgBase> msg) override
  {
    auto msg_impl = std::dynamic_pointer_cast<MsgImpl<MsgType>>(msg);
    if (msg_impl == nullptr)
    {
      RCLCPP_ERROR(rclcpp::get_logger("PublisherImpl"), "Failed to cast message");
      return;
    }
    publisher->publish(msg_impl->data);
  }
};

class Serial
{
public:
  Serial() : owned_ctx_{ new IoContext(2) } {};

  Serial(const std::string& serial_name, int baud_rate);

  ~Serial();

  bool open(const std::string& serial_name, int baud_rate);

  void close();

  void spin();

  void stop();

  void register_callback(uint16_t cmd_id, std::function<void(std::vector<uint8_t>)> callback);

  void register_publisher(uint16_t cmd_id, std::shared_ptr<PublisherBase> publisher);

  template <typename MessageT>
  void write(MessageT& msg, uint16_t cmd_id)
  {
    RCLCPP_DEBUG_ONCE(logger_, "First Write Message Of Serial Begin");
    FrameHeader header;
    header.data_length = sizeof(msg);
    header.cmd_id = cmd_id;
    Append_CRC8_Check_Sum(reinterpret_cast<uint8_t*>(&header), sizeof(header));
    auto head_buffer = toVector(header);
    auto data_buffer = toVector(msg);
    auto new_vector = std::vector<uint8_t>(head_buffer.size() + data_buffer.size());
    std::copy(head_buffer.begin(), head_buffer.end(), new_vector.begin());
    std::copy(data_buffer.begin(), data_buffer.end(), new_vector.begin() + head_buffer.size());
    Append_CRC16_Check_Sum(new_vector.data(), sizeof(msg) + sizeof(header));
    write_fifo_.push(new_vector);
    RCLCPP_DEBUG_ONCE(logger_, "First Write Message Of Serial End");
  }

  template <typename MessageT>
  void publish(std::shared_ptr<MessageT> msg, uint16_t cmd_id)
  {
    RCLCPP_DEBUG_ONCE(logger_, "First Publish Of Serial Begin");
    auto it = publisher_map_.find(cmd_id);
    if (it != publisher_map_.end())
    {
      auto msg_impl = std::make_shared<MsgImpl<MessageT>>(*msg);
      it->second->publish(std::dynamic_pointer_cast<MsgBase>(msg_impl));
    }
    else
    {
      RCLCPP_ERROR(logger_, "No publisher found for cmd_id: %d", cmd_id);
    }
    RCLCPP_DEBUG_ONCE(logger_, "First Publish Of Serial End");
  }

private:
  void receive_loop(std::future<void> futureObj);

  void write_loop(std::future<void> futureObj);

  void reopen_port();

  // Serial
  std::unique_ptr<IoContext> owned_ctx_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::queue<std::vector<uint8_t>> write_fifo_;

  // template map
  std::unordered_map<uint16_t, std::shared_ptr<PublisherBase>> publisher_map_;
  std::unordered_map<uint16_t, std::function<void(std::vector<uint8_t>)>> callback_map_;

  // thread
  std::thread receive_thread_;
  std::thread write_thread_;

  std::promise<void> read_exit_signal_;
  std::promise<void> write_exit_signal_;

  // Logger
  rclcpp::Logger logger_ = rclcpp::get_logger("ctrl_bridge");
};

}  // namespace helios_cv