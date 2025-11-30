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
#include "Serial.hpp"
#include "CRC.h"
#include <chrono>
#include <codecvt>
#include <cstdint>
#include <future>
#include <memory>
#include <rclcpp/logging.hpp>
#include <stdexcept>

namespace helios_cv
{

Serial::Serial(const std::string& serial_name, int baud_rate) : owned_ctx_{ new IoContext(2) }
{
  serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*owned_ctx_);
  // create serial
  device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
      baud_rate, drivers::serial_driver::FlowControl::NONE, drivers::serial_driver::Parity::NONE,
      drivers::serial_driver::StopBits::ONE);
  serial_driver_->init_port(serial_name, *device_config_);
  if (!serial_driver_->port()->is_open())
  {
    serial_driver_->port()->open();
  }
}

Serial::~Serial()
{
  serial_driver_->port()->close();
}

bool Serial::open(const std::string& serial_name, int baud_rate)
{
  serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*owned_ctx_);
  // create serial
  device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
      baud_rate, drivers::serial_driver::FlowControl::NONE, drivers::serial_driver::Parity::NONE,
      drivers::serial_driver::StopBits::ONE);
  serial_driver_->init_port(serial_name, *device_config_);
  if (!serial_driver_->port()->is_open())
  {
    serial_driver_->port()->open();
  }
  if (!serial_driver_->port()->is_open())
  {
    return false;
  }
  return true;
}

void Serial::close()
{
  serial_driver_->port()->close();
}

void Serial::spin()
{
  std::future<void> read_future = read_exit_signal_.get_future();
  std::future<void> write_future = write_exit_signal_.get_future();
  receive_thread_ = std::thread(&Serial::receive_loop, this, std::move(read_future));
  write_thread_ = std::thread(&Serial::write_loop, this, std::move(write_future));
}

void Serial::stop()
{
  read_exit_signal_.set_value();
  write_exit_signal_.set_value();
}

void Serial::register_callback(uint16_t cmd_id, std::function<void(std::vector<uint8_t>)> callback)
{
  callback_map_.insert({ cmd_id, callback });
}

void Serial::register_publisher(uint16_t cmd_id, std::shared_ptr<PublisherBase> publisher)
{
  publisher_map_.insert({ cmd_id, publisher });
}

void Serial::reopen_port()
{
  RCLCPP_WARN(logger_, "Attempting to reopen port");
  RCLCPP_DEBUG_ONCE(logger_, "First Reopen Port Begin");
  while (true)
  {
    try
    {
      if (serial_driver_->port()->is_open())
      {
        serial_driver_->port()->close();
      }
      serial_driver_->port()->open();
      RCLCPP_INFO(logger_, "Successfully reopened port");
      break;
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(logger_, "Error while reopening port: %s, sleep and retrying", ex.what());
      if (rclcpp::ok())
      {
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  RCLCPP_DEBUG_ONCE(logger_, "First Reopen Port End");
}

void Serial::receive_loop(std::future<void> futureObj)
{
  RCLCPP_INFO(logger_, "Starting Reading...");
  std::vector<uint8_t> SOF(1);
  std::vector<uint8_t> data;
  std::vector<uint8_t> header(7, 0xa5);
  data.reserve(256);
  while (rclcpp::ok() && futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
  {
    try
    {
      RCLCPP_DEBUG_ONCE(logger_, "First Read Message Of Serial Begin");
      serial_driver_->port()->receive(SOF);
      if (SOF[0] == 0xA5)
      {
        std::vector<uint8_t> temp_header(6);
        serial_driver_->port()->receive(temp_header);
        FrameHeader frame_header;
        std::copy(temp_header.begin(), temp_header.end(), header.begin() + 1);
        std::copy(header.begin(), header.end(), reinterpret_cast<uint8_t*>(&frame_header));
        if (frame_header.data_length > 256)
        {
          continue;
        }
        if (Verify_CRC8_Check_Sum(reinterpret_cast<uint8_t*>(&frame_header), sizeof(frame_header) - 2))
        {
          /// TODO: 这个位置记得检查一下temp data的长度应该为多少，然后根据这个修改后面的+2是否应该为其他数字，现在+2
          // 主要是因为之前电控的data_length好像没算上crc16的长度，所以+2了，记得检查是不是这么多
          RCLCPP_DEBUG_ONCE(logger_, "First Pass CRC8 Check");
          std::vector<uint8_t> temp_data(frame_header.data_length + 2);
          serial_driver_->port()->receive(temp_data);
          std::vector<uint8_t> data(temp_data.size() + sizeof(frame_header));

          std::copy(header.begin(), header.end(), data.begin());
          std::copy(temp_data.begin(), temp_data.end(), data.begin() + sizeof(frame_header));

          if (Verify_CRC16_Check_Sum(data.data(), data.size()))
          {
            RCLCPP_DEBUG_ONCE(logger_, "First Pass CRC16 Check");
            const auto& callback = callback_map_.find(frame_header.cmd_id);
            if (callback != callback_map_.end())
            {
              callback->second(temp_data);
            }
            else
            {
              RCLCPP_WARN(logger_, "Unknown cmd id : %x", frame_header.cmd_id);
            }
          }
          else
          {
            RCLCPP_WARN(logger_, "Invalid data CRC16 checksum");
          }
        }
        else
        {
          RCLCPP_WARN(logger_, "Invalid data CRC8 checksum");
        }
      }
      RCLCPP_DEBUG_ONCE(logger_, "First Read Message Of Serial End");
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(logger_, "Error while receiving data: %s", ex.what());
      reopen_port();
    }
  }
}

void Serial::write_loop(std::future<void> futureObj)
{
  RCLCPP_WARN(logger_, "Starting writting...");
  while (rclcpp::ok() && futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
  {
    if (write_fifo_.empty())
    {
      continue;
    }
    RCLCPP_DEBUG_ONCE(logger_, "First Write Message Of Serial Begin");
    try
    {
      auto buffer = write_fifo_.front();
      write_fifo_.pop();
      serial_driver_->port()->send(buffer);
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(logger_, "Error while sending data: %s", ex.what());
      reopen_port();
    }
    RCLCPP_DEBUG_ONCE(logger_, "First Write Message Of Serial End");
  }
}

}  // namespace helios_cv