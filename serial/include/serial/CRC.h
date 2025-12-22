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
#include <vector>

// CRC8
void Append_CRC8_Check_Sum(uint8_t* pchMessage, uint16_t dwLength);
uint32_t Verify_CRC8_Check_Sum(uint8_t* pchMessage, uint16_t dwLength);
uint8_t Get_CRC8_Check_Sum(uint8_t* pchMessage, uint16_t dwLength, uint8_t ucCRC8);

// CRC16
void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);

template <typename PacketT>
inline PacketT fromVector(const std::vector<uint8_t>& data)
{
  PacketT packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t*>(&packet));
  return packet;
}

template <typename PacketT>
inline std::vector<uint8_t> toVector(const PacketT& data)
{
  std::vector<uint8_t> packet(sizeof(PacketT));
  std::copy(reinterpret_cast<const uint8_t*>(&data), reinterpret_cast<const uint8_t*>(&data) + sizeof(PacketT),
            packet.begin());
  return packet;
}
