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

#include <stdint.h>

#include <cmath>
#include <cstdint>
#include <vector>

#define __packed __attribute__((__packed__))

namespace helios_cv
{

typedef enum CMD_ID
{
  SEND_TARGET_INFO_CMD_ID = 0x0002,
  RECEIVE_AUTOAIM_RECEIVE_CMD_ID = 0x0107
} CMD_ID;

typedef struct __packed
{
  uint8_t gimbal_id;  // 0为左云台，1为右云台
  uint8_t tracking;  // tracking  = 1或0是指当前上位机的predictor的整车观测器当前是否观测到了车辆
                     // 或者打符是否在观测中
  uint8_t id;          // outpost = 8  guard =  7  base = 9   energy = 10
  uint8_t armors_num;  // 2-balance 3-outpost 4-normal  5-energy  1-armor
  float_t x;           // 车辆中心x  (能量机关装甲板x)
  float_t y;           // 车辆中心y  (能量机关装甲板y)
  float_t z;           // 车辆中心z  (能量机关装甲板z)
  float_t yaw;         // 面对我们的装甲板的yaw值
  float_t vx;          // 类推
  float_t vy;
  float_t vz;
  float_t v_yaw;
  float_t r1;                  // 车辆第一个半径
  float_t r2;                  // 车辆第二个半径
  float_t dz;                  //两对装甲板之间的高低差值
  float_t vision_delay;        // 视觉处理消耗的全部时间
  uint16_t crc_check_sum = 0;  // 校验和
} TargetInfo;

typedef struct __packed
{
  uint8_t self_color;     // 自身颜色 0 蓝 1 红
  uint8_t autoaim_mode;     // 0自瞄 1 小符 2 大符
  uint8_t use_traditional;  // 0 use net, 1 use traditional  @陈俊伟 自己设置健位
  float bullet_speed;       // 弹速
  float yaw;                // 直接转发陀螺仪yaw即可
  float pitch;              // 直接转发陀螺仪pitch即可
  float roll;               // 直接转发陀螺仪的roll即可
  float x;                  // final predicted target x
  float y;                  // final predicted target y
  float z;                  // final predicted target z
  uint16_t crc_check_sum;   // crc16校验
} MCUPacket;

}  // namespace helios_cv