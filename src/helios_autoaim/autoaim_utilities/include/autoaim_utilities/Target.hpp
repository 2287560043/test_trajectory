#pragma once

#include <Eigen/Core>
#include <optional>
#include <chrono>
#include "../../include/autoaim_utilities/Armor.hpp"
#include "../../include/autoaim_utilities/ExtendedKalmanFilter.hpp"
#include <rclcpp/rclcpp.hpp>



namespace helios_cv
{

typedef enum
{
  LOST,
  TEMP_LOST,
  TRACKING,
  DETECTING
} TrakerState;

const std::vector<std::string> TRACKER_STATE_STR = {"LOST", "TEMP_LOST", "TRACKING", "DETECTING"};

enum AUTOAIM_MODE
{
  AUTOAIM = 0,
  SMALL_ENERGY = 1,
  BIG_ENERGY = 2,
};

// 强行兼容的权宜之计
enum TargetType
{
  OUTPOST,
  NORMAL
};


struct Target {
    // 核心状态向量
    Eigen::VectorXd state;  // 10维状态向量
    
    // 装甲板信息
    std::vector<Eigen::Vector3d> armor_positions;  // 4个装甲板的世界坐标位置
    std::vector<double> armor_yaws;                 // 4个装甲板的yaw角度
    std::vector<bool> armor_available;             // 装甲板是否可用
    
    // 运动状态
    double speed;                    // 目标速度
    bool is_top;                      
    
    // 跟踪信息
    std::string tracking_number;            // 跟踪的装甲板编号
    TargetType target_type;         // 装甲板类型
    ArmorType armor_type;
    int locked_armor_id;           // 锁定的装甲板ID (-1表示未锁定)
    
    // 时间信息
    rclcpp::Time last_update_time;  // 最后更新时间
    
    
    Target() : state(10) {
        state.setZero();
        armor_positions.resize(4);
        armor_yaws.resize(4);
        armor_available.resize(4, false);
        locked_armor_id = -1;
        tracking_number = -1;
    }
    
    // 从状态向量更新装甲板信息
    void update_armor_info() {
        double xc = state(0), yc = state(2);
        double z1 = state(4), z2 = state(5);
        double r1 = state(6), r2 = state(7);
        double yaw = state(8);
        
        for (int i = 0; i < 4; ++i) {
            double armor_yaw = yaw + i * M_PI_2;
            double r = (i % 2 == 0) ? r1 : r2;
            double z = (i % 2 == 0) ? z1 : z2;
            
            armor_positions[i] = Eigen::Vector3d(
                xc - r * cos(armor_yaw),
                yc - r * sin(armor_yaw),
                z
            );
            armor_yaws[i] = armor_yaw;
            armor_available[i] = true;
        }
        
        // 计算目标速度
        speed = sqrt(state(1) * state(1) + state(3) * state(3));
        is_top = (std::abs(state(9)) > 2.0) && (target_type != OUTPOST);
    }
};

}
