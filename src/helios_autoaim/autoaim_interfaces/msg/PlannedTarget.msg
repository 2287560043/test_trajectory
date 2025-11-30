std_msgs/Header header

# 控制标志
bool control
bool fire

# 目标角度
float32 target_yaw
float32 target_pitch

# 规划结果
float32 yaw
float32 yaw_vel
float32 yaw_acc
float32 pitch
float32 pitch_vel
float32 pitch_acc

# 调试信息
geometry_msgs/Vector3 debug_position