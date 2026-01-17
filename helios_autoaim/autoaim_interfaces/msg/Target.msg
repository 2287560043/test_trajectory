std_msgs/Header header
# 暂留
bool tracking
uint8 gimbal_id
string id
int32 armors_num
geometry_msgs/Point position
geometry_msgs/Point aim_position
geometry_msgs/Vector3 velocity
float64 yaw
float64 v_yaw
float64 radius_1
float64 radius_2
float64 dz
string armor_type
PreTrajectory[] pretraj     # 命中时刻前后一段时间的理想云台轨迹
float64 yaw0
int32 armor_id

