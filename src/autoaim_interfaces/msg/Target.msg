std_msgs/Header header
bool tracking
uint8 gimbal_id
string id
int32 armors_num
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 yaw
float64 v_yaw
float64 radius_1
float64 radius_2
float64 dz
string armor_type
# add
float64 fly_time
float64 bullet_speed # 这个是送入predictor节点时刻从下位机获取的弹速
PreTrajectory[] pretraj # 命中时刻前后一段时间的理想云台轨迹
float64 pitch

