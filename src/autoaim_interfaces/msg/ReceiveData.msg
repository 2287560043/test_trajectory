std_msgs/Header header

uint8 target_color# 敌方颜色
uint8 autoaim_mode# 0自瞄 1 小符 2 大符
uint8 use_traditional# 0 use net, 1 use traditional  @陈俊伟 自己设置健位
float32 bullet_speed# 弹速
float32 yaw# 直接转发陀螺仪yaw即可
float32 pitch# 直接转发陀螺仪pitch即可
float32 roll # 直接转发陀螺仪的roll即可
float32 x    # final predicted target x
float32 y    # final predicted target y
float32 z    # final predicted target z