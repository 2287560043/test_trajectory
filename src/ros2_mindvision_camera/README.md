# ros2_mindvision_camera


A ROS2 package for MindVision industrial camera

## Setting up a camera 

```shell
lsusb
sudo vim /etc/udev/rules.d/88-mvusb.rules
```

```c
KERNEL=="*", ATTRS{idVendor}=="f622", ATTRS{idProduct}=="d13a", MODE:="0777", SYMLINK+="mindvision134"
KERNEL=="*", ATTRS{idVendor}=="f622", ATTRS{idProduct}=="d132", MODE:="0777", SYMLINK+="mindvision133"
```
## Usage

### run as a node

> ros2 run mindvision_camera mindvision_camera_node

### run as a composable node

> ros2 launch  mindvision_camera mindvision_camera.launch.py

### using intra-process communication

```cpp
rclcpp::executors::MultiThreadedExecutor exec{};
auto intra_comms_options = rclcpp::NodeOptions{}.use_intra_process_comms(true);
auto camera_node = std::make_shared<RMCamera::MVCamera>(intra_comms_options);
auto sub_node = std::make_shared<...>(intra_comms_options);
exec.add_node(camera_node);
exec.add_node(sub_node);
exec.add_node(...);
exec.spin();
```

[read more](https://design.ros2.org/articles/intraprocess_communications.html)

## Topic Name

> /image_raw

## Acknowledgement
[chenjunnn/ros2_mindvision_camera](https://github.com/chenjunnn/ros2_mindvision_camera)
