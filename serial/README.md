# ROS2通用串口模块


写这个模块包的原因是直接简化了发送串口的大部分代码，包含了挂载处理下位机函数、挂载ROS2 Publisher、以及通过串口发送任何结构体的功能，
对串口进行了一次高度结合ROS2的封装。

## 通信协议

通信协议遵从以下文档中的串口通信协议：[通信协议](https://swjtuhelios.feishu.cn/wiki/UegHwlC0GiaUXtksSHXcFIZJnWh?from=from_copylink)

## 代码说明

```c++
typedef struct __packed
{
  uint8_t sof = 0xA5;
  uint16_t data_length;
  uint8_t seq;
  uint8_t crc8;
  uint16_t cmd_id;
} FrameHeader;
```

Header数据格式如上结构体所述

```c++
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
  void write(MessageT& msg, uint16_t cmd_id);

  template <typename MessageT>
  void publish(std::shared_ptr<MessageT> msg, uint16_t cmd_id);
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


```

Serial代码如上所示

基本原理如下：

1. 使用register publisher可以将ros2 里面的 publisher挂到publisher map中，再结合对应的cmd id，可以实现O(1)复杂度查询到对应的publisher
2. 使用register callback可以将一个function容器挂在到callback_map里面，依然结合对应的cmd id，可以自动识别到下位机发送的是什么信息，然后做对应的转换，最后传入到我们自定义的function中。需要注意的一点是传入到callback里面的不是结构体，而是数组，需要调用fromVector函数来转成对应的结构体。
3. 需要注意的一点是，如果在callback中调用了对应的publisher，需要先进行挂载，否则无法发送到指定的话题。
4. reopen_port函数可以在出现异常的时候自动重启串口，但是具体性能的影响之类的需要具体的验证。
5. 需要向下位机发送消息时，可以直接调用write函数，请传入自己的结构体和cmd id。


## 编写自定义代码示例

首先创建我们自己的功能包
```bash
ros2 pkg create xxx_pkg
```

然后再写好对应的package.xml和CMakeLists，记得引用serial包作为依赖（需要克隆到src文件夹，或者直接作为子模块（git submodule add 链接））

```bash
cd src
git submodule add git@e.coding.net:swjtuhelios/cv/serial.git
```

然后编写我们自己的ROS类

```c++
#include "..."
#include "serial/Serial.hpp"

class XXXBridge : public rclcpp::Node {
public:
  ......

private:


  rclcpp::Subscription<xxx::msg::XXXMsg>::SharedPtr xxx_sub_;
  rclcpp::Publisher<xxx::msg::XXXMsg>::SharedPtr xxx_pub_;
  std::shared_ptr<Serial> serial_;
}

```

然后编写对应的构造函数之类的创建Serial类，注意查看Serial的构造函数需要什么参数

```c++
  Serial(const std::string& serial_name, int baud_rate);
```

然后我们可以在构造函数中挂载我们需要的东西

```c++

// ...注意这里的CMDID是一个uint16，注意和协议匹配
serial_->register_publisher(XXX_CMD_ID,
                              std::make_shared<PublisherImpl<xxx::msg::XXXMsg>>(xxx_pub_));
// 注意这里的lamba函数的参数
serial_->register_callback(XXX_CMD_ID, [this](const std::vector<uint8_t>& data) {
  // 做转换
  auto mcu_info = fromVector<XXXPacket>(data);
  auto msg = std::make_shared<xxx::msg::XXXData>();
  msg->xxx = mcu_info.xxx;
  msg->xxx = mcu_info.xxx;
  //...
  msg->header.stamp = this->now() + rclcpp::Duration::from_seconds(params_.time_stamp_offset);
  // 直接调用serial的publish函数
  serial_->publish<xxx::msg::XXXData>(msg, XXX_CMD_ID);
});

// ...

```
我们还可以在处理消息的回调函数中向串口写东西

```c++
void XXXBridge::xxx_callback(xxx::msg::XXXData::SharedPtr msg)
{
  // ...注意这里使用的是我们自己的结构体而非ROS2的消息结构体
  auto xxx_data = std::make_shared<XXXData>();
  xxx_data.xxx = msg->xxx;
  // ...
  // Push into write fifo
  serial_->write(xxx_data, XXX_CMD_ID);
}


```

在写好这些函数过后，我们在构造函数的最后调用spin函数即可

```c++
  // ...
  serial_->spin();

```

spin函数的本质是创建了线程，我们也可以调用Stop函数来结束线程，再次启动时需要再次调用spin函数（未测试）


详细的代码可以参考自瞄代码中的ctrl_bridge。