// Licensed under the MIT License.

// MindVision Camera SDK
#include "CameraStatus.h"
#include <CameraApi.h>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
// C++ system
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <atomic>
#include <array>

namespace RMCamera
{
class MVCamera : public rclcpp::Node
{
public:
  explicit MVCamera(const rclcpp::NodeOptions& options) : Node("mv_camera", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting MVCameraNode with Callback Mode!");

  // ========== 1. SDK初始化 ==========
  CameraSdkInit(1);
  std::array<char, 256> version;
  CameraSdkGetVersionString(version.data());
  RCLCPP_INFO(this->get_logger(), "Camera SDK Version: %s", version.data());
  
  // ========== 2. 枚举设备 ==========
  int i_camera_counts = 1;
  tSdkCameraDevInfo t_camera_enum_list;
  int i_status = CameraEnumerateDevice(&t_camera_enum_list, &i_camera_counts);
  
  if (i_camera_counts == 0) {
    RCLCPP_ERROR(this->get_logger(), "No camera found!");
    throw std::runtime_error("No camera found");
  }

  RCLCPP_INFO(this->get_logger(), "Found %d camera(s)", i_camera_counts);
  RCLCPP_INFO(this->get_logger(), "Camera: %s [%s] SN:%s", 
              t_camera_enum_list.acProductName, 
              t_camera_enum_list.acPortType, 
              t_camera_enum_list.acSn);

  // ========== 3. 相机初始化 ==========
  i_status = CameraInit(&t_camera_enum_list, -1, -1, &h_camera_);
  if (i_status != CAMERA_STATUS_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Camera init failed with status: %d", i_status);
    throw std::runtime_error("Camera initialization failed");
  }
  RCLCPP_INFO(this->get_logger(), "Camera initialized successfully");

  // 加载相机配置文件
  if (std::string(t_camera_enum_list.acProductName) == "MV-SUA230GC") {
    RCLCPP_INFO(this->get_logger(), "Detected MV-SUA230GC camera model");
    std::string pkg_path = ament_index_cpp::get_package_share_directory("mindvision_camera");
    std::string config_file = pkg_path + "/config/0409.config";

    CameraSdkStatus status = CameraReadParameterFromFile(h_camera_, const_cast<char*>(config_file.c_str()));
      
    if (status != CAMERA_STATUS_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), 
                  "Failed to load configuration file for MV-SUA230GC: %s", 
                  CameraGetErrorString(status));
      RCLCPP_WARN(this->get_logger(), "Continuing with default settings");
    } else {
      RCLCPP_INFO(this->get_logger(), "Configuration file loaded successfully");
    }
  }

  CameraGetCapability(h_camera_, &t_capability_);
  
  tSdkImageResolution resolution;
  CameraGetImageResolution(h_camera_, &resolution);
  image_width_ = resolution.iWidth;
  image_height_ = resolution.iHeight;
  RCLCPP_INFO(this->get_logger(), "Image resolution: %dx%d", image_width_, image_height_);

  // 关闭自动曝光
  CameraSetAeState(h_camera_, false); 
  
  // 设置触发模式
  CameraSdkStatus triggerStatus = CameraSetTriggerMode(h_camera_, 0);
  if (triggerStatus != CAMERA_STATUS_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set trigger mode: %d", triggerStatus);
    throw std::runtime_error("Failed to set trigger mode");
  }
  RCLCPP_INFO(this->get_logger(), "Camera configured for software triggering");

  // 声明参数
  declareParameters();

  // ========== ROS发布器配置（最优QoS） ==========
  //QoS配置：BEST_EFFORT + VOLATILE + KEEP_LAST(1)
  rclcpp::QoS qos(1);  // 只保留最新1帧
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);  // 允许丢帧，降低延迟
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);       // 不保存历史
  qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);            // 只保留最新

  if (options.use_intra_process_comms()) {
    // Intra-process 零拷贝模式
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
    RCLCPP_INFO(this->get_logger(), "Using intra-process communication (zero-copy)");
  } else {
    // 标准模式（会拷贝数据）
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos.get_rmw_qos_profile());
    RCLCPP_INFO(this->get_logger(), "Using standard communication (with copy)");
  }

  // Camera info manager
  camera_name_ = this->declare_parameter("camera_name", "mv_camera");
  camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, camera_name_);
  auto camera_info_url = this->declare_parameter(
      "camera_info_url", "package://mindvision_camera/config/camera_info.yaml");
  
  if (camera_info_manager_->validateURL(camera_info_url)) {
    camera_info_manager_->loadCameraInfo(camera_info_url);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
  }

  // FPS统计
  print_fps_ = this->declare_parameter("print_fps", true);
  fps_print_interval_ = this->declare_parameter("fps_print_interval", 2.0);
  frame_count_.store(0);
  last_fps_print_time_ = std::chrono::steady_clock::now();

  // 参数回调
  params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MVCamera::parametersCallback, this, std::placeholders::_1));

  // 设置帧回调
  CameraSdkStatus status = CameraSetCallbackFunction(
      h_camera_, frameCallback_static, this, nullptr);
  if (status != CAMERA_STATUS_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set frame callback: %d", status);
    throw std::runtime_error("Failed to set frame callback");
  }

  // 设置连接状态回调
  status = CameraSetConnectionStatusCallback(
      h_camera_, connectionStatusCallback_static, this);
  if (status != CAMERA_STATUS_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "Failed to set connection status callback: %d", status);
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
  CameraPlay(h_camera_);
  RCLCPP_INFO(this->get_logger(), "Camera started successfully!");
}

  ~MVCamera() override
  {
    if (h_camera_) {
      CameraSetCallbackFunction(h_camera_, nullptr, nullptr, nullptr);
      CameraSetConnectionStatusCallback(h_camera_, nullptr, nullptr);
      CameraUnInit(h_camera_);
    }
    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
  }

private:

  // 零拷贝回调函数
  static void frameCallback_static(CameraHandle hCamera, BYTE* pBuffer, 
                                 tSdkFrameHead* pHead, PVOID pContext)
  {
      auto* camera = static_cast<MVCamera*>(pContext);
      if (!camera) return;


      auto timestamp = camera->now();
      
      auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
      
      // 设置图像参数
      image_msg->header.stamp = timestamp;  
      image_msg->header.frame_id = "camera_optical_frame";
      image_msg->encoding = "bgr8";
      image_msg->height = camera->image_height_;
      image_msg->width = camera->image_width_;
      image_msg->step = camera->image_width_ * 3;
      image_msg->data.resize(image_msg->height * image_msg->step);

      // SDK图像处理
      CameraSdkStatus status = CameraImageProcessEx(
          hCamera, 
          pBuffer, 
          image_msg->data.data(), 
          pHead, 
          CAMERA_MEDIA_TYPE_BGR8, 
          0);
      
      if (status != CAMERA_STATUS_SUCCESS) {
        return;
      }
      
      // Camera info消息
      auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(camera->camera_info_msg_);
      info_msg->header.stamp = timestamp;  
      info_msg->header.frame_id = "camera_optical_frame";

      // 零拷贝发布
      if (camera->camera_pub_) {
        camera->camera_pub_.publish(*image_msg, *info_msg);
      } else if (camera->image_pub_ && camera->info_pub_) {
        camera->image_pub_->publish(std::move(image_msg));
        camera->info_pub_->publish(std::move(info_msg));
      }

      // FPS统计
      camera->frame_count_.fetch_add(1, std::memory_order_relaxed);
      
      if (camera->print_fps_) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - camera->last_fps_print_time_).count();

        if (elapsed >= camera->fps_print_interval_) {
          uint32_t count = camera->frame_count_.exchange(0, std::memory_order_relaxed);
          double fps = count / elapsed;
          RCLCPP_INFO(camera->get_logger(), "Camera FPS: %.2f", fps);
          camera->last_fps_print_time_ = now;
        }
      }
  }

  static void connectionStatusCallback_static(CameraHandle hCamera, UINT uMsg, UINT uParam, PVOID pContext)
  {
    auto* camera = static_cast<MVCamera*>(pContext);
    if (!camera) return;
    
    switch(uMsg) {
      case 0: RCLCPP_ERROR(camera->get_logger(), "Camera disconnected!"); break;
      case 1: RCLCPP_INFO(camera->get_logger(), "Camera reconnected!"); break;
      default: RCLCPP_WARN(camera->get_logger(), "Unknown connection status: %d", uMsg); break;
    }
  }

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    double exposure_line_time;
    CameraGetExposureLineTime(h_camera_, &exposure_line_time);
    param_desc.integer_range[0].from_value = t_capability_.sExposeDesc.uiExposeTimeMin * exposure_line_time;
    param_desc.integer_range[0].to_value = t_capability_.sExposeDesc.uiExposeTimeMax * exposure_line_time;
    double exposure_time = this->declare_parameter("exposure_time", 2000, param_desc);
    CameraSetExposureTime(h_camera_, exposure_time);
    CameraSetFrameSpeed(h_camera_, 2);
    RCLCPP_INFO(this->get_logger(), "Exposure time = %f", exposure_time);

    // Analog gain
    param_desc.description = "Analog gain";
    param_desc.integer_range[0].from_value = t_capability_.sExposeDesc.uiAnalogGainMin;
    param_desc.integer_range[0].to_value = t_capability_.sExposeDesc.uiAnalogGainMax;
    int analog_gain;
    CameraGetAnalogGain(h_camera_, &analog_gain);
    analog_gain = this->declare_parameter("analog_gain", analog_gain, param_desc);
    CameraSetAnalogGain(h_camera_, analog_gain);
    RCLCPP_INFO(this->get_logger(), "Analog gain = %d", analog_gain);

    // RGB Gain
    CameraGetGain(h_camera_, &r_gain_, &g_gain_, &b_gain_);
    param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iRGainMin;
    param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iRGainMax;
    r_gain_ = this->declare_parameter("rgb_gain.r", r_gain_, param_desc);
    param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iGGainMin;
    param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iGGainMax;
    g_gain_ = this->declare_parameter("rgb_gain.g", g_gain_, param_desc);
    param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iBGainMin;
    param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iBGainMax;
    b_gain_ = this->declare_parameter("rgb_gain.b", b_gain_, param_desc);
    CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
    RCLCPP_INFO(this->get_logger(), "RGB Gain: R=%d, G=%d, B=%d", r_gain_, g_gain_, b_gain_);

    // Saturation
    param_desc.description = "Saturation";
    param_desc.integer_range[0].from_value = t_capability_.sSaturationRange.iMin;
    param_desc.integer_range[0].to_value = t_capability_.sSaturationRange.iMax;
    int saturation;
    CameraGetSaturation(h_camera_, &saturation);
    saturation = this->declare_parameter("saturation", saturation, param_desc);
    CameraSetSaturation(h_camera_, saturation);
    RCLCPP_INFO(this->get_logger(), "Saturation = %d", saturation);

    // Gamma
    param_desc.integer_range[0].from_value = t_capability_.sGammaRange.iMin;
    param_desc.integer_range[0].to_value = t_capability_.sGammaRange.iMax;
    int gamma;
    CameraGetGamma(h_camera_, &gamma);
    gamma = this->declare_parameter("gamma", gamma, param_desc);
    CameraSetGamma(h_camera_, gamma);
    RCLCPP_INFO(this->get_logger(), "Gamma = %d", gamma);
    
    this->declare_parameter("recorder_save_root", "/home/jk/Videos");
    
    // 镜像参数（使用SDK硬件镜像）
    mirror_angle_ = this->declare_parameter("mirror_angle", 0);
    if(mirror_angle_ == 1) {
      CameraSdkStatus status = CameraSetHardwareMirror(h_camera_, 0, true);
      if (status != CAMERA_STATUS_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set hardware mirror: %s", CameraGetErrorString(status));
      }
      status = CameraSetHardwareMirror(h_camera_, 1, true);
      if (status != CAMERA_STATUS_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set hardware mirror: %s", CameraGetErrorString(status));
      }
    } 

    RCLCPP_INFO(this->get_logger(), "Image rotation set to %d degrees", mirror_angle_ * 90);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    std::vector<std::string> error_messages;
    
    for (const auto& param : parameters) {
      const auto& name = param.get_name();
      int status = CAMERA_STATUS_SUCCESS;
      
      if (name == "exposure_time") {
        status = CameraSetExposureTime(h_camera_, param.as_int());
      } else if (name == "analog_gain") {
        status = CameraSetAnalogGain(h_camera_, param.as_int());
      } else if (name == "rgb_gain.r") {
        r_gain_ = param.as_int();
        status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
      } else if (name == "rgb_gain.g") {
        g_gain_ = param.as_int();
        status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
      } else if (name == "rgb_gain.b") {
        b_gain_ = param.as_int();
        status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
      } else if (name == "saturation") {
        status = CameraSetSaturation(h_camera_, param.as_int());
      } else if (name == "gamma") {
        status = CameraSetGamma(h_camera_, param.as_int());
      } else if (name == "mirror_angle") {
        mirror_angle_ = param.as_int();
      } else if (name == "print_fps") {
        print_fps_ = param.as_bool();
      } else if (name == "fps_print_interval") {
        fps_print_interval_ = param.as_double();
      } else {
        error_messages.push_back("Unknown parameter: " + name);
        result.successful = false;
        continue;
      }
      
      if (status != CAMERA_STATUS_SUCCESS) {
        error_messages.push_back("Failed to set " + name + ", status = " + std::to_string(status));
        result.successful = false;
      }
    }
    
    // 合并所有错误信息
    if (!error_messages.empty()) {
      result.reason = "";
      for (size_t i = 0; i < error_messages.size(); ++i) {
        result.reason += error_messages[i];
        if (i < error_messages.size() - 1) {
          result.reason += "; ";
        }
      }
    }
    
    return result;
  }

  // 相机句柄和参数
  int h_camera_;
  tSdkCameraCapbility t_capability_;
  int image_width_;
  int image_height_;

  // 发布器
  image_transport::CameraPublisher camera_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

  // 相机信息
  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  // 相机参数
  int r_gain_, g_gain_, b_gain_;
  int mirror_angle_; 

  // FPS统计
  bool print_fps_;
  double fps_print_interval_;
  std::atomic<uint32_t> frame_count_{0};
  std::chrono::steady_clock::time_point last_fps_print_time_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

}  // namespace RMCamera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(RMCamera::MVCamera)