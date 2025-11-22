#include "autoaim_gimbal_planner/GimbalPlannerNode.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rmw/qos_profiles.h>
#include <tf2_ros/transform_listener.hpp>

namespace helios_cv {

GimbalPlannerNode::GimbalPlannerNode(const rclcpp::NodeOptions & options) 
: Node("gimbal_trajectory_node", options)
{
    param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();


    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
            cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
            cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
            cam_info_sub_.reset();
        });
    serial_sub_ = this->create_subscription<autoaim_interfaces::msg::ReceiveData>(
        "/receive_data", rclcpp::SensorDataQoS(), 
        [this] (autoaim_interfaces::msg::ReceiveData::SharedPtr mcu_packet) {
            if (last_autoaim_mode_ != mcu_packet->autoaim_mode) {
                rclcpp::Parameter param("autoaim_mode", mcu_packet->autoaim_mode);
                last_autoaim_mode_ = mcu_packet->autoaim_mode;
                this->set_parameter(param);
            }
        }
    );
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = 
        std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    target_sub_.subscribe(this, "/predictor/target", rmw_qos_profile_sensor_data);
    tf2_filter_ = std::make_shared<tf2_filter>(target_sub_, *tf2_buffer_, "odoom", 10,
                                             this->get_node_logging_interface(), this->get_node_clock_interface(),
                                             std::chrono::duration<int>(2));
    tf2_filter_->registerCallback(&GimbalPlannerNode::gimbal_planner_callback, this);
}

GimbalPlannerNode::~GimbalPlannerNode()
{
    RCLCPP_INFO(this->get_logger(), "GimbalPlannerNode destroyed");
}
void GimbalPlannerNode::gimbal_planner_callback(const autoaim_interfaces::msg::Target::SharedPtr target_msg) {
    
}

} // namespace helios_cv



RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::GimbalPlannerNode)