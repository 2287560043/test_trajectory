#include "gimbal_trajectory/gimbal_trajectory_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace helios_cv {

GimbalTrajectoryNode::GimbalTrajectoryNode(const rclcpp::NodeOptions & options)
: Node("gimbal_trajectory_node", options)
{
    param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();
    RCLCPP_DEBUG(this->get_logger(), "debug: %d", params_.debug);
    
    
    RCLCPP_INFO(this->get_logger(), "GimbalTrajectoryNode initialized");
}

GimbalTrajectoryNode::~GimbalTrajectoryNode()
{
    RCLCPP_INFO(this->get_logger(), "GimbalTrajectoryNode destroyed");
}

} // namespace helios_cv

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::GimbalTrajectoryNode)