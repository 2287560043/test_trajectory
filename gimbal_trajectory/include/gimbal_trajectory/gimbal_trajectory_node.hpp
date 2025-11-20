#pragma once

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/parameter.hpp>
#include "gimbal_trajectory/gimbal_trajectory_node_parameters.hpp"

namespace helios_cv {

using Params = gimbal_trajectory_node::Params;
using ParamListener = gimbal_trajectory_node::ParamListener;

class GimbalTrajectoryNode : public rclcpp::Node
{
public:
    explicit GimbalTrajectoryNode(const rclcpp::NodeOptions & options);
    ~GimbalTrajectoryNode() override;
    
private:
    Params params_;
    std::shared_ptr<ParamListener> param_listener_;
};

} // namespace helios_cv