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

    yaw_offset_ = params_.yaw_offset / 57.3;
    pitch_offset_ = params_.pitch_offset / 57.3;
    fire_thresh_ = params_.fire_thresh;
    decision_speed_ = params_.decision_speed;
    high_speed_delay_time_ = params_.high_speed_delay_time;
    low_speed_delay_time_ = params_.low_speed_delay_time;




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
    RCLCPP_INFO(logger_, "GimbalPlannerNode destroyed");
}
void GimbalPlannerNode::gimbal_planner_callback(const autoaim_interfaces::msg::Target::SharedPtr target_msg) {
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
        RCLCPP_WARN(logger_, "params updated");
    }

    rclcpp::Time time = target_msg->header.stamp;
    double dt = time.seconds() - time_planner_start_;
    time_planner_start_ = time.seconds();

    if (params_.autoaim_mode != 0 || dt < 0) {
        return;
    }
    init_yaw_planner();
    init_pitch_planner();


}

void GimbalPlannerNode::init_yaw_planner() {
    auto max_yaw_acc = params_.max_yaw_acc;
    std::vector<double> Q_yaw = params_.Q_yaw;
    std::vector<double> R_yaw = params_.R_yaw;

    Eigen::MatrixXd A{{1, DT}, {0, 1}};
    Eigen::MatrixXd B{{0}, {DT}};
    Eigen::VectorXd f{{0, 0}};
    Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
    Eigen::Matrix<double, 1, 1> R(R_yaw.data());

    tiny_setup(&yaw_planner_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

    Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
    tiny_set_bound_constraints(yaw_planner_, x_min, x_max, u_min, u_max);

    yaw_planner_->settings->max_iter = 10;
}

void GimbalPlannerNode::init_pitch_planner() {
    auto max_pitch_acc = params_.max_pitch_acc;
    std::vector<double> Q_pitch = params_.Q_pitch;
    std::vector<double> R_pitch = params_.R_pitch;

    Eigen::MatrixXd A{{1, DT}, {0, 1}};
    Eigen::MatrixXd B{{0}, {DT}};
    Eigen::VectorXd f{{0, 0}};
    Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
    Eigen::Matrix<double, 1, 1> R(R_pitch.data());
    tiny_setup(&pitch_planner_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

    Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
    tiny_set_bound_constraints(pitch_planner_, x_min, x_max, u_min, u_max);

    pitch_planner_->settings->max_iter = 10;
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::GimbalPlannerNode)