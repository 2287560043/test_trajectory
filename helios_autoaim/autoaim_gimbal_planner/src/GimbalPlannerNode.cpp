#include "autoaim_gimbal_planner/GimbalPlannerNode.hpp"
#include <autoaim_interfaces/msg/detail/planned_target__struct.hpp>

namespace helios_cv {

GimbalPlannerNode::GimbalPlannerNode(const rclcpp::NodeOptions & options) 
: Node("gimbal_trajectory_node", options),
  yaw_planner_(nullptr),
  pitch_planner_(nullptr),
  time_planner_start_(0.0)
{
    param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();

    yaw_offset_ = params_.yaw_offset / 57.3;
    pitch_offset_ = params_.pitch_offset / 57.3;
    fire_thresh_ = params_.fire_thresh;
    decision_speed_ = params_.decision_speed;
    high_speed_delay_time_ = params_.high_speed_delay_time;
    low_speed_delay_time_ = params_.low_speed_delay_time;

    
    setup_yaw_planner();
    setup_pitch_planner();

    target_sub_ = this->create_subscription<autoaim_interfaces::msg::Target>(
        "/predictor/target", rclcpp::SensorDataQoS(),
        std::bind(&GimbalPlannerNode::gimbal_planner_callback, this, std::placeholders::_1));
    planned_target_pub_ = this->create_publisher<autoaim_interfaces::msg::PlannedTarget>(
            "/gimbal_planner/planned_target", rclcpp::SensorDataQoS());

    // tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // auto timer_interface = 
    //     std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    
    // tf2_buffer_->setCreateTimerInterface(timer_interface);
    // tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    // target_sub_.subscribe(this, "/predictor/target", rmw_qos_profile_sensor_data);
    // tf2_filter_ = std::make_shared<tf2_filter>(target_sub_, *tf2_buffer_, "odoom", 10,
    //                                          this->get_node_logging_interface(), this->get_node_clock_interface(),
    //                                          std::chrono::duration<int>(2));
    // tf2_filter_->registerCallback(&GimbalPlannerNode::gimbal_planner_callback, this);
}

GimbalPlannerNode::~GimbalPlannerNode()
{
    RCLCPP_INFO(logger_, "GimbalPlannerNode destroyed");
}
void GimbalPlannerNode::gimbal_planner_callback(const autoaim_interfaces::msg::Target::SharedPtr target_msg) 
{
    // for (int i = 0; i < HORIZON; i++) {
    //     RCLCPP_ERROR(logger_, "yaw: %f, yaw_vel: %f, pitch: %f, pitch_vel: %f", 
    //         target_msg->pretraj[i].yaw, target_msg->pretraj[i].yaw_vel, 
    //         target_msg->pretraj[i].pitch, target_msg->pretraj[i].pitch_vel);
    // }
    
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
        RCLCPP_WARN(logger_, "params updated");
    }

    rclcpp::Time time = target_msg->header.stamp;
    double dt = time.seconds() - time_planner_start_;
    time_planner_start_ = time.seconds();

    if (dt < 0) return;

    Eigen::Matrix<double, 4, HORIZON> pretraj;
    if (target_msg->pretraj.size() != HORIZON) {
        // RCLCPP_ERROR(logger_, "pretraj size not equal to HORIZON: %ld", target_msg->pretraj.size());
        return;
    }
    for (int i = 0; i < HORIZON; i++) {
        // RCLCPP_ERROR(logger_, "yaw: %f, yaw_vel: %f, pitch: %f, pitch_vel: %f", 
        //     target_msg->pretraj[i].yaw, target_msg->pretraj[i].yaw_vel, 
        //     target_msg->pretraj[i].pitch, target_msg->pretraj[i].pitch_vel);
        pretraj.col(i) << 
        target_msg->pretraj[i].yaw, 
        target_msg->pretraj[i].yaw_vel, 
        target_msg->pretraj[i].pitch, 
        target_msg->pretraj[i].pitch_vel;
        
    }
    double yaw0 = target_msg->yaw0;

    // RCLCPP_ERROR(logger_, "yaw0: %f", yaw0);
    // RCLCPP_ERROR(logger_, "pretraj(HALF_HORIZON, 0): %f", pretraj(0, HALF_HORIZON));
    // RCLCPP_ERROR(logger_, "pretraj(HALF_HORIZON, 1): %f", pretraj(2, HALF_HORIZON));

    if (!yaw_planner_ || !yaw_planner_->work || !pitch_planner_ || !pitch_planner_->work) {
        RCLCPP_ERROR(logger_, "CRITICAL: Planner pointers are invalid. Skipping solve.");
        return; 
    }
    // solve yaw and pitch
    Eigen::VectorXd x0(2);
    x0 << pretraj(0, 0), pretraj(1, 0);
    tiny_set_x0(yaw_planner_, x0);
    yaw_planner_->work->Xref = pretraj.block(0, 0, 2, HORIZON);
    tiny_solve(yaw_planner_);

    x0 << pretraj(2, 0), pretraj(3, 0);
    tiny_set_x0(pitch_planner_, x0);
    pitch_planner_->work->Xref = pretraj.block(2, 0, 2, HORIZON);
    tiny_solve(pitch_planner_);


    autoaim_interfaces::msg::PlannedTarget plan;
    plan.control = true;

    plan.target_yaw = math::get_rad(pretraj(0, HALF_HORIZON) + yaw0);
    plan.target_pitch = pretraj(2, HALF_HORIZON);

    plan.yaw = math::get_rad(yaw_planner_->work->x(0, HALF_HORIZON) + yaw0);
    plan.yaw_vel = yaw_planner_->work->x(1, HALF_HORIZON);
    plan.yaw_acc = yaw_planner_->work->u(0, HALF_HORIZON);

    plan.pitch = pitch_planner_->work->x(0, HALF_HORIZON);
    plan.pitch_vel = pitch_planner_->work->x(1, HALF_HORIZON);
    plan.pitch_acc = pitch_planner_->work->u(0, HALF_HORIZON);


    auto shoot_offset_ = 2;
    plan.fire = std::hypot( 
                    pretraj(0, HALF_HORIZON + shoot_offset_) - 
                        yaw_planner_->work->x(0, HALF_HORIZON + shoot_offset_),
                    pretraj(2, HALF_HORIZON + shoot_offset_) - 
                        pitch_planner_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
    
    planned_target_msg_ = plan;
    planned_target_msg_.header.stamp = target_msg->header.stamp;
    planned_target_pub_->publish(planned_target_msg_);
    // RCLCPP_ERROR(logger_, "send_to_mcu---yaw: %f, yaw_vel: %f, fire: %s", 
    //                                 plan.yaw, plan.yaw_vel, plan.fire ? "true" : "false");
}

void GimbalPlannerNode::setup_yaw_planner() {
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

void GimbalPlannerNode::setup_pitch_planner() {
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