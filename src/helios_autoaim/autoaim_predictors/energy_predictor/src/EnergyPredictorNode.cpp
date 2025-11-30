// created by liuhan on 2023/10/29
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */

#include "EnergyPredictorNode.hpp"
#include <rclcpp/logging.hpp>
#include <string>

namespace helios_cv
{

using namespace std::chrono_literals;

EnergyPredictorNode::EnergyPredictorNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("EnergyPredictorNode", options)
{
  // create params
  try
  {
    param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(logger_, "Failed to get parameters: %s, use empty params", e.what());
  }
  // create cam info subscriber
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
        cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
        cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        cam_info_sub_.reset();
      });
  serial_sub_ = this->create_subscription<autoaim_interfaces::msg::ReceiveData>(
    "/receive_data", rclcpp::SensorDataQoS(), 
    [this] (autoaim_interfaces::msg::ReceiveData::SharedPtr mcu_packet) {
      if (last_autoaim_mode_ != mcu_packet->autoaim_mode) {
        rclcpp::Parameter param("autoaim_mode", mcu_packet->autoaim_mode);
        param_listener_->update({param});
        omega_.energy_mode_examed_ = false;
        omega_.energy_exam_cnt_ = 0;
      }
    }
  );
  // create viusalization markers
  create_visualization_markers();
  init_kalman();
  reset_observer();
  // create publishers and subscribers
  target_pub_ = this->create_publisher<autoaim_interfaces::msg::Target>("/predictor/target", 10);
  debug_energy_pub_ = this->create_publisher<autoaim_interfaces::msg::DebugEnergyVel>("/predictor/debug_energy",
                                                                                      rclcpp::SystemDefaultsQoS());
  ceres_pub_ =
      this->create_publisher<autoaim_interfaces::msg::Omega>("/predictor/solve_problem", rclcpp::SystemDefaultsQoS());
  ceres_sub_ = this->create_subscription<autoaim_interfaces::msg::Omega>(
      "/predictor/solve_answer", rclcpp::SystemDefaultsQoS(),
      [this](autoaim_interfaces::msg::Omega::SharedPtr msg) { omega_.set_a_w_phi(msg->a, msg->w, msg->phi); });
  // init tf2 utilities
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_ = std::make_shared<tf2_filter>(armors_sub_, *tf2_buffer_, params_.target_frame, 10,
                                             this->get_node_logging_interface(), this->get_node_clock_interface(),
                                             std::chrono::duration<int>(2));
  tf2_filter_->registerCallback(&EnergyPredictorNode::energy_predictor_callback, this);
}

void EnergyPredictorNode::init_kalman()
{
  // Init
  find_state_ = LOST;
  reset_observer();
  // Create kalman
  auto f = [this](const Eigen::VectorXd&) {
    Eigen::MatrixXd f(3, 3);
    double t2 = 0.5 * dt_ * dt_;
    // clang-format off
        f << 1,  dt_, t2,
             0,  1,   dt_,
             0,  0,   1;
    // clang-format on
    return f;
  };
  auto h = [this](const Eigen::VectorXd&) {
    Eigen::MatrixXd h(2, 3);
    // clang-format off
        h << 1, 0, 0,
             0, 1, 0;
    // clang-format on
    return h;
  };
  auto q = [this]() {
    Eigen::MatrixXd q(3, 3);
    // clang-format off
        q << params_.kf_params.sigma_q_x, 0, 0,
             0, params_.kf_params.sigma_q_x, 0,
             0, 0, params_.kf_params.sigma_q_x;
    // clang-format on
    return q;
  };
  auto r = [this](const Eigen::VectorXd& z) {
    // Eigen::DiagonalMatrix<double, 2> r;
    // r.setIdentity();
    Eigen::MatrixXd r(2, 2);
          r << params_.kf_params.sigma_r_x, 0,
            0, params_.kf_params.sigma_r_x;
    return r;
  };
  Eigen::DiagonalMatrix<double, 3> p;
  p.setIdentity();
  omega_kf_ = EigenKalmanFilter(f, h, q, r, p);
}

void EnergyPredictorNode::reset_observer()
{
  omega_.refresh();
  isSolve_ = false;
  ceres_cnt_ = 1;
  refresh_ = true;
  circle_mode_ = static_cast<int>(INIT);
  omega_.energy_mode_examed_ = false;
  omega_.energy_exam_cnt_ = 0;
  omega_kf_.set_state(Eigen::Vector3d::Zero());
  RCLCPP_WARN(logger_, "Reset Energy Observer");
}

void EnergyPredictorNode::create_visualization_markers()
{
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;

  target_energy_marker_.ns = "armors";
  target_energy_marker_.type = visualization_msgs::msg::Marker::CUBE;
  target_energy_marker_.scale.x = 0.03;
  target_energy_marker_.scale.y = 0.370;
  target_energy_marker_.scale.z = 0.430;
  target_energy_marker_.color.a = 1.0;
  target_energy_marker_.color.r = 1.0;

  target_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/predictor/target_marker",
                                                                                    rclcpp::SystemDefaultsQoS());
}

void EnergyPredictorNode::track_energy(const autoaim_interfaces::msg::Armor& armor)
{
  matched_ = false;
  // When we hit one armor, while the armor switch, we can only see the energy fan or nothing
  // we should take this as temp lost, don't reset params and keep predicting
  if (armor.type == static_cast<int>(ArmorType::ENERGY_TARGET))
  {
    matched_ = true;
    float omega_origin = orientation2roll(armor.pose.orientation);
    last_roll_ = last_roll_ + angles::shortest_angular_distance(last_roll_, omega_origin);
    omega_.set_theta(last_roll_);

    double omega_diff = omega_.current_theta_ - omega_.last_current_theta_;
    //std::cout<<"omega_diff:"<<omega_diff<<std::endl;
    if(fabs(omega_diff) > 0.4){
      omega_.current_theta_ = omega_.last_current_theta_ + (omega_diff > 0 ? 0.1 : -0.1);
      RCLCPP_WARN(logger_, "omega_diff :%f", omega_diff);
    }

    if (!omega_.start_ or params_.autoaim_mode == 1)
    {
      a_ = 0;
      w_ = 0;
      phi_ = 0;
      b_ = 0;
    }
    else
    { 
      pub_time_ = omega_.time_series_.back();
      // Get omega from kalman filter
      Eigen::VectorXd measure(2);
      measure << omega_.total_theta_, omega_.current_theta_;
      omega_kf_.predict();
      auto state = omega_kf_.correct(measure);
      double omega = state(1);
      omega_.set_filter(omega);
      // if(not omega_.energy_mode_examed_){
      //   energy_mode_ = omega_.judge_mode(std_dev);
      //   omega_.energy_mode_examed_ = (energy_mode_ != 0);
      //   omega_.energy_exam_cnt_ = 0;
      // }
      // std::cout<<"mode: "<<energy_mode_<<std::endl;
      if (params_.autoaim_mode == 2)
      {
        // Set ceres' problem
        autoaim_interfaces::msg::Omega problem;
        problem.a = omega_.a_;
        problem.w = omega_.w_;
        problem.phi = omega_.phi_;
        problem.t = pub_time_;
        //problem.b = omega_.b_;
        problem.st = omega_.st_;
        problem.omega = omega_.filter_omega_.back();
        problem.solve = isSolve_;
        problem.refresh = refresh_;

        ceres_pub_->publish(problem);
        refresh_ = false;

        if (energy_state_switch())
        {
          autoaim_interfaces::msg::DebugEnergyVel debug_energy;
          debug_energy.header.stamp = this->now();
          debug_energy.omega_raw = omega_.current_theta_;
          debug_energy.omega_filter = omega;
          debug_energy.omega_fit = omega_.get_omega();
          debug_energy.omega_roll = omega_origin;
          debug_energy_pub_->publish(debug_energy);
          if (std::fabs(omega_.get_err()) > 0.3)
          {
            //RCLCPP_WARN(logger_, "get_err:%f",omega_.get_err());
            omega_.fit_cnt_++;
            if ((omega_.fit_cnt_ % 40 == 0))
            {
              // std::cout<<"fit_cnt_: "<<omega_.fit_cnt_<<std::endl;
              circle_mode_ = STANDBY;
              omega_.change_st();
            }
          }
          a_ = omega_.a_;
          w_ = omega_.w_;
          phi_ = omega_.phi_;
          //b_ = omega_.b_;
        }
      }
    }
    last_tracking_armor_ = armor;
  }
}

bool EnergyPredictorNode::energy_state_switch() {
  switch (circle_mode_) {
    case INIT:
      if (omega_.FindWavePeak()) { 
        circle_mode_ = STANDBY;
        isSolve_ = false;
        omega_.refresh_after_wave(); 
      }
      return false;
    case STANDBY: {
      auto time_gap = omega_.get_time_gap();
      RCLCPP_WARN(logger_, "time_gap:%f",time_gap);
      if (time_gap > 1.0) {  // this can be optimized
        circle_mode_ = ESTIMATE;
      }
      return false;
    }
    case ESTIMATE:
      ceres_cnt_++;
      isSolve_ = true;
      circle_mode_ = PREDICT;
      return true;
    case PREDICT:
      isSolve_ = false;
      return true;
    default:
      return true;
  }
}

void EnergyPredictorNode::energy_predictor_callback(autoaim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    if (params_.autoaim_mode != last_autoaim_mode_)
    {
      find_state_ = LOST;
      omega_.energy_mode_examed_ = false;
      omega_.energy_exam_cnt_ = 0;
      last_autoaim_mode_ = params_.autoaim_mode;
    }
    RCLCPP_WARN(logger_, "Parameters updated");
  }
  if (params_.autoaim_mode == 0) {
    return;
  }
  // build time series
  rclcpp::Time time = armors_msg->header.stamp;
  double dt = time.seconds() - time_predictor_start_;
  time_predictor_start_ = time.seconds();
  // transform armors to target frame
  for (auto& armor : armors_msg->armors)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try
    {
      armor.pose = tf2_buffer_->transform(ps, params_.target_frame).pose;
    }
    catch (const tf2::ExtrapolationException& ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }
  // Start Prediction
  dt_ = dt;
  if (dt > 1.5)
  {
    find_state_ = TEMP_LOST;
  }
  autoaim_interfaces::msg::Target target;
  target.header.frame_id = params_.target_frame;
  target.header.stamp = armors_msg->header.stamp;
  if (armors_msg->armors.empty())
  {
    target.tracking = false;
    matched_ = false;
    tracking_armor_.type = static_cast<int>(ArmorType::INVALID);
  }
  else
  {
    // Find target armor
    tracking_armor_ = armors_msg->armors[0];
    bool is_target_find = false;
    for (auto armor : armors_msg->armors)
    {
      if (armor.type == static_cast<int>(ArmorType::ENERGY_TARGET))
      {
        tracking_armor_ = armor;
        is_target_find = true;
        break;
      }
    }
    if (is_target_find)
    {
      // Start observation
      if (find_state_ == LOST)
      {
        reset_observer();
        find_state_ = DETECTING;
      }
      else
      {
        omega_.set_time(rclcpp::Time(armors_msg->header.stamp).seconds());
        track_energy(tracking_armor_);
        if (find_state_ == TRACKING || find_state_ == TEMP_LOST)
        {
          // Pack data
          // if (a_ == 0 && w_ == 0 && phi_ == 0)
          // {
          //   a_ = 0.9125;
          //   w_ = 1.942;
          //   phi_ = M_PI_2;
          //   b_ = 1.1775;
          // }
          target.tracking = true;
          target.velocity.x = a_;
          target.velocity.y = w_;
          target.velocity.z = phi_;
          target.armors_num = 5;
          target.armor_type = "energy";
          target.radius_1 = 0.7;
          target.radius_2 = omega_.t_list_.back();
          double roll, pitch, yaw;
          tf2::Quaternion q(tracking_armor_.pose.orientation.x, tracking_armor_.pose.orientation.y,
                            tracking_armor_.pose.orientation.z, tracking_armor_.pose.orientation.w);
          tf2::Matrix3x3 m(q);
          m.getRPY(roll, pitch, yaw);
          target.yaw = yaw;
          target.v_yaw = roll;
          auto center_position = get_energy_center(tracking_armor_, yaw);
          target.position.x = center_position(0);
          target.position.y = center_position(1);
          target.position.z = center_position(2);
        }
      }
    }
  }
  // Update state machine
  if (find_state_ == DETECTING)
  {
    if (matched_)
    {
      detect_cnt_++;
      if (detect_cnt_ > 5)
      {
        detect_cnt_ = 0;
        find_state_ = TRACKING;
      }
    }
    find_state_ = TRACKING;
  }
  else if (find_state_ == TRACKING)
  {
    if (!matched_)
    {
      find_state_ = TEMP_LOST;
      lost_cnt_++;
    }
  }
  else if (find_state_ == TEMP_LOST)
  {
    if (!matched_)
    {
      lost_cnt_++;
      if (lost_cnt_ > 20)
      {
        RCLCPP_WARN(logger_, "Target Lost!");
        lost_cnt_ = 0;
        find_state_ = LOST;
      }
    }
    else
    {
      find_state_ = TRACKING;
      lost_cnt_ = 0;
    }
  }
  check_and_kill_invalid(target);
  // Publish
  target_pub_->publish(target);
  if (params_.debug)
  {
    get_marker_array(target);
  }
}

double EnergyPredictorNode::orientation2roll(const geometry_msgs::msg::Quaternion& orientation)
{
  double roll, pitch, yaw;
  tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  return roll;
}

Eigen::Vector3d EnergyPredictorNode::get_energy_center(const autoaim_interfaces::msg::Armor& armor, double yaw)
{
  double roll = orientation2roll(armor.pose.orientation);
  double xc = armor.pose.position.x + 0.7 * std::sin(-roll) * std::sin(yaw),
         yc = armor.pose.position.y - 0.7 * std::sin(-roll) * std::cos(yaw),
         zc = armor.pose.position.z - 0.7 * std::cos(-roll);
  return { xc, yc, zc };
}

void EnergyPredictorNode::get_marker_array(autoaim_interfaces::msg::Target target)
{
  /// Clear marker array
  target_marker_array_.markers.clear();
  target_energy_marker_.header = target.header;
  position_marker_.header = target.header;
  /// Push target marker
  if (target.tracking)
  {
    double yaw = target.yaw, roll = target.v_yaw;
    double r = target.radius_1;
    double a = target.velocity.x, w = target.velocity.y, phi = target.velocity.z;
    double xc = target.position.x, yc = target.position.y, zc = target.position.z;
    // Energy Center
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = zc;
    target_marker_array_.markers.emplace_back(position_marker_);
    // Energy Fan
    target_energy_marker_.action = visualization_msgs::msg::Marker::ADD;
    size_t a_n = target.armors_num;
    geometry_msgs::msg::Point p_a;
    for (size_t i = 0; i < a_n; i++)
    {
      double tmp_roll = roll + i * (2 * M_PI / a_n);
      p_a.x = xc - r * std::sin(-tmp_roll) * std::sin(yaw);
      p_a.y = yc + r * std::sin(-tmp_roll) * std::cos(yaw);
      p_a.z = zc + r * std::cos(-tmp_roll);
      target_energy_marker_.id = i;
      target_energy_marker_.pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(tmp_roll, 0, yaw);
      target_energy_marker_.pose.orientation = tf2::toMsg(q);
      target_marker_array_.markers.emplace_back(target_energy_marker_);
    }
  }
  else
  {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    target_energy_marker_.action = visualization_msgs::msg::Marker::DELETE;
  }
  target_marker_array_.markers.emplace_back(position_marker_);
  target_marker_pub_->publish(target_marker_array_);
}

void EnergyPredictorNode::check_and_kill_invalid(const autoaim_interfaces::msg::Target& target_msg) {
  double sum = target_msg.yaw + target_msg.v_yaw + 
              target_msg.radius_1 + target_msg.radius_2 + target_msg.dz +
              target_msg.position.x + target_msg.position.y + target_msg.position.z +
              target_msg.velocity.x + target_msg.velocity.y + target_msg.velocity.z;
  bool is_invalid = std::isnan(sum) || std::isinf(sum);

  if (is_invalid) {
    RCLCPP_FATAL(rclcpp::get_logger("target_checker"), 
                  "Invalid number (NaN/Inf) in target message! Killing all nodes...");
    rclcpp::shutdown();
    system("ps aux | grep ros |  awk '{print $2}' | xargs kill -9; ps aux | grep rviz |  awk '{print $2}' | xargs kill -9");
  }
}

EnergyPredictorNode::~EnergyPredictorNode()
{
}

}  // namespace helios_cv

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::EnergyPredictorNode);
