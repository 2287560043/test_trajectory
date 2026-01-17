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

#include "ArmorPredictorNode.hpp"
#include <rclcpp_lifecycle/state.hpp>

namespace helios_cv
{

using namespace std::chrono_literals;

ArmorPredictorNode::ArmorPredictorNode(const rclcpp::NodeOptions& options) : rclcpp::Node("ArmorPredictorNode", options)
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
  node_namespace_ = this->get_namespace();
  if (node_namespace_.size() == 1)
  {
    node_namespace_.clear();
  }
  if (node_namespace_ == "/right")
  {
    gimbal_id_ = 1;
  }
  else
  {
    gimbal_id_ = 0;
  }
  frame_namespace_ = node_namespace_;
  if (!frame_namespace_.empty())
  {
    frame_namespace_.erase(frame_namespace_.begin());
    frame_namespace_ = frame_namespace_ + "_";
  }
  init_predictors();
  // create cam info subscriber
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      node_namespace_ + "/camera_info", rclcpp::SensorDataQoS(),
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
  // create viusalization markers
  create_visualization_markers();
  // create publishers and subscribers
  target_pub_ = this->create_publisher<autoaim_interfaces::msg::Target>(node_namespace_ + "/predictor/target", 10);
  // init tf2 utilities
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, node_namespace_ + "/detector/armors", rmw_qos_profile_sensor_data);
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_ = std::make_shared<tf2_filter>(armors_sub_, *tf2_buffer_, frame_namespace_ + params_.target_frame, 10,
                                             this->get_node_logging_interface(), this->get_node_clock_interface(),
                                             std::chrono::duration<int>(2));
  tf2_filter_->registerCallback(&ArmorPredictorNode::armor_predictor_callback, this);

}

void ArmorPredictorNode::init_predictors()
{
  vehicle_observer_ = std::make_shared<StandardObserver>(StandardObserverParams{
      static_cast<int>(params_.max_lost), static_cast<int>(params_.max_detect), params_.lost_time_thres_, params_.target_frame,
      StandardObserverParams::DDMParams{
          params_.standard_observer.ekf.sigma2_q_xyz, params_.standard_observer.ekf.sigma2_q_yaw,
          params_.standard_observer.ekf.sigma2_q_r, params_.standard_observer.ekf.sigma2_q_z,
          params_.standard_observer.ekf.r_xyz_factor, params_.standard_observer.ekf.r_yaw,
          params_.standard_observer.ekf.r_z_factor },
      params_.is_sentry, params_.priority_sequence });
  vehicle_observer_->target_type_ = TargetType::NORMAL;
  last_target_type_ = TargetType::NORMAL;
}

void ArmorPredictorNode::create_visualization_markers()
{
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;

  text_marker_.ns = "armor_text";
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.color.a = 1.0;

  target_armor_marker_.ns = "armors";
  target_armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  target_armor_marker_.scale.x = 0.03;
  target_armor_marker_.scale.z = 0.125;
  target_armor_marker_.color.a = 1.0;
  target_armor_marker_.color.r = 1.0;

  target_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      node_namespace_ + "/predictor/target_marker", rclcpp::SystemDefaultsQoS());
}

void ArmorPredictorNode::armor_predictor_callback(autoaim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  geometry_msgs::msg::TransformStamped ts_odom2cam, ts_cam2odom;
  try {
      ts_odom2cam = tf2_buffer_->lookupTransform(
          frame_namespace_ + "camera_optical_frame",
          frame_namespace_ + "odoom",
          armors_msg->header.stamp,
          rclcpp::Duration::from_seconds(0.02)
      );
      ts_cam2odom = tf2_buffer_->lookupTransform(
          frame_namespace_ + "odoom",
          frame_namespace_ + "camera_optical_frame",
          armors_msg->header.stamp,
          rclcpp::Duration::from_seconds(0.02)
      );
      auto odom2yawlink = tf2_buffer_->lookupTransform(
          frame_namespace_ + "yaw_link",
          frame_namespace_ + "odoom",
          armors_msg->header.stamp,
          rclcpp::Duration::from_seconds(0.02)
      );
      tf2::Quaternion q(
          odom2yawlink.transform.rotation.x,
          odom2yawlink.transform.rotation.y,
          odom2yawlink.transform.rotation.z,
          odom2yawlink.transform.rotation.w
      );
      tf2::Matrix3x3 m(q);
      // blank roll and pitch, not using them
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      gimbal_yaw_ = yaw;
  } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR_ONCE(get_logger(), "Error while transforming %s", ex.what());
      return;
  }
  
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_WARN(logger_, "Parameters updated");
    update_predictor_params();
  }
  if (params_.autoaim_mode != 0) {
    return;
  }
  // build time series
  rclcpp::Time time = armors_msg->header.stamp;

  // dt = predict_time + 下位机耗时 + detect_time ????
  double dt = time.seconds() - time_predictor_start_;
  if (dt > 1.0) {
    dt = 0.01;
  }
  time_predictor_start_ = time.seconds();

  if (dt < 0) {
  // RCLCPP_INFO(logger_, "Current time: %.10f", time.seconds());
  // RCLCPP_INFO(logger_, "Last time: %.10f", time_predictor_start_);
  // RCLCPP_INFO(logger_, "Time difference (dt): %.10f", dt);
    return ;
  }

  // transform armors to target frame
  for (auto& armor : armors_msg->armors)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try
    {
      armor.pose = tf2_buffer_->transform(ps, frame_namespace_ + params_.target_frame).pose;
    }
    catch (const tf2::ExtrapolationException& ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
    //  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
    //             "PoseStamped: position: [x: %f, y: %f, z: %f], orientation: [x: %f, y: %f, z: %f, w: %f]",
    //             ps.pose.position.x, ps.pose.position.y, ps.pose.position.z,
    //             ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w);
    //       RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
    //             "PoseStamped: position: [x: %f, y: %f, z: %f], orientation: [x: %f, y: %f, z: %f, w: %f]",
    //             armor.pose.position.x, armor.pose.position.y, armor.pose.position.z,
    //             armor.pose.orientation.x, armor.pose.orientation.y, armor.pose.orientation.z, armor.pose.orientation.w);
  }

  // doing predict
  // RCLCPP_INFO(logger_, "gimbal_yaw: %.10f, dt: %.10f", gimbal_yaw_, dt);

  target_msg_ = vehicle_observer_->predict_target(*armors_msg, dt, gimbal_yaw_, 28.0);
  target_msg_.gimbal_id = gimbal_id_;
  // choose predict mode
  update_predictor_type(vehicle_observer_);
  Eigen::Vector3d target_position =
      Eigen::Vector3d{ target_msg_.position.x, target_msg_.position.y, target_msg_.position.z };
  target_msg_.tracking = 1;
  if (target_msg_.tracking)
  {
    last_target_distance_ = target_position.norm();
  }
  target_msg_.header.stamp = armors_msg->header.stamp;
  target_msg_.header.frame_id = frame_namespace_ + params_.target_frame;
  check_and_kill_invalid(target_msg_);
  target_pub_->publish(target_msg_);
  if (params_.debug)
  {
    get_marker_array(target_msg_);
  }
}

void ArmorPredictorNode::get_marker_array(autoaim_interfaces::msg::Target target)
{
  target_marker_array_.markers.clear();
  
  target_armor_marker_.header = target.header;
  position_marker_.header = target.header;
  text_marker_.header = target.header;

  double zc = target.position.z - target.dz / 2;

  position_marker_.ns = "position";
  position_marker_.id = 0;
  if (target.tracking) {
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position = target.position;
    position_marker_.pose.position.z = zc;
  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
  }
  target_marker_array_.markers.emplace_back(position_marker_);


  if (target.tracking)
  {
    if (target.armors_num == 1) {
      target.yaw = 0;
      target.radius_1 = 0.0;
      target.radius_2 = 0.0;
      target.dz = 0.0;
    }

    double yaw = target.yaw;
    double r1 = target.radius_1;
    double r2 = target.radius_2;
    double xc = target.position.x;
    double yc = target.position.y;
    double dz = target.dz;
    double z = target.position.z;

    bool is_current_pair = true;
    size_t a_n = target.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;

    for (size_t i = 0; i < a_n; i++)
    {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = z - (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = zc;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      target_armor_marker_.action = visualization_msgs::msg::Marker::ADD;
      target_armor_marker_.id = i;
      target_armor_marker_.pose.position = p_a;
      
      tf2::Quaternion q;
      q.setRPY(0, target.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
      target_armor_marker_.pose.orientation = tf2::toMsg(q);

      target_armor_marker_.scale.y = (target.armor_type == "SMALL") ? 0.135 : 0.23;

      text_marker_.action = visualization_msgs::msg::Marker::ADD;
      text_marker_.id = i;
      text_marker_.pose.position = p_a;
      text_marker_.pose.position.z += 0.25;

      if (static_cast<int>(i) == target.armor_id) {
        target_armor_marker_.color.r = 0.0; 
        target_armor_marker_.color.g = 1.0; 
        target_armor_marker_.color.b = 0.0;
        
        text_marker_.text = std::to_string(i);
        text_marker_.scale.z = 0.2; 
        text_marker_.color.r = 1.0; 
        text_marker_.color.g = 1.0; 
        text_marker_.color.b = 0.0;
      } else {
        target_armor_marker_.color.r = 1.0; 
        target_armor_marker_.color.g = 1.0; 
        target_armor_marker_.color.b = 1.0;

        text_marker_.text = std::to_string(i);
        text_marker_.scale.z = 0.2;
        text_marker_.color.r = 1.0; 
        text_marker_.color.g = 1.0; 
        text_marker_.color.b = 1.0;
      }

      target_marker_array_.markers.emplace_back(target_armor_marker_);
      target_marker_array_.markers.emplace_back(text_marker_);
    }
  }
  else
  {
    target_armor_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    target_marker_array_.markers.emplace_back(target_armor_marker_);
    text_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    target_marker_array_.markers.emplace_back(text_marker_);
  }

  target_marker_pub_->publish(target_marker_array_);
}

void ArmorPredictorNode::update_predictor_params()
{
  if (vehicle_observer_->target_type_ == TargetType::NORMAL)
  {
    auto params = StandardObserverParams{ static_cast<int>(params_.max_lost),
                                          static_cast<int>(params_.max_detect),
                                          params_.lost_time_thres_,
                                          params_.target_frame,
                                          StandardObserverParams::DDMParams{ params_.standard_observer.ekf.sigma2_q_xyz,
                                                                             params_.standard_observer.ekf.sigma2_q_yaw,
                                                                             params_.standard_observer.ekf.sigma2_q_r,
                                                                             params_.standard_observer.ekf.sigma2_q_z,
                                                                             params_.standard_observer.ekf.r_xyz_factor,
                                                                             params_.standard_observer.ekf.r_yaw,
                                                                             params_.standard_observer.ekf.r_z_factor },
                                          params_.is_sentry,
                                          params_.priority_sequence };
    vehicle_observer_->set_params(&params);
  }
  else if (vehicle_observer_->target_type_ == TargetType::OUTPOST)
  {
    auto params = OutpostObserverParams{ static_cast<int>(params_.max_lost),
                                         static_cast<int>(params_.max_detect),
                                         params_.lost_time_thres_,
                                         params_.target_frame,
                                         OutpostObserverParams::DDMParams{ params_.outpost_observer.ekf.sigma2_q_yaw,
                                                                           params_.outpost_observer.ekf.sigma2_q_xyz,
                                                                           params_.outpost_observer.ekf.r_xyz_factor,
                                                                           params_.outpost_observer.ekf.r_yaw },
                                         params_.is_sentry,
                                         params_.priority_sequence };
    vehicle_observer_->set_params(&params);
  }
}

void ArmorPredictorNode::update_predictor_type(std::shared_ptr<BaseObserver>& observer)
{
  if (last_target_type_ == observer->target_type_)
  {
    // RCLCPP_INFO(logger_, "target state not changed %d", last_target_type_);
    return;
  }
  if (observer->target_type_ == TargetType::NORMAL && last_target_type_ != TargetType::NORMAL)
  {
    observer = std::make_shared<StandardObserver>(StandardObserverParams{
        static_cast<int>(params_.max_lost), static_cast<int>(params_.max_detect), params_.lost_time_thres_, params_.target_frame,
        StandardObserverParams::DDMParams{
            params_.standard_observer.ekf.sigma2_q_xyz, params_.standard_observer.ekf.sigma2_q_yaw,
            params_.standard_observer.ekf.sigma2_q_r, params_.standard_observer.ekf.sigma2_q_z,
            params_.standard_observer.ekf.r_xyz_factor, params_.standard_observer.ekf.r_yaw,
            params_.standard_observer.ekf.r_z_factor },
        params_.is_sentry, params_.priority_sequence });
    last_target_type_ = TargetType::NORMAL;
    RCLCPP_ERROR(logger_, "change to normal");
  }
  else if (observer->target_type_ == TargetType::OUTPOST && last_target_type_ != TargetType::OUTPOST)
  {
    observer = std::make_shared<OutpostObserver>(OutpostObserverParams{
        static_cast<int>(params_.max_lost), static_cast<int>(params_.max_detect), params_.lost_time_thres_, params_.target_frame,
        OutpostObserverParams::DDMParams{
            params_.outpost_observer.ekf.sigma2_q_yaw, params_.outpost_observer.ekf.sigma2_q_xyz,
            params_.outpost_observer.ekf.r_xyz_factor, params_.outpost_observer.ekf.r_yaw },
        params_.is_sentry, params_.priority_sequence });
    RCLCPP_ERROR(logger_, "change to outpost");
    last_target_type_ = TargetType::OUTPOST;
  }
  else
  {
    RCLCPP_INFO(logger_, "last type %d", last_target_type_);
    RCLCPP_INFO(logger_, "this type %d", observer->target_type_);
  }
}

void ArmorPredictorNode::check_and_kill_invalid(const autoaim_interfaces::msg::Target& target_msg) {
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

ArmorPredictorNode::~ArmorPredictorNode()
{
  // file.close();
}

}  // namespace helios_cv

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::ArmorPredictorNode);
