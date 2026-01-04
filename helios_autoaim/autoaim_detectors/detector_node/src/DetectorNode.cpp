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

#include "DetectorNode.hpp"

#include "DetectorFactory.hpp"
#include "autoaim_interfaces/msg/receive_data.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoaim_utilities/Armor.hpp>
#include <autoaim_utilities/PnPSolver.hpp>
#include <detectors/DetectStream.hpp>
#include <detectors/OVNetArmorEnergyDetector.hpp>
#include <detectors/TraditionalArmorDetector.hpp>
#include <detectors/TraditionalEnergyDetector.hpp>
#include <exception>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <stdexcept>
#include <thread>

namespace helios_cv {

using namespace std::chrono_literals;

DetectorNode::DetectorNode(const rclcpp::NodeOptions& options):
    rclcpp::Node("detector_node", options) {
    // create params
    try {
        param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
        params_ = param_listener_->get_params();
        armor_use_traditional_ = params_.armor.use_traditional;
        energy_use_traditional_ = params_.energy.use_traditional;
    } catch (const std::exception& e) {
        RCLCPP_FATAL(logger_, "Failed to get parameters: %s, use empty params", e.what());
    }
    node_namespace_ = this->get_namespace();
    if (node_namespace_.size() == 1) {
        node_namespace_.clear();
    }
    frame_namespace_ = node_namespace_;
    if (!frame_namespace_.empty()) {
        frame_namespace_.erase(frame_namespace_.begin());
        frame_namespace_ = frame_namespace_ + "_";
    }
    // init detectors
    init_detectors();
    // init debug info
    binary_img_pub_ =
        image_transport::create_publisher(this, node_namespace_ + "/detector/binary_img");
    result_img_pub_ =
        image_transport::create_publisher(this, node_namespace_ + "/detector/result_img");
    number_img_pub_ =
        image_transport::create_publisher(this, node_namespace_ + "/detector/number_img");
    lights_data_pub_ = this->create_publisher<autoaim_interfaces::msg::DebugLights>(
        node_namespace_ + "/detector/debug_lights",
        10
    );
    armors_data_pub_ = this->create_publisher<autoaim_interfaces::msg::DebugArmors>(
        node_namespace_ + "/detector/debug_armors",
        10
    );
    // create publishers and subscribers
    armors_pub_ = this->create_publisher<autoaim_interfaces::msg::Armors>(
        node_namespace_ + "/detector/armors",
        10
    );
    // create cam info subscriber
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        node_namespace_ + "/camera_info",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::UniquePtr camera_info) {
            std::lock_guard<std::mutex> lock(detector_mutex_);
            cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
            detector_->set_cam_info(*camera_info);
            pnp_solver_ = std::make_shared<ArmorProjectYaw>(camera_info->k, camera_info->d);
            pnp_solver_->set_use_projection(params_.use_projection);
            cam_info_sub_.reset();
        }
    );
    serial_sub_ = this->create_subscription<autoaim_interfaces::msg::ReceiveData>(
        "/receive_data",
        rclcpp::SensorDataQoS(),
        [this](autoaim_interfaces::msg::ReceiveData::SharedPtr mcu_packet) {
            if (last_autoaim_mode_ != mcu_packet->autoaim_mode) {
                rclcpp::Parameter param("autoaim_mode", mcu_packet->autoaim_mode);
                this->set_parameter(param);
            }
            if (params_.is_blue != mcu_packet->target_color) {
                rclcpp::Parameter param("is_blue", mcu_packet->target_color);
                this->set_parameter(param);
            }
        }
    );
    auto image_qos = rclcpp::QoS(1)
                         .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                         .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        node_namespace_ + "/image_raw",
        image_qos,
        [this](sensor_msgs::msg::Image::UniquePtr image_msg) {
            armor_image_callback(std::move(image_msg));
        }
    );
    // init tf2 utilities
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
    );
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    // subscriber and filter
    // image_sub_.subscribe(this, node_namespace_ + "/image_raw", rmw_qos_profile_sensor_data);
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    // tf2_filter_ = std::make_shared<tf2_filter>(
    //     image_sub_,
    //     *tf2_buffer_,
    //     frame_namespace_ + "camera_optical_frame",
    //     10,
    //     this->get_node_logging_interface(),
    //     this->get_node_clock_interface(),
    //     std::chrono::duration<int>(0)
    // );
    // image_sub_.registerCallback(&DetectorNode::armor_image_callback, this);

    // Default register armor callback
    // tf2_filter_->registerCallback(&DetectorNode::armor_image_callback, this);

    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    armor_marker_.ns = "armors";
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.05;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.g = 0.5;
    armor_marker_.color.b = 1.0;
    armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    text_marker_.ns = "classification";
    text_marker_.action = visualization_msgs::msg::Marker::ADD;
    text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker_.scale.z = 0.1;
    text_marker_.color.a = 1.0;
    text_marker_.color.r = 1.0;
    text_marker_.color.g = 1.0;
    text_marker_.color.b = 1.0;
    text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

    // autoaim mode thread
    param_switch_thread = std::thread([this]() -> void {
        while (rclcpp::ok()) {
            if (param_listener_->is_old(params_)) {
                // RCLCPP_INFO(logger_, "Params start updated");
                std::unique_lock<std::mutex> lock(detector_mutex_);
                params_ = param_listener_->get_params();
                if ((params_.autoaim_mode != last_autoaim_mode_
                     || params_.armor.use_traditional != armor_use_traditional_
                     || params_.energy.use_traditional != energy_use_traditional_)
                    && cam_info_ != nullptr)
                {
                    switch_detector_stream();
                    last_autoaim_mode_ = params_.autoaim_mode;
                }
                try {
                    update_detector_params();
                } catch (std::bad_alloc& e) {
                    RCLCPP_ERROR(logger_, "Failed to update detector params: %s", e.what());
                }
            }
            std::this_thread::sleep_for(10ms);
        }
    });
}

void DetectorNode::init_detectors() {
    // create factory
    detector_factory_ = std::make_shared<DetectorFactory>();
    // create detectors, default create armor detector
    if (params_.armor.use_traditional) {
        detector_ = std::move(detector_factory_->create_traditional_armor_detector(params_));
    } else {
        detector_ = std::move(detector_factory_->create_ovnet_armor_energy_detector(params_));
    }
}

void DetectorNode::armor_image_callback(sensor_msgs::msg::Image::UniquePtr image_msg) {
    // convert image msg to cv::Mat
            auto image_header = image_msg->header;

    try {
        image_ = std::move(
            cv_bridge::toCvShare(std::move(image_msg), sensor_msgs::image_encodings::RGB8)->image

        );

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
        return;
    }
    // Get transform
    geometry_msgs::msg::TransformStamped ts_odom2cam, ts_cam2odom;
    double yaw;

    if (pnp_solver_ != nullptr) {
        armors_msg_.header = armor_marker_.header = text_marker_.header = image_header;

        armors_msg_.armors.clear();
        marker_array_.markers.clear();
        armor_marker_.id = 0;
        text_marker_.id = 0;
    }

    try {
        ts_odom2cam = tf2_buffer_->lookupTransform(
            frame_namespace_ + "camera_optical_frame",
            frame_namespace_ + "odoom",
            //timestamp from image
            image_header.stamp
        );
        ts_cam2odom = tf2_buffer_->lookupTransform(
            frame_namespace_ + "odoom",
            frame_namespace_ + "camera_optical_frame",
            image_header.stamp
        );
        auto odom2yawlink = tf2_buffer_->lookupTransform(
            frame_namespace_ + "yaw_link",
            frame_namespace_ + "odoom",
            image_header.stamp
        );
        tf2::Quaternion q(
            odom2yawlink.transform.rotation.x,
            odom2yawlink.transform.rotation.y,
            odom2yawlink.transform.rotation.z,
            odom2yawlink.transform.rotation.w
        );
        tf2::Matrix3x3 m(q);
        // blank roll and pitch, not using them
        double roll, pitch;
        m.getRPY(roll, pitch, yaw);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
        return;
    }
    // detect
    {
        std::unique_lock<std::mutex> lock(detector_mutex_);
        if (params_.autoaim_mode == AUTOAIM) {
            ArmorTransformInfo armor_transform_info { ros2cv(ts_odom2cam.transform.rotation),
                                                      ros2cv(ts_cam2odom.transform.rotation),
                                                      yaw };
            if (params_.armor.use_traditional) {
                auto input_image = std::make_shared<Image>(std::move(image_));
                armors_msg_ = ArmorEnergyDetectStream::detect(
                    detector_,
                    pnp_solver_,
                    input_image,
                    &armor_transform_info
                );
                armors_msg_.header = image_header;
            } else {
                auto input_image =
                    std::make_shared<ImageStamped>(std::move(image_), image_header.stamp);
                armors_msg_ = ArmorEnergyDetectStream::detect(
                    detector_,
                    pnp_solver_,
                    input_image,
                    &armor_transform_info
                );
                armors_msg_.header.frame_id = image_header.frame_id;
            }
        } else {
            EnergyTransformInfo energy_transform_info { ros2cv(ts_odom2cam.transform.rotation),
                                                        ros2cv(ts_cam2odom.transform.rotation),
                                                        yaw };
            if (params_.energy.use_traditional) {
                auto input_image = std::make_shared<Image>(std::move(image_));
                armors_msg_ = ArmorEnergyDetectStream::detect(
                    detector_,
                    pnp_solver_,
                    input_image,
                    &energy_transform_info
                );
                armors_msg_.header = image_header;
            } else {
                auto input_image =
                    std::make_shared<ImageStamped>(std::move(image_), image_header.stamp);
                armors_msg_ = ArmorEnergyDetectStream::detect(
                    detector_,
                    pnp_solver_,
                    input_image,
                    &energy_transform_info
                );
                armors_msg_.header.frame_id = image_header.frame_id;
            }
        }
        if (armors_msg_.header.stamp.nanosec == 0 && armors_msg_.header.stamp.sec == 0) {
            armors_msg_.header = image_header;
        }

        // publish
        armors_pub_->publish(armors_msg_);
        rclcpp::Time now = this->get_clock()->now();
        RCLCPP_INFO(
            logger_,
            "time cost: %.2f ms",
            (now - rclcpp::Time(image_header.stamp)).seconds() * 1000.0
        );
        // debug info
        if (params_.debug) {
            publish_debug_infos();
            // Publishing marker
            publishMarkers();
        }
    }
}

void DetectorNode::publishMarkers() {
    using Marker = visualization_msgs::msg::Marker;
    for (const auto& armor: armors_msg_.armors) {
        armor_marker_.id++;
        armor_marker_.scale.y = armor.type == 0 ? 0.135 : 0.23;
        armor_marker_.pose = armor.pose;
        text_marker_.id++;
        text_marker_.pose.position = armor.pose.position;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.number;
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);
    }
    armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
    marker_array_.markers.emplace_back(armor_marker_);
    marker_pub_->publish(marker_array_);
}

void DetectorNode::switch_detector_stream() {
    // Check if we need to switch anything
    if ((params_.autoaim_mode == AUTOAIM && params_.armor.use_traditional != armor_use_traditional_)
        || (params_.autoaim_mode != AUTOAIM
            && params_.energy.use_traditional != energy_use_traditional_)
        || (params_.autoaim_mode != last_autoaim_mode_))
    {
        // First, release all resources
        detector_.reset();
        pnp_solver_.reset();

        // Then recreate everything based on current mode
        if (params_.autoaim_mode == AUTOAIM) {
            // Create armor detector
            if (params_.armor.use_traditional) {
                detector_ =
                    std::move(detector_factory_->create_traditional_armor_detector(params_));
                RCLCPP_INFO(logger_, "Created traditional armor detector");
            } else {
                detector_ =
                    std::move(detector_factory_->create_ovnet_armor_energy_detector(params_));
                RCLCPP_INFO(logger_, "Created network armor detector");
            }
            // Create armor pnp solver
            pnp_solver_ = std::make_shared<ArmorProjectYaw>(cam_info_->k, cam_info_->d);
            pnp_solver_->set_use_projection(params_.use_projection);
            RCLCPP_INFO(logger_, "Created armor pnp solver");
        } else {
            // Create energy detector
            if (params_.energy.use_traditional) {
                detector_ =
                    std::move(detector_factory_->create_traditional_energy_detector(params_));
                RCLCPP_INFO(logger_, "Created traditional energy detector");
            } else {
                detector_ =
                    std::move(detector_factory_->create_ovnet_armor_energy_detector(params_));
                RCLCPP_INFO(logger_, "Created network energy detector");
            }
            // Create energy pnp solver
            pnp_solver_ = std::make_shared<EnergyProjectRoll>(cam_info_->k, cam_info_->d);
            pnp_solver_->set_use_projection(params_.use_projection);
            RCLCPP_INFO(logger_, "Created energy pnp solver");
        }

        // Set camera info for new detector
        if (cam_info_) {
            detector_->set_cam_info(*cam_info_);
        }

        // Log mode change if applicable
        if (params_.autoaim_mode != last_autoaim_mode_) {
            RCLCPP_WARN(
                logger_,
                "Mode changed from %s to %s",
                last_autoaim_mode_ == AUTOAIM ? "armor" : "energy",
                params_.autoaim_mode == AUTOAIM ? "armor" : "energy"
            );
        }
    } else {
        // Just update parameters if no switching needed
        update_detector_params();
    }

    // Update flags
    armor_use_traditional_ = params_.armor.use_traditional;
    energy_use_traditional_ = params_.energy.use_traditional;
    last_autoaim_mode_ = params_.autoaim_mode;

    RCLCPP_INFO(
        logger_,
        "Current state - Use traditional armor: %d, Use traditional energy: %d",
        armor_use_traditional_,
        energy_use_traditional_
    );
}

void DetectorNode::update_detector_params() {
    if (pnp_solver_) {
        pnp_solver_->set_use_projection(params_.use_projection);
    }
    if (params_.autoaim_mode == AUTOAIM) {
        if (params_.armor.use_traditional) {
            set_detector_params<TraditionalArmorParams>(params_, detector_);
        } else {
            set_detector_params<OVNetArmorEnergyDetectorParams>(params_, detector_);
        }
    } else {
        if (params_.energy.use_traditional) {
            set_detector_params<TraditionalEnergyParams>(params_, detector_);
        } else {
            set_detector_params<OVNetArmorEnergyDetectorParams>(params_, detector_);
        }
    }
}

void DetectorNode::publish_debug_infos() {
    try {
        // Check current mode and detector type
        if (params_.autoaim_mode == AUTOAIM) {
            // Armor mode
            if (params_.armor.use_traditional) {
                auto images = ArmorEnergyDetectStream::get_debug_images<TraditionalArmorDebugImage>(
                    detector_
                );
                if (params_.use_projection) {
                    pnp_solver_->draw_projection_points(images.result_img);
                }
                binary_img_pub_.publish(
                    cv_bridge::CvImage(
                        std_msgs::msg::Header(),
                        sensor_msgs::image_encodings::MONO8,
                        images.binary_img
                    )
                        .toImageMsg()
                );
                result_img_pub_.publish(
                    cv_bridge::CvImage(
                        std_msgs::msg::Header(),
                        sensor_msgs::image_encodings::RGB8,
                        images.result_img
                    )
                        .toImageMsg()
                );
                number_img_pub_.publish(
                    cv_bridge::CvImage(
                        std_msgs::msg::Header(),
                        sensor_msgs::image_encodings::MONO8,
                        images.number_img
                    )
                        .toImageMsg()
                );

                try {
                    auto debug_infos =
                        ArmorEnergyDetectStream::get_debug_infos<TraditionalArmorDebugInfo>(
                            detector_
                        );
                    armors_data_pub_->publish(debug_infos.debug_armors);
                    lights_data_pub_->publish(debug_infos.debug_lights);
                } catch (const std::exception& e) {
                    RCLCPP_WARN_THROTTLE(
                        logger_,
                        *this->get_clock(),
                        1000,
                        "Failed to publish debug info: %s",
                        e.what()
                    );
                }
            } else {
                // Network armor detector
                auto images =
                    ArmorEnergyDetectStream::get_debug_images<OVNetArmorEnergyDebugImages>(
                        detector_
                    );
                if (params_.use_projection) {
                    pnp_solver_->draw_projection_points(images.result_img);
                }
                result_img_pub_.publish(
                    cv_bridge::CvImage(
                        std_msgs::msg::Header(),
                        sensor_msgs::image_encodings::RGB8,
                        images.result_img
                    )
                        .toImageMsg()
                );
            }
        } else {
            // Energy mode
            if (params_.energy.use_traditional) {
                auto images =
                    ArmorEnergyDetectStream::get_debug_images<TraditionalEnergyDebugImages>(
                        detector_
                    );
                if (params_.use_projection) {
                    pnp_solver_->draw_projection_points(images.detect_img);
                }
                binary_img_pub_.publish(
                    cv_bridge::CvImage(
                        std_msgs::msg::Header(),
                        sensor_msgs::image_encodings::MONO8,
                        images.binary_img
                    )
                        .toImageMsg()
                );
                result_img_pub_.publish(
                    cv_bridge::CvImage(
                        std_msgs::msg::Header(),
                        sensor_msgs::image_encodings::RGB8,
                        images.detect_img
                    )
                        .toImageMsg()
                );
            } else {
                // Network energy detector
                auto images =
                    ArmorEnergyDetectStream::get_debug_images<OVNetArmorEnergyDebugImages>(
                        detector_
                    );
                if (params_.use_projection) {
                    pnp_solver_->draw_projection_points(images.result_img);
                }
                result_img_pub_.publish(
                    cv_bridge::CvImage(
                        std_msgs::msg::Header(),
                        sensor_msgs::image_encodings::RGB8,
                        images.result_img
                    )
                        .toImageMsg()
                );
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(
            logger_,
            *this->get_clock(),
            1000,
            "Error in publish_debug_infos: %s",
            e.what()
        );
    }
}

DetectorNode::~DetectorNode() {
    if (param_switch_thread.joinable()) {
        param_switch_thread.join();
    }
    if (serial_switch_thread.joinable()) {
        serial_switch_thread.join();
    }
    binary_img_pub_.shutdown();
    result_img_pub_.shutdown();
    detector_.reset();
    pnp_solver_.reset();
    detector_factory_.reset();
    param_listener_.reset();
    RCLCPP_INFO(logger_, "DetectorNode destructed");
}

} // namespace helios_cv

// register node to component
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::DetectorNode);
