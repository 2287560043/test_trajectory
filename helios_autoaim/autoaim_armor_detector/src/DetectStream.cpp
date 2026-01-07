// created by liuhan on 2024/4/6
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */

#include "autoaim_armor_detector/DetectStream.hpp"
#include "autoaim_armor_detector/OVNetArmorEnergyDetector.hpp"
#include <autoaim_utilities/PnPSolver.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace helios_cv {

autoaim_interfaces::msg::Armors
ArmorDetectStream::detect(std::shared_ptr<Image> image, void* extra_param) {
    autoaim_interfaces::msg::Armors armors_msg;

    if (extra_param) {
        pnp_solver_->update_transform_info(reinterpret_cast<BaseTransformInfo*>(extra_param));
    }

    auto armors = detector_->detect_armors(image);
    auto armors_stamped = dynamic_cast<ArmorsStamped*>(armors.get());

    if (armors_stamped) {
        armors_msg.header.stamp = armors_stamped->stamp;
    }

    if (!armors_stamped || armors_stamped->armors.empty()) {
      return armors_msg;
    }
    autoaim_interfaces::msg::Armor temp_armor;
    for (const auto& armor: armors->armors) {
        cv::Mat tvec, rvec;
        if (armor.type == ArmorType::ENERGY_FAN) {
            pnp_solver_->solve_pose(armor, rvec, tvec);
            continue;
        }
        if (pnp_solver_->solve_pose(armor, rvec, tvec)) {
            temp_armor.type = static_cast<int>(armor.type);
            temp_armor.number = armor.number;

            temp_armor.pose.position.x = tvec.at<double>(0);
            temp_armor.pose.position.y = tvec.at<double>(1);
            temp_armor.pose.position.z = tvec.at<double>(2);

            cv::Mat rotation_matrix;
            if (pnp_solver_->use_projection()) {
                rotation_matrix = rvec;
            } else {
                cv::Rodrigues(rvec, rotation_matrix);
            }
            tf2::Matrix3x3 tf2_rotation_matrix(
                rotation_matrix.at<double>(0, 0),
                rotation_matrix.at<double>(0, 1),
                rotation_matrix.at<double>(0, 2),
                rotation_matrix.at<double>(1, 0),
                rotation_matrix.at<double>(1, 1),
                rotation_matrix.at<double>(1, 2),
                rotation_matrix.at<double>(2, 0),
                rotation_matrix.at<double>(2, 1),
                rotation_matrix.at<double>(2, 2)
            );
            tf2::Quaternion tf2_quaternion;
            tf2_rotation_matrix.getRotation(tf2_quaternion);
            temp_armor.pose.orientation = tf2::toMsg(tf2_quaternion);
            temp_armor.distance_to_image_center =
                pnp_solver_->calculateDistanceToCenter(armor.center);

            armors_msg.armors.push_back(temp_armor);
        }
    }

    return armors_msg;
}

} // namespace helios_cv
