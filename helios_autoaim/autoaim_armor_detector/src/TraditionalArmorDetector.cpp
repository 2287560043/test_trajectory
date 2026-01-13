#include "autoaim_armor_detector/TraditionalArmorDetector.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoaim_utilities/NumberClassifier.hpp>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <utility>
#include <vector>

namespace helios_cv {

TraditionalArmorDetector::TraditionalArmorDetector(const detector_node::Params& params) {
    RCLCPP_INFO(logger_,"TraditionalArmorDetector Constructed");
    params_ = params;
    auto model_path =
        ament_index_cpp::get_package_share_directory("autoaim_armor_detector") + "/model/mlp.onnx";
    auto label_path =
        ament_index_cpp::get_package_share_directory("autoaim_armor_detector") + "/model/label.txt";
    number_classifier_ = std::make_shared<NumberClassifier>(
        model_path,
        label_path,
        params_.traditional.number_classifier_threshold,
        std::vector<std::string> { "negative" }
    );
}

std::vector<Armor> TraditionalArmorDetector::detect_armors(cv::Mat image) {
    // preprocess
    debug_images_.binary_img = preprocessImage(image);
    lights_ = findLights(image, debug_images_.binary_img);
    auto armors = matchLights(lights_);
    if (!armors.empty()) {
        number_classifier_->extractNumbers(image, armors);
        number_classifier_->classify(armors);
    }
    // debug infos
    if (params_.debug) {
        draw_results(armors);
    }
    return std::move(armors);
}

void TraditionalArmorDetector::updateParams(const detector_node::Params& params) {
    params_ = params;
}

cv::Mat TraditionalArmorDetector::get_debug_image() {
    return debug_images_.result_img;
}

cv::Mat TraditionalArmorDetector::preprocessImage(const cv::Mat& input) {
    cv::Mat gray_img;
    cv::cvtColor(input, gray_img, cv::COLOR_RGB2GRAY);

    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, params_.traditional.binary_thres, 255, cv::THRESH_BINARY);

    debug_images_.result_img = input.clone();
    return binary_img;
}

std::vector<Light>
TraditionalArmorDetector::findLights(const cv::Mat& rbg_img, const cv::Mat& binary_img) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Light> lights;
    debug_infos_.debug_lights.data.clear();

    for (const auto& contour: contours) {
        if (contour.size() < 5) {
            continue;
        }
        auto r_rect = cv::minAreaRect(contour);
        auto light = Light(r_rect);

        if (isLight(light)) {
            auto rect = light.boundingRect();
            if (  // Avoid assertion failed
          0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y && 0 <= rect.height &&
          rect.y + rect.height <= rbg_img.rows)
      {
                int sum_r = 0, sum_b = 0;
                auto roi = rbg_img(rect);
                // Iterate through the ROI
                for (int i = 0; i < roi.rows; i++) {
                    for (int j = 0; j < roi.cols; j++) {
                        if (cv::pointPolygonTest(
                                contour,
                                cv::Point2f(j + rect.x, i + rect.y),
                                false
                            )
                            >= 0)
                        {
                            // if point is inside contour
                            sum_r += roi.at<cv::Vec3b>(i, j)[0];
                            sum_b += roi.at<cv::Vec3b>(i, j)[2];
                        }
                    }
                }
                // Sum of red pixels > sum of blue pixels ?
                light.color = sum_r > sum_b ? RED : BLUE;
                lights.emplace_back(light);
            }
        }
    }
    return lights;
}

std::vector<Armor> TraditionalArmorDetector::matchLights(const std::vector<Light>& lights) {
    std::vector<Armor> armors;

    debug_infos_.debug_armors.data.clear();

    // Loop all the pairing of lights
    for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
        for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
            // if (light_1->color == params_.is_blue || light_2->color == params_.is_blue)
            // {
            //   continue;
            // }
            if (containLight(*light_1, *light_2, lights)) {
                continue;
            }

            auto type = isArmor(*light_1, *light_2);
            if (type != ArmorType::INVALID) {
                auto armor = Armor(*light_1, *light_2);
                armor.type = type;
                armors.emplace_back(armor);
            }
        }
    }
    return armors;
}

bool TraditionalArmorDetector::isLight(const Light& possible_light) {
    // The ratio of light (short side / long side)
    float ratio = possible_light.width / possible_light.length;
    bool ratio_ok =
        params_.traditional.light.min_ratio < ratio && ratio < params_.traditional.light.max_ratio;

    bool angle_ok = possible_light.tilt_angle < params_.traditional.light.max_angle;

    bool is_light = ratio_ok && angle_ok;

    // Fill debug lights info
    autoaim_interfaces::msg::DebugLight debug_light;
    debug_light.center_x = possible_light.center.x;
    debug_light.ratio = ratio;
    debug_light.angle = possible_light.tilt_angle;
    debug_light.is_light = is_light;
    debug_infos_.debug_lights.data.emplace_back(debug_light);
    // if (is_light == 0)
    //   RCLCPP_INFO(rclcpp::get_logger("light_info"), "%f,%f",possible_light.width/possible_light.length,possible_light.tilt_angle);

    return is_light;
}

bool TraditionalArmorDetector::containLight(
    const Light& light_1,
    const Light& light_2,
    const std::vector<Light>& lights
) {
    auto points =
        std::vector<cv::Point2f> { light_1.top, light_1.bottom, light_2.top, light_2.bottom };
    auto bounding_rect = cv::boundingRect(points);

    for (const auto& test_light: lights) {
        if (test_light.center == light_1.center || test_light.center == light_2.center) {
            continue;
        }
        if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom)
            || bounding_rect.contains(test_light.center))
        {
            return true;
        }
    }

    return false;
}

ArmorType TraditionalArmorDetector::isArmor(const Light& light_1, const Light& light_2) {
    // Ratio of the length of 2 lights (short side / long side)
    float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                               : light_2.length / light_1.length;
    bool light_ratio_ok = light_length_ratio > params_.traditional.armor.min_light_ratio;

    // Distance between the center of 2 lights (unit : light length)
    float avg_light_length = (light_1.length + light_2.length) / 2;
    float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
    bool center_distance_ok =
        (params_.traditional.armor.min_small_center_distance <= center_distance
         && center_distance < params_.traditional.armor.max_small_center_distance)
        || (params_.traditional.armor.min_large_center_distance <= center_distance
            && center_distance < params_.traditional.armor.max_large_center_distance);
    // Angle of light center connection
    cv::Point2f diff = light_1.center - light_2.center;
    float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
    bool angle_ok = angle < params_.traditional.armor.max_angle;
    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

    // Judge armor type
    ArmorType type;
    if (is_armor) {
        // RCLCPP_WARN(logger_, "center_distance : %f", center_distance);
        type = center_distance > params_.traditional.armor.min_large_center_distance
            ? ArmorType::LARGE
            : ArmorType::SMALL;
    } else {
        type = ArmorType::INVALID;
    }
    // if (!angle_ok && params_.debug) {
    //     RCLCPP_WARN(logger_, "now angle %f, thresh %f", angle, params_.armor_params.max_angle);
    // }
    // if (!center_distance_ok && params_.debug) {
    //     RCLCPP_WARN(logger_, "now distance %f, thresh %f", center_distance,
    //     params_.armor_params.max_large_center_distance);
    // }
    // if (!light_length_ratio && params_.debug) {
    //     RCLCPP_WARN(logger_, "now ratio %f, thresh %f", light_length_ratio, params_.armor_params.min_light_ratio);
    // }
    // Fill debug armors info
    autoaim_interfaces::msg::DebugArmor debug_armor;
    debug_armor.center_x = (light_1.center.x + light_2.center.x) / 2;
    debug_armor.light_ratio = light_length_ratio;
    debug_armor.center_distance = center_distance;
    debug_armor.angle = angle;
    debug_armor.type = ARMOR_TYPE_STR[static_cast<int>(type)];
    debug_infos_.debug_armors.data.emplace_back(debug_armor);

    return type;
}

void TraditionalArmorDetector::draw_results(const std::vector<Armor>& armors) {
    // Draw Lights
    for (const auto& light: lights_) {
        cv::circle(debug_images_.result_img, light.top, 3, cv::Scalar(255, 255, 255), 1);
        cv::circle(debug_images_.result_img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
        auto line_color = light.color == RED ? cv::Scalar(100, 79, 70) : cv::Scalar(255, 0, 255);
        cv::line(debug_images_.result_img, light.top, light.bottom, line_color, 3);
    }

    // Draw armors
    for (const auto& armor: armors) {
        if (armor.type != ArmorType::INVALID) {
            cv::line(
                debug_images_.result_img,
                armor.left_light.top,
                armor.right_light.bottom,
                cv::Scalar(0, 255, 0),
                2
            );
            cv::line(
                debug_images_.result_img,
                armor.left_light.bottom,
                armor.right_light.top,
                cv::Scalar(0, 255, 0),
                2
            );
        }
    }

    // Show numbers and confidence
    for (const auto& armor: armors) {
        if (armor.type != ArmorType::INVALID) {
            cv::putText(
                debug_images_.result_img,
                armor.classfication_result,
                armor.left_light.top,
                cv::FONT_HERSHEY_SIMPLEX,
                0.8,
                cv::Scalar(0, 255, 255),
                2
            );
        }
    }
    // Draw latency
    std::stringstream latency_ss;
    get_all_number_images(armors);
}

void TraditionalArmorDetector::get_all_number_images(const std::vector<Armor>& armors) {
    // Get all number imgs
    std::vector<cv::Mat> all_number_imgs;
    if (armors.empty()) {
        debug_images_.number_img = cv::Mat(cv::Size(20, 28), CV_8UC1);
    } else {
        all_number_imgs.reserve(armors.size());
        for (auto armor: armors) {
            all_number_imgs.emplace_back(armor.number_img);
        }
        cv::vconcat(all_number_imgs, debug_images_.number_img);
    }
}

} // namespace helios_cv
