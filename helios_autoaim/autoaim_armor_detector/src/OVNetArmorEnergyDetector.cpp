//
// Created by zyi on 24-9-18.
//
#include "autoaim_armor_detector/OVNetArmorEnergyDetector.hpp"
#include "autoaim_armor_detector/BaseDetector.hpp"
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <utility>
namespace helios_cv {

// const std::vector<std::string> ARMOR_NUMBER_LABEL{ "guard", "1", "2", "3", "4", "outpost", "base", "base" };
const std::vector<std::string> ARMOR_NUMBER_LABEL { "guard", "1",       "2",    "3",
                                                    "4",     "outpost", "base", "base" };
const std::vector<std::string> ENERGY_NUMBER_LABEL { /// TODO: add energy number label
                                                     "energy_target",
                                                     "energy_fan",
                                                     "energy_r"
};
YOLOXDetector::~YOLOXDetector() {
    stop_processing();
}

void YOLOXDetector::initialize(
    const std::string& model_path,
    const OVNetArmorEnergyDetectorParams& params
) {
    compiled_model = ov::CompiledModel();
    core = ov::Core();
    params_ = params;
    NUM_CLASS = params_.net_params.NUM_CLASS;

    try {
        core.set_property(ov::cache_dir("./cache"));

        std::filesystem::path path(model_path);
        std::string extension = path.extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

        if (extension == ".xml") {
            std::string bin_path = path.replace_extension(".bin").string();
            if (std::filesystem::exists(bin_path)) {
                model = core.read_model(model_path, bin_path);
            } else {
                throw std::runtime_error("Corresponding .bin file not found for XML model.");
            }
        } else if (extension == ".onnx") {
            model = core.read_model(model_path);
        } else {
            throw std::runtime_error("Unsupported model format. Please use .xml or .onnx");
        }

        auto input_shape = model->input().get_shape();
        if (input_shape.size() != 4) {
            throw std::runtime_error("Expected 4D input shape (NCHW)");
        }
        INPUT_H = input_shape[2];
        INPUT_W = input_shape[3];

        ov::preprocess::PrePostProcessor ppp(model);
        ppp.input()
            .tensor()
            .set_element_type(ov::element::u8)
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::BGR);
        ppp.input()
            .preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB);
        ppp.input().model().set_layout("NCHW");
        ppp.output().tensor().set_element_type(ov::element::f32);

        model = ppp.build();

        std::vector<std::string> available_devices = core.get_available_devices();
        std::string target_device = "CPU";

        if (std::find(available_devices.begin(), available_devices.end(), "GPU")
            != available_devices.end())
        {
            target_device = "GPU";
        }

        ov::AnyMap config;
        if (target_device == "GPU") {
            config = {
                ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
                ov::hint::model_priority(ov::hint::Priority::HIGH),
                ov::hint::execution_mode(ov::hint::ExecutionMode::PERFORMANCE),
                ov::num_streams(1),
                ov::cache_dir("./cache"),
                ov::cache_mode(ov::CacheMode::OPTIMIZE_SPEED),
                ov::hint::enable_cpu_pinning(true),
            };
        } else {
            config = {
                ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
                ov::hint::execution_mode(ov::hint::ExecutionMode::PERFORMANCE),
                ov::num_streams(1),
                ov::cache_dir("./cache"),
                ov::hint::inference_precision(ov::element::f32),
                ov::inference_num_threads(std::thread::hardware_concurrency()),
                ov::hint::enable_cpu_pinning(true),
                ov::hint::scheduling_core_type(ov::hint::SchedulingCoreType::ANY_CORE),
                ov::hint::enable_hyper_threading(true),
            };
        }

        compiled_model = core.compile_model(model, target_device, config);

        infer_request = compiled_model.create_infer_request();

        cv::Mat dummy(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(114, 114, 114));
        ov::Tensor dummy_tensor = ov::Tensor(
            ov::element::u8,
            { static_cast<size_t>(1),
              static_cast<size_t>(INPUT_H),
              static_cast<size_t>(INPUT_W),
              static_cast<size_t>(3) },
            dummy.data
        );
        infer_request.set_input_tensor(dummy_tensor);
        infer_request.infer();

        const int strides[3] = { 8, 16, 32 };
        grid_strides_.clear();
        generate_grids_and_stride(INPUT_W, INPUT_H, strides, grid_strides_);
        scale_x_ = static_cast<float>(INPUT_W) / 1440;
        scale_y_ = static_cast<float>(INPUT_H) / 1080;
        std::cout << "input size: " << INPUT_W << "x" << INPUT_H << std::endl;
        std::cout << "scale_x_: " << scale_x_ << ", scale_y_: " << scale_y_ << std::endl;
        debug_image = cv::Mat(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(0, 0, 0));

    } catch (const std::exception& e) {
        throw;
    }
}

void YOLOXDetector::draw_objects(cv::Mat& frame, const ArmorsStamped& armors_stamped) {
    for (const auto& armor: armors_stamped.armors) {
        if (armor.type != ArmorType::INVALID) {
            cv::line(
                frame,
                armor.left_light.top,
                armor.right_light.bottom,
                cv::Scalar(0, 255, 0),
                2
            );
            cv::line(
                frame,
                armor.left_light.bottom,
                armor.right_light.top,
                cv::Scalar(0, 255, 0),
                2
            );
            if (params_.autoaim_mode != 0) {
                cv::line(frame, armor.left_light.bottom, armor.center, cv::Scalar(0, 255, 0), 2);
                cv::line(frame, armor.right_light.bottom, armor.center, cv::Scalar(0, 255, 0), 2);
            }
        }
    }

    for (const auto& armor: armors_stamped.armors) {
        if (armor.type != ArmorType::INVALID) {
            cv::putText(
                frame,
                armor.number,
                armor.left_light.top,
                cv::FONT_HERSHEY_SIMPLEX,
                0.8,
                cv::Scalar(0, 255, 255),
                2
            );
            cv::putText(
                frame,
                std::to_string(armor.confidence),
                armor.right_light.top,
                cv::FONT_HERSHEY_SIMPLEX,
                0.8,
                cv::Scalar(0, 255, 255),
                2
            );
        }
    }

    for (const auto& armor: armors_stamped.armors) {
        if (armor.type != ArmorType::INVALID) {
            auto check_point = [&frame](const cv::Point2f& p) -> bool {
                return p.x >= 0 && p.x < frame.cols && p.y >= 0 && p.y < frame.rows
                    && !std::isnan(p.x) && !std::isnan(p.y);
            };

            if (check_point(armor.left_light.bottom)) {
                cv::circle(frame, armor.left_light.bottom, 10, cv::Scalar(0, 255, 0));
            }
            if (check_point(armor.left_light.top)) {
                cv::circle(frame, armor.left_light.top, 20, cv::Scalar(0, 255, 0));
            }
            if (check_point(armor.right_light.top)) {
                cv::circle(frame, armor.right_light.top, 30, cv::Scalar(0, 255, 0));
            }
            if (check_point(armor.right_light.bottom)) {
                cv::circle(frame, armor.right_light.bottom, 40, cv::Scalar(0, 255, 0));
            }
        }
    }
}
ArmorType YOLOXDetector::judge_armor_type(const Armor& armor) {
    cv::Point2f light_center1 = (armor.left_light.top + armor.left_light.bottom) / 2.0;
    cv::Point2f light_center2 = (armor.right_light.top + armor.right_light.bottom) / 2.0;
    float light_length1 = cv::norm(armor.left_light.top - armor.left_light.bottom);
    float light_length2 = cv::norm(armor.right_light.top - armor.right_light.bottom);

    float avg_light_length = (light_length1 + light_length2) / 2.0;
    float center_distance = cv::norm(light_center1 - light_center2) / avg_light_length;

    return center_distance > params_.min_large_center_distance ? ArmorType::LARGE
                                                               : ArmorType::SMALL;
}
std::vector<Armor> YOLOXDetector::sync_detect_with_timestamp(const cv::Mat& frame) {
    cv::Mat resized_frame;
    float scale = std::min(
        static_cast<float>(INPUT_W) / frame.cols,
        static_cast<float>(INPUT_H) / frame.rows
    );
    int unpad_w = static_cast<int>(scale * frame.cols);
    int unpad_h = static_cast<int>(scale * frame.rows);
    // std::cout << "unpad_w: " << unpad_w << ", unpad_h: " << unpad_h << std::endl;
    cv::resize(frame, resized_frame, cv::Size(unpad_w, unpad_h));
    cv::Mat input_frame(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(114, 114, 114));
    resized_frame.copyTo(input_frame(cv::Rect(0, 0, unpad_w, unpad_h)));

    ov::Tensor input_tensor = ov::Tensor(
        ov::element::u8,
        { 1, static_cast<size_t>(INPUT_H), static_cast<size_t>(INPUT_W), 3 },
        input_frame.data
    );
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    const ov::Tensor& output_tensor = infer_request.get_output_tensor();

    // 处理结果
    std::vector<Object> objects = process(output_tensor);

    // 转换为Armor对象
    std::vector<Armor> armors;
    for (const auto& object: objects) {
        // 只处理指定颜色的装甲板
        // if (object.color != params_.is_blue) {
        //     continue;
        // }
        if (object.conf < CLASSIFIER_THRESHOLD) {
            continue;
        }

        Armor armor_target;
        armor_target.confidence = object.conf;

        auto map_to_original = [scale, unpad_w, unpad_h](cv::Point2f point) -> cv::Point2f {
            return cv::Point2f(point.x / scale, point.y / scale);
        };

        if (params_.autoaim_mode != 0) {
            armor_target.number = ENERGY_NUMBER_LABEL[object.label];
            // 应用坐标映射
            armor_target.left_light.bottom = map_to_original(object.apexes[3]);
            armor_target.left_light.top = map_to_original(object.apexes[0]);
            armor_target.right_light.top = map_to_original(object.apexes[1]);
            armor_target.right_light.bottom = map_to_original(object.apexes[2]);
            armor_target.center = map_to_original(cv::Point2f(0, 0));
            armor_target.type =
                object.label == 0 ? ArmorType::ENERGY_TARGET : ArmorType::ENERGY_FAN;
        } else {
            armor_target.number = ARMOR_NUMBER_LABEL[object.label];
            // 应用坐标映射
            armor_target.left_light.bottom = map_to_original(object.apexes[1]);
            armor_target.left_light.top = map_to_original(object.apexes[0]);
            armor_target.right_light.top = map_to_original(object.apexes[3]);
            armor_target.right_light.bottom = map_to_original(object.apexes[2]);

            // 计算中心点(使用映射后的坐标)
            armor_target.center = (armor_target.left_light.top + armor_target.left_light.bottom
                                   + armor_target.right_light.top + armor_target.right_light.bottom)
                * 0.25f;

            armor_target.type = judge_armor_type(armor_target);
        }

        armors.push_back(armor_target);
    }

    if (params_.debug && !frame.empty()) {
        cv::Mat debug_frame = frame.clone();
        ArmorsStamped armor_stamped;
        armor_stamped.armors = armors;
        draw_objects(debug_frame, armor_stamped);

        std::stringstream ss_latency;
        cv::putText(
            debug_frame,
            ss_latency.str(),
            cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            cv::Scalar(0, 255, 0),
            2
        );

        std::lock_guard<std::mutex> lock(debug_image_mutex);
        debug_image = debug_frame;
    }

    return armors;
}

std::vector<YOLOXDetector::Object> YOLOXDetector::process(const ov::Tensor& output_tensor) {
    const float* output_buffer = output_tensor.data<const float>();
    std::vector<Object> objects;
    generate_yolox_proposal(grid_strides_, output_buffer, objects);

    std::sort(objects.begin(), objects.end(), [](const Object& a, const Object& b) {
        return a.conf > b.conf;
    });

    std::vector<int> picked;
    nms_sorted_bboxes(objects, picked);

    std::vector<Object> result;
    result.reserve(picked.size());
    for (int i: picked) {
        result.push_back(objects[i]);
    }

    avg_rect(result);
    return result;
}

void YOLOXDetector::generate_yolox_proposal(
    const std::vector<GridAndStride>& grid_strides,
    const float* output_buffer,
    std::vector<Object>& objects
) {
    const int num_anchors = grid_strides.size();
    const int class_start = 2 * NUM_APEX + 1;

    for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++) {
        const int basic_pos = anchor_idx * (class_start + NUM_CLASS + NUM_COLORS);
        float box_conf = output_buffer[basic_pos + 2 * NUM_APEX];

        if (box_conf >= 0.8) {
            const GridAndStride& gs = grid_strides[anchor_idx];
            std::vector<cv::Point2f> point;
            for (int i = 0; i < NUM_APEX; i++) {
                float x = (output_buffer[basic_pos + 0 + i * 2] + gs.grid0) * gs.stride;
                float y = (output_buffer[basic_pos + 1 + i * 2] + gs.grid1) * gs.stride;
                point.emplace_back(x, y);
            }

            int color_idx = argmax(output_buffer + basic_pos + class_start, NUM_COLORS);
            int class_idx = argmax(output_buffer + basic_pos + class_start + NUM_COLORS, NUM_CLASS);
            float color_conf = output_buffer[basic_pos + class_start + color_idx];
            float class_conf = output_buffer[basic_pos + class_start + NUM_COLORS + class_idx];

            Object obj;
            obj.apexes = std::move(point);
            obj.rect = cv::boundingRect(obj.apexes);
            obj.label = class_idx;
            obj.color = color_idx;
            obj.conf = box_conf * ((class_conf + color_conf) / 2);

            objects.push_back(obj);
        }
    }
}
void YOLOXDetector::stop_processing() {
    // Lock to prevent concurrent access while stopping
    std::lock_guard<std::mutex> lock(params_mutex_);

    try {
        if (infer_request) {
            infer_request.cancel();
        }

        compiled_model = ov::CompiledModel();
        {
            std::lock_guard<std::mutex> debug_lock(debug_image_mutex);
            debug_image = cv::Mat();
        }
        RCLCPP_INFO(rclcpp::get_logger("YOLOXDetector"), "Processing stopped successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger("YOLOXDetector"),
            "Error stopping processing: %s",
            e.what()
        );
    }
}

void YOLOXDetector::update_params(const OVNetArmorEnergyDetectorParams& new_params) {
    std::lock_guard<std::mutex> lock(params_mutex_);
    bool significant_change = (new_params.net_params.NUM_CLASS != params_.net_params.NUM_CLASS)
        || (new_params.net_params.MODEL_PATH != params_.net_params.MODEL_PATH);

    if (significant_change) {
        RCLCPP_INFO(
            rclcpp::get_logger("YOLOXDetector"),
            "Significant parameter change detected, reinitializing detector"
        );
        stop_processing();
        initialize(new_params.net_params.MODEL_PATH, new_params);
        start_processing();
    } else {
        RCLCPP_INFO(
            rclcpp::get_logger("YOLOXDetector"),
            "Updating parameters without reinitialization"
        );
        params_ = new_params;
    }
}

void YOLOXDetector::qsort_descent_inplace(std::vector<Object>& objects) {
    std::sort(objects.begin(), objects.end(), [](const Object& a, const Object& b) {
        return a.conf > b.conf;
    });
}

void YOLOXDetector::nms_sorted_bboxes(std::vector<Object>& faceobjects, std::vector<int>& picked) {
    picked.clear();
    const int n = faceobjects.size();

    for (int i = 0; i < n; i++) {
        const Object& a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++) {
            const Object& b = faceobjects[picked[j]];

            float inter_area = intersection_area(a, b);
            float union_area = a.rect.area() + b.rect.area() - inter_area;
            float iou = inter_area / union_area;

            if (iou > NMS_THRESH || std::isnan(iou)) {
                keep = 0;
                if (iou > 0.9 && std::abs(a.conf - b.conf) < 0.2 && a.label == b.label
                    && a.color == b.color)
                {
                    faceobjects[picked[j]].apexes.insert(
                        faceobjects[picked[j]].apexes.end(),
                        a.apexes.begin(),
                        a.apexes.end()
                    );
                }
                break;
            }
        }

        if (keep) {
            picked.push_back(i);
        }
    }
}

void YOLOXDetector::avg_rect(std::vector<Object>& objects) {
    for (auto& object: objects) {
        std::size_t N = object.apexes.size();

        if (N >= 2 * NUM_APEX) {
            std::vector<cv::Point2f> fin_point(NUM_APEX, cv::Point2f(0, 0));

            for (size_t i = 0; i < N; i++) {
                fin_point[i % NUM_APEX] += object.apexes[i];
            }

            for (int i = 0; i < NUM_APEX; i++) {
                fin_point[i] /= static_cast<float>(N / NUM_APEX);
            }

            object.apexes = fin_point;
        }
    }
}

int YOLOXDetector::argmax(const float* ptr, int len) {
    return std::max_element(ptr, ptr + len) - ptr;
}

float YOLOXDetector::intersection_area(const Object& a, const Object& b) {
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

void YOLOXDetector::generate_grids_and_stride(
    const int w,
    const int h,
    const int strides[],
    std::vector<GridAndStride>& grid_strides
) {
    for (int i = 0; i < 3; i++) {
        int num_grid_w = w / strides[i];
        int num_grid_h = h / strides[i];

        for (int g1 = 0; g1 < num_grid_h; g1++) {
            for (int g0 = 0; g0 < num_grid_w; g0++) {
                grid_strides.emplace_back(GridAndStride { g0, g1, strides[i] });
            }
        }
    }
}

OVNetArmorEnergyDetector::OVNetArmorEnergyDetector(const OVNetArmorEnergyDetectorParams& params) {
    params_ = params;
    detector_ = std::make_unique<YOLOXDetector>(params_.net_params.MODEL_PATH, params_);
}

OVNetArmorEnergyDetector::~OVNetArmorEnergyDetector() {
    detector_->stop_processing();
}

std::vector<Armor> OVNetArmorEnergyDetector::detect_armors(cv::Mat image) {
    auto armors = detector_->sync_detect_with_timestamp(image);

    return std::move(armors);
}

void OVNetArmorEnergyDetector::set_params(void* params) {
    auto new_params = *static_cast<OVNetArmorEnergyDetectorParams*>(params);

    if (first_call_) {
        params_ = new_params;
        first_call_ = false;
        return;
    }

    bool params_changed = (new_params.autoaim_mode != params_.autoaim_mode)
        || (new_params.net_params.MODEL_PATH != params_.net_params.MODEL_PATH
            || new_params.is_blue != params_.is_blue);

    if (params_changed) {
        try {
            detector_ =
                std::make_unique<YOLOXDetector>(new_params.net_params.MODEL_PATH, new_params);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(
                rclcpp::get_logger("OVNetArmorEnergyDetector"),
                "Failed to update detector parameters: %s",
                e.what()
            );
            return;
        }
    }

    params_ = new_params;
}

cv::Mat OVNetArmorEnergyDetector::get_debug_image() {
    return detector_->get_debug_image();
}

} // namespace helios_cv
