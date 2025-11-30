// created by lijunqi, liuhan on 2024/2/1
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */
 #pragma once

 #include "BaseDetector.hpp"
 
 #include <autoaim_utilities/ThreadPool.hpp>
 #include <memory>
 #include <opencv2/core.hpp>
 #include <opencv2/core/mat.hpp>
 #include <opencv2/highgui.hpp>
 #include <opencv2/imgproc.hpp>
 #include <rclcpp/rclcpp.hpp>
 #include <rclcpp/time.hpp>
 #include <sensor_msgs/msg/camera_info.hpp>
 #include <string>
 #include <utility>
 #include <vector>
 
 // custum utilities
 #include "autoaim_utilities/Armor.hpp"
 #include "autoaim_utilities/ThreadPool.hpp"
 
 
 // openvino
 #include <openvino/core/core.hpp>
 #include <openvino/openvino.hpp>
 #include <openvino/runtime/infer_request.hpp>
 
 #include "rclcpp/rclcpp.hpp"
 
 namespace helios_cv
 {
 
 struct OVNetArmorEnergyDetectorParams : public BaseParams
 {
   double classifier_threshold;
   double min_large_center_distance;
   typedef struct OVNetParams
   {
     std::string MODEL_PATH;
     int NUM_CLASS;     //类别总数 9
     int NUM_COLORS;    //颜色 2
     float NMS_THRESH;  // NMS阈值 0.2
     int NUM_APEX;      // 4
     int POOL_NUM;      // Thread number 3
   } OVNetParams;
   OVNetParams net_params;
 };
 
 struct ArmorsStamped : public Armors
 {
   rclcpp::Time stamp;
 };
 
 struct ImageStamped : public Image
 {
   ImageStamped(cv::Mat&& image, rclcpp::Time stamp) : Image(std::forward<cv::Mat>(image)), stamp(stamp)
   {
   }
 
   ImageStamped() = default;
 
   rclcpp::Time stamp;
 };
 
 struct OVNetArmorEnergyDebugImages : public DebugImages
 {
   cv::Mat result_img;
 };
 
 class YOLOXDetector
 {
 public:
   
   YOLOXDetector(const std::string& model_path, const OVNetArmorEnergyDetectorParams& params)
   {
     initialize(model_path, params);
   }
   
 
   ~YOLOXDetector();
   
 
   void stop_processing();
   
   
   void start_processing() {} 
   
 
   std::pair<std::vector<Armor>, rclcpp::Time> sync_detect_with_timestamp(const cv::Mat& frame, const rclcpp::Time& timestamp);  
 
   cv::Mat get_debug_image()
   {
     std::lock_guard<std::mutex> lock(debug_image_mutex);
     if (!debug_image.empty())
     {
       return debug_image.clone();
     }
     return cv::Mat();
   }
   
   void add_frame(const ImageStamped& image_stamped) {} 
   std::optional<ArmorsStamped> get_processed_armors() { return std::nullopt; }
 
 private:
 
   struct GridAndStride
   {
     int grid0;
     int grid1;
     int stride;
     GridAndStride(int g0, int g1, int s) : grid0(g0), grid1(g1), stride(s) {}
   };
 
   struct Object
   {
     std::vector<cv::Point2f> apexes;
     cv::Rect_<float> rect;
     int label;
     int color;
     float conf;
   };
 
 
   void initialize(const std::string& model_path, const OVNetArmorEnergyDetectorParams& params);
   
   
   std::vector<Object> process(const ov::Tensor& output_tensor);
   
 
   void generate_yolox_proposal(const std::vector<GridAndStride>& grid_strides, const float* output_buffer,
                               std::vector<Object>& objects);
   
 
   void nms_sorted_bboxes(std::vector<Object>& faceobjects, std::vector<int>& picked);
   
 
   void qsort_descent_inplace(std::vector<Object>& objects);
   void avg_rect(std::vector<Object>& objects);
   int argmax(const float* ptr, int len);
   float intersection_area(const Object& a, const Object& b);
   void generate_grids_and_stride(const int w, const int h, const int strides[],
                                 std::vector<GridAndStride>& grid_strides);
   
   void update_params(const OVNetArmorEnergyDetectorParams& new_params);
   ArmorType judge_armor_type(const Armor& armor);
   
   
   void draw_objects(cv::Mat& frame, const ArmorsStamped& armors_stamped);
   
 
   ov::Core core;
   std::shared_ptr<ov::Model> model;
   ov::CompiledModel compiled_model;
   ov::InferRequest infer_request;  
 
   int INPUT_H = 1080;
   int INPUT_W = 1440;
   int NUM_CLASS;
   float scale_x_;
   float scale_y_;
   
 
   OVNetArmorEnergyDetectorParams params_;
 
   cv::Mat debug_image;
   std::mutex debug_image_mutex;
   
 
   std::vector<GridAndStride> grid_strides_;
   
 
   constexpr static float CLASSIFIER_THRESHOLD = 0.8f;
   constexpr static float NMS_THRESH = 0.8f;
   constexpr static int NUM_APEX = 4;
   constexpr static int NUM_COLORS = 2;
   
   std::mutex reinit_mutex_;
   std::mutex params_mutex_;
   bool should_stop = false;
 
   double current_latency_ms_ = 0.0;
 };
 class OVNetArmorEnergyDetector : public BaseDetector
 {
 public:
   explicit OVNetArmorEnergyDetector(const OVNetArmorEnergyDetectorParams& params);
 
   ~OVNetArmorEnergyDetector();
 
   std::shared_ptr<Armors> detect_armors(std::shared_ptr<Image> image) final;
 
   // std::vector<ArmorStamped> detect_armors(const ImageStamped& image_stamped) final;
 
   void set_params(void* params) final;
 
   DebugImages* get_debug_images() final;
 
   DebugInfos* get_debug_infos() final;
 
 
 private:
   bool first_call_ = true;
   void clear_queue(std::queue<std::future<ArmorsStamped>>& futs);
 
   int frames_;
 
   std::unique_ptr<YOLOXDetector> detector_;
 
   std::queue<std::future<ArmorsStamped>> futs_;
 
 
   std::vector<ImageStamped> img_stampeds_;
 
   OVNetArmorEnergyDebugImages debug_images_;
   DebugInfos debug_infos_;
 
   OVNetArmorEnergyDetectorParams params_;
 
   rclcpp::Logger logger_ = rclcpp::get_logger("OVNetArmorEnergyDetector");
 
   std::mutex detector_mutex_;
 };
 
 }  // namespace helios_cv