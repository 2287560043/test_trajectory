#include "TraditionalEnergyDetector.hpp"
#include "OVNetArmorEnergyDetector.hpp"

#include <autoaim_utilities/Armor.hpp>
#include <cmath>
#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <utility>
#include "BaseDetector.hpp"
#include "TraditionalArmorDetector.hpp"

namespace helios_cv
{

TraditionalEnergyDetector::TraditionalEnergyDetector(const TraditionalEnergyParams& params)
{
  params_ = params;
  contours.clear();
  Arrows_.clear();
  R_.clear();
  Fans_.clear();
  target_pts_.clear();
  target_pts_.resize(4);
}

std::shared_ptr<Armors> TraditionalEnergyDetector::detect_armors(std::shared_ptr<Image> image)
{
  auto armors_stamped = std::make_shared<ArmorsStamped>();
  debug_images_.detect_img = image->image.clone();
  cv::Mat img_roi;
  // binary threshold
  raw_width_ = debug_images_.detect_img.cols;
  raw_height_ = debug_images_.detect_img.rows;
  refresh_global_Roi();
  debug_images_.binary_img = preprocess(debug_images_.detect_img, params_.is_blue);
  // find target circles, arrows, fans
  find_target_flow(debug_images_.binary_img, contours);
  // return energy armor
  if (find_target_)
  {
    getPts();
    get_Fan_Pts();

    // 一定要fan先发点
    for(int i = 0; i < fan_pts_.size() / 2; i++){
      Armor armor;
      armor.left_light.bottom = fan_pts_[2*i];
      armor.left_light.top = fan_pts_[2*i + 1];
      armor.right_light.top = cv::Point2f(0, 0);
      armor.right_light.bottom = cv::Point2f(0, 0);

      armor.center = circle_center_point;
      armor.type = ArmorType::ENERGY_FAN;
      armor.classfication_result = "energy_fan";
      armors_stamped->armors.emplace_back(armor);
    }
    Armor armor;
    armor.left_light.bottom = target_pts_[0];
    armor.left_light.top = target_pts_[1];
    armor.right_light.top = target_pts_[2];
    armor.right_light.bottom = target_pts_[3];

    armor.center = circle_center_point;
    armor.type = ArmorType::ENERGY_TARGET;
    armor.classfication_result = "energy_target";
    armors_stamped->armors.emplace_back(armor);
  }
  // draw debug infos
  if (params_.debug)
  {
    draw_results(debug_images_.detect_img);
  }
  return std::move(armors_stamped);
}

void TraditionalEnergyDetector::set_params(void* params)
{
  params_ = *static_cast<TraditionalEnergyParams*>(params);
}

DebugImages* TraditionalEnergyDetector::get_debug_images()
{
  return &debug_images_;
}

DebugInfos* TraditionalEnergyDetector::get_debug_infos()
{
  return &debug_infos_;
}

// 彩色图二值化
cv::Mat TraditionalEnergyDetector::preprocess(cv::Mat& src, bool isred)
{
  // cv::Mat src_roi = src(global_roi_);
  cv::Mat src_roi = src;

   //颜色掩膜处理（提取红/蓝，排除白色）
  std::vector<cv::Mat> channels;
  cv::split(src_roi, channels);
  cv::Mat blue = channels[0], green = channels[1], red = channels[2];
  cv::Mat colorMask;
  cv::Mat whiteMask = (blue > params_.white_mask_thres) & (green > params_.white_mask_thres) & (red > params_.white_mask_thres);
  if (params_.is_blue) {
    // 蓝符(is_blue = 1)
    colorMask = (blue > 120); 
  } else {
    // 红符(is_blue = 0)
    colorMask = (red > 110); 
  }
  // 合并掩膜并排除白色
  colorMask = ~(colorMask | whiteMask);
  // std::cout << "Non-zero pixels in colorMask: " << cv::countNonZero(colorMask) << std::endl;

  cv::Mat masked_roi;
  src_roi.copyTo(masked_roi, colorMask);
  //考虑下是否保留
  debug_images_.detect_img = masked_roi;
  //
  masked_roi=masked_roi(global_roi_);
  std::vector<cv::Mat> mask_channels;
  cv::split(masked_roi, mask_channels); 
  cv::Mat mask_blue = mask_channels[0], mask_green = mask_channels[1], mask_red = mask_channels[2];
  
  //通道加权合成灰度图（直接对掩膜后区域操作）
  cv::Mat img_gry;
  if (params_.is_blue) {
    // 蓝符(is_blue = 1)
    img_gry = params_.rgb_weight_b_1 * mask_blue + 
    params_.rgb_weight_b_2 * mask_green + 
    params_.rgb_weight_b_3 * mask_red;
  } else {
    // 红符(is_blue = 0)
    img_gry = params_.rgb_weight_r_1 * mask_blue + 
    params_.rgb_weight_r_2 * mask_green + 
    params_.rgb_weight_r_3 * mask_red;
  }
  cv::Mat img_th;
  //THRESH_OTSU：忽略传入参数，自动调整阈值
  cv::threshold(img_gry, img_th, params_.binary_thresh, 255, cv::THRESH_BINARY);
  
  // 形态闭运算，膨胀+腐蚀
  cv::Mat kernel_close = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(img_th, img_th, cv::MORPH_CLOSE, kernel_close);
  // 高斯模糊（卷积核越大越模糊，但不要大到让特征边界连在一起）
  cv::GaussianBlur(img_th, img_th, cv::Size(3, 3), 0);
  return img_th;
}

// 寻找目标
bool TraditionalEnergyDetector::find_target_flow(const cv::Mat& src,std::vector<std::vector<cv::Point>>& contours)
{
  reset();
  cv::RotatedRect rota_circle;
  cv::Point2f point_1[4], point_trans[4];

  // 注意prepose中已经裁剪过原图大小了
  findContours(src, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
  // 画所有轮廓
  // cv::drawContours(img, contours, -1, cv::Scalar(255, 255, 0), 2, cv::LINE_8);

  for (int i = 0; i < contours.size(); i++)
  {
    if (contours[i].size() < 12) continue; 
    double area = cv::contourArea(contours[i]);
    if(area < 200) continue;

    if(area > 2000 and area < 18000)  //（原开符点面积约3700,远近不同变化较大，粗筛即可）
    {
    // 待打击圆环
    double perimeter = cv::arcLength(contours[i], true);
    // 圆形度(圆环越大圆形度越低，且待击打图形本身圆形度偏低，会到0.6开头)
    double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
    // RCLCPP_WARN(rclcpp::get_logg er("TraditionalE-Detect"),"circularity: %f", circularity);
    if(circularity > 0.5){
      int innerContoursCount = 0;
      double inner_area;
      if (hierarchy[i][2] != -1) 
      {
        int childIdx = hierarchy[i][2]; // 第一个子轮廓
        while (childIdx != -1) {
          inner_area = cv::contourArea(contours[childIdx]);
          if(inner_area > area * 0.01 and inner_area < area * 0.1){
            innerContoursCount ++;
          }
          childIdx = hierarchy[childIdx][0]; // 同级下一个子轮廓
        }
      }
      //std::cout<<"innerContoursCount: "<<innerContoursCount<<std::endl;
      if(innerContoursCount <= 1) continue;  //小符内轮廓数量一般为1

      // 拟合椭圆
      cv::RotatedRect ellipse = cv::fitEllipse(contours[i]);
      // 获取椭圆的长轴和短轴（注意是整体长宽的一半）
      cv::Size axes = ellipse.size;
      target_long_radius_ = std::max(axes.width, axes.height) / 2.0;
      target_short_radius_ = std::min(axes.width, axes.height) / 2.0;
      // target_angle_rad = ellipse.angle * CV_PI / 180;

      // // 获取最小外接圆
      // cv::Point2f center;
      // float radius;
      // cv::minEnclosingCircle(contours[i], center, radius);
      // rota_circle = minAreaRect(contours[i]);

      rota_far_ = ellipse;
      target_center_ = ellipse.center;
      target_area_ = M_PI * target_long_radius_ * target_short_radius_;
      find_target_ = true;
      }
    }
    //物理尺寸0.28
    if (area > (target_area_ * 0.15) and area < (target_area_ * 0.5))
    {
      find_target_Arrows(src, contours[i]);
    }
    //物理尺寸0.12
    if (area > (target_area_ * 0.02) and area < (target_area_ * 0.2))
    {
      find_target_R(src, contours[i]);
    }
    //物理尺寸2.01，只看亮部：0.4，经验值0.85
    if (area > (target_area_ * 0.6) and area < (target_area_ * 1.2))
    {
      find_target_Fans(src, contours[i]);
    }
  }
  // 如果找到圆环，用圆环与候选目标进一步比对进行筛选
  bool find_R_=false;
  if(find_target_)
  {
    if(Arrows_.size() == 0)
    {
      rota_close_ = cv::RotatedRect();
    }
    if(R_.size() == 0)
    {
      rota_R_ = cv::RotatedRect();
      circle_center_point = cv::Point2f(0, 0); 
    }
    for(int i = 0; i < Arrows_.size(); i++)
    {
      // 用待击打圆形与箭头灯条的距离继续筛选
      //物理尺寸：2.38
      float C2C_distance = cv::norm(target_center_ - Arrows_[i].center) / target_long_radius_;
      if(C2C_distance < 1.2 or C2C_distance > 4.5) continue;
      for(int j = 0; j < R_.size(); j++)
      {
        float R2C_distance = cv::norm(target_center_ - R_[j].center) / target_long_radius_;
        //物理尺寸：4.61
        if(R2C_distance < 3 or R2C_distance > 6) continue;
        if(angle_exame(Arrows_[i], R_[j]))
        {
          find_R_ = true;
          rota_close_ = Arrows_[i];
          rota_R_ = R_[j];
          circle_center_point = R_[j].center;
        }
      }
    }
    for (int i = 0; i < Fans_.size(); ) {
      bool should_erase = false;
      if (not isEmpty(rota_R_)) {
          float C2C_distance = cv::norm(circle_center_point - Fans_[i].center);
          //物理尺寸：2.3
          if (C2C_distance < target_long_radius_ || C2C_distance > 3 * target_long_radius_) {
              should_erase = true;
          } else {
              cv::Point2f corners[4];
              Fans_[i].points(corners); 
              float max_distance1 = 0.0f;
              float max_distance2 = 0.0f;
              for (int j = 0; j < 4; ++j) {
                  float dx = corners[j].x - circle_center_point.x;
                  float dy = corners[j].y - circle_center_point.y;
                  float distance = std::sqrt(dx * dx + dy * dy);
                  if (distance > max_distance1) {
                      max_distance2 = max_distance1;
                      max_distance1 = distance;
                  }
                  else if(distance > max_distance2){
                      max_distance2 = distance;
                  }
              }
              //物理尺寸：3.56
              if(max_distance1 > 4.5 * target_long_radius_ or (max_distance1 - max_distance2 > 25)){
                  should_erase = true;
              }
          }
        }
      
      if(should_erase) {
          Fans_.erase(Fans_.begin() + i);
      } else {
        ++i;
      }
    }
  }
  if(find_R_){
    find_target_ = true;
    //std::cout<<"find_target: A:"<<Arrows_.size()<<" R:"<<R_.size()<<std::endl;
  }
  else{
    find_target_ = false;
    // std::cout<<"target_lost: R_ size:"<<R_.size()<<" Arrows_ size:"<<Arrows_.size()<<" rota_far empty:"<<isEmpty(rota_far_)<<std::endl;
  }
  setPoint();
  return find_target_;
}

bool TraditionalEnergyDetector::find_target_R(const cv::Mat& src, std::vector<cv::Point>& i)
{
  cv::RotatedRect rota_R = cv::minAreaRect(i);
  
  // float R_whiteRatio = Rota_White_Ratio(rota_R, src);
  // // RCLCPP_WARN(rclcpp::get_logger("TraditionalE-Detect"),"R_whiteRatio: %f", R_whiteRatio); 
  // // 亮暗比一般比较高，但会受到前处理白色掩膜的影响
  // if (R_whiteRatio < 0.35){
  //   return false;
  // }
  // 用旋转矩形的长宽比进行筛选，R长宽比约为1
  float R_LW_ratio = rota_R.size.width / rota_R.size.height;
  // 旋转矩形的长不一定是更长的边，保证长宽比>1
  if (R_LW_ratio < 1)
  {
    R_LW_ratio = 1 / R_LW_ratio;
  }
  if (R_LW_ratio > 3)
  {
    //RCLCPP_WARN(rclcpp::get_logger("TraditionalE-Detect"),"R_L_W_ratio: %f", R_LW_ratio);
    return false;
  }
  R_.push_back(rota_R);
  return true;
}

// 查找待打击扇叶的箭头
bool TraditionalEnergyDetector::find_target_Arrows(const cv::Mat& src, std::vector<cv::Point>& i)
{
  // 轮廓逼近,使轮廓更加直线化
  std::vector<cv::Point> approxContour;
  if (!i.empty()) {
      double epsilon = 0.02 * cv::arcLength(i, true);
      cv::approxPolyDP(contours[0], approxContour, epsilon, true);
  }
  cv::RotatedRect rota_arrow = minAreaRect(i);
  // 用旋转矩形的长宽比排除一些干扰项，箭头长宽比略大于4
  float L_W_ratio = rota_arrow.size.width / rota_arrow.size.height;
  // RCLCPP_WARN(rclcpp::get_logger("TraditionalE-Detect"),"L_W_ratio: %f", L_W_ratio);
  if (L_W_ratio < 1)
  {
    L_W_ratio = 1 / L_W_ratio;
  }
  if (L_W_ratio < 3 or L_W_ratio > 6)
  {
    return false;
  }
  Arrows_.push_back(rota_arrow);
  return true;
}

// 查找已点亮扇叶
bool TraditionalEnergyDetector::find_target_Fans(const cv::Mat& src, std::vector<cv::Point>& i)
{
  cv::RotatedRect rota_fan = minAreaRect(i);
  // float Fan_whiteRatio = Rota_White_Ratio(rota_fan, src);
  // // //不同掩膜下最小0.28，最大0.4
  // if (Fan_whiteRatio > 0.55){
  //   // RCLCPP_INFO(rclcpp::get_logger("TraditionalE-Detect"),"Fan_whiteRatio: %f", Fan_whiteRatio);
  //   return false;
  // }
  // 使用多边形拟合轮廓  
  // double epsilon = 0.04 * arcLength(i, true); 
  // approxPolyDP(i, fan_approx_, epsilon, true);
  // // 判断轮廓的角点数量
  // //一般为7
  // if (fan_approx_.size() < 5 or fan_approx_.size() > 9){
  //   // RCLCPP_INFO(rclcpp::get_logger("TraditionalE-Detect"),"numVertices: %zu", fan_approx_.size());
  //   return false;
  // }  
  //缩小fan外接矩形，减小外轮廓膨胀对角点精度的影响
  rota_fan = cv::RotatedRect(rota_fan.center, cv::Size2f(rota_fan.size.width * 0.9,rota_fan.size.height * 0.93), rota_fan.angle);
  Fans_.push_back(rota_fan);
  return true;
}

void TraditionalEnergyDetector::refresh_global_Roi()
{
  int x_roi, y_roi; 
  // 以R为中心预估整个能量机关的范围。注意还是要以原图的尺寸进行计算
  if (find_target_) 
  {
    //用待打击圆环的半径预估扇叶末端到R标的距离（外框长宽的一半）
    int c2c = 7 * target_long_radius_;
    //确保不越界（-1防止长宽为零）
    global_roi_.x = (circle_center_point.x - c2c) < 0 ? 0 : (circle_center_point.x - c2c) >= raw_width_ ? raw_width_ -1  : (circle_center_point.x - c2c);
    global_roi_.y = (circle_center_point.y - c2c) < 0 ? 0 : (circle_center_point.y - c2c) >= raw_height_ ? raw_height_ -1  : (circle_center_point.y - c2c);
    x_roi = std::min(c2c * 2, raw_width_ - global_roi_.x);
    y_roi = std::min(c2c * 2, raw_height_ - global_roi_.y);    
    global_roi_ = cv::Rect(global_roi_.x, global_roi_.y, std::max(0, x_roi), std::max(0, y_roi));

  } else 
  {
    global_roi_ = cv::Rect(0, 0, raw_width_, raw_height_);
  }
}

float TraditionalEnergyDetector::Rota_White_Ratio(const cv::RotatedRect& rota, const cv::Mat& src)
{
  if(isEmpty(rota)){
    return 0;
  }
  cv::Mat mask = cv::Mat::zeros(src.size(), CV_8UC1);
  cv::Point2f vertices[4];
  rota.points(vertices);
  std::vector<cv::Point> pts;
  for (int i = 0; i < 4; i++) {
      vertices[i].x = std::max(0.f, std::min(vertices[i].x, static_cast<float>(src.cols-1)));
      vertices[i].y = std::max(0.f, std::min(vertices[i].y, static_cast<float>(src.rows-1)));
      pts.push_back(vertices[i]);
  }
  cv::fillConvexPoly(mask, pts, cv::Scalar(255));
  cv::Mat result;
  cv::bitwise_and(src, mask, result);

  int whitePixels = cv::countNonZero(result);  
  int totalPixels = cv::countNonZero(mask);       
  if(totalPixels == 0) return 0; 
  float whiteRatio = static_cast<float>(whitePixels) / totalPixels;
  return whiteRatio;
}
 
bool TraditionalEnergyDetector::angle_exame(cv::RotatedRect& Arrow, cv::RotatedRect& R)
{
  if(not find_target_){
    return false;
  }
  cv::Point v1(Arrow.center.x - target_center_.x, Arrow.center.y - target_center_.y); 
  cv::Point v2(R.center.x - target_center_.x, R.center.y - target_center_.y);
  double angle = get_angle(v1,v2);
  if(std::abs(angle) > 0.2){
    return false;
  }
  return true;
}

void TraditionalEnergyDetector::getPts()
{
  // 计算旋转点到中心点的向量并单位化
  cv::Point2f direction = {target_center_.x - circle_center_point.x, target_center_.y - circle_center_point.y};
  cv::Point2f unitVector = normalize(direction);
  // 计算垂直方向的单位向量（顺时针旋转90度）
  cv::Point2f perpendicular = {-unitVector.y, unitVector.x};
  target_pts_.resize(4); //顺时针存储
  target_pts_[0] = {target_center_.x - target_long_radius_ * unitVector.x, target_center_.y - target_long_radius_ * unitVector.y};
  target_pts_[1] = {target_center_.x - target_short_radius_ * perpendicular.x, target_center_.y - target_short_radius_ * perpendicular.y};
  target_pts_[2] = {target_center_.x + target_long_radius_ * unitVector.x, target_center_.y + target_long_radius_ * unitVector.y};
  target_pts_[3] = {target_center_.x + target_short_radius_ * perpendicular.x, target_center_.y + target_short_radius_ * perpendicular.y};
}

void TraditionalEnergyDetector::get_Fan_Pts()
{
  // 注意ROI带来的坐标变化对距离计算的影响，使用ROI内的Fans_和center的坐标进行计算
  cv::Point2f center = cv::Point2f(circle_center_point.x - global_roi_.x, circle_center_point.y - global_roi_.y);
  for(int i = 0; i < Fans_.size(); i++)
  {
    float center_distance = cv::norm(center - Fans_[i].center);
    cv::Point2f fan_vertices[4];
    Fans_[i].points(fan_vertices);
    for(cv::Point2f fan_point : fan_vertices)
    {
      float distance = cv::norm(center - fan_point);
      // 取较远端的两个角点
      if(distance > center_distance * 1.2)
      {
        fan_pts_.push_back(fan_point);
      }
    }
    // RCLCPP_WARN(rclcpp::get_logger("TraditionalE-Detect"),"fan_pts_.size: %zu", fan_pts_.size());
    if(fan_pts_.size() > 0 and fan_pts_.size() % 2 == 0)
    {
      float middle_angle = atan2((Fans_[i].center.y - center.y), (Fans_[i].center.x - center.x));
      float first_angle = atan2((fan_pts_[fan_pts_.size() - 1].y - center.y), (fan_pts_[fan_pts_.size() - 1].x - center.x));
      if(first_angle < middle_angle)
      {
        // 每次只检验最后新加入的两个角点的顺序即可
        std::swap(fan_pts_[fan_pts_.size() - 1], fan_pts_[fan_pts_.size() - 2]);
      }
      fan_pts_[fan_pts_.size() - 1].x += global_roi_.x;
      fan_pts_[fan_pts_.size() - 1].y += global_roi_.y;
      fan_pts_[fan_pts_.size() - 2].x += global_roi_.x;
      fan_pts_[fan_pts_.size() - 2].y += global_roi_.y;
    }
    else{
      if(fan_pts_.size() > 0){
        // 剔除误差角点影响
        fan_pts_.pop_back();
      }
    }
  }
}

void TraditionalEnergyDetector::setPoint()
{
  target_center_.x += global_roi_.x;
  target_center_.y += global_roi_.y;
  circle_center_point.x += global_roi_.x;
  circle_center_point.y += global_roi_.y;
  rota_far_.center += cv::Point2f(global_roi_.x,global_roi_.y);
}

void TraditionalEnergyDetector::draw_results(cv::Mat& img)
{
  cv::rectangle(img, global_roi_, cv::Scalar(255, 255, 255), 2);
  cv::putText(img, "global_roi", cv::Point(global_roi_.x, global_roi_.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

  cv::putText(img, "center_point", circle_center_point, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
  cv::circle(img, circle_center_point, 3, cv::Scalar(255, 0, 0), -1);
  float c2c = cv::norm(circle_center_point - target_center_);
  // 画整体大圆
  cv::circle(img, circle_center_point, c2c, cv::Scalar(255, 255, 0), 2);
  if(not isEmpty(rota_R_) and not isEmpty(rota_far_)){
    cv::line(img, target_center_, circle_center_point, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
  }
  //RCLCPP_WARN(rclcpp::get_logger("TraditionalE-Detect"),"c2c: %f", c2c);
  // 画关键点
  for(int i = 0; i < target_pts_.size(); i++){
    std::string text = std::to_string(i);
    cv::putText(img, text, target_pts_[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    cv::circle(img, target_pts_[i], 3, cv::Scalar(0, 255, 0), -1);
  }

  // 画R标
  cv::Point2f R_vertices[4];
  rota_R_.points(R_vertices);
  for (int i = 0; i < 4; i++) {
    R_vertices[i].x += global_roi_.x;
    R_vertices[i].y += global_roi_.y;
  }
  for (int i = 0; i < 4; i++) {
    cv::line(img, R_vertices[i], R_vertices[(i+1)%4], cv::Scalar(0, 255, 255), 2);
  }
  // 画待打击圆环
  cv::ellipse(img, rota_far_, cv::Scalar(0, 255, 255), 2);
  
  // 画箭头
  cv::Point2f arrow_vertices[4];
  rota_close_.points(arrow_vertices);
  for (int i = 0; i < 4; i++) {
    arrow_vertices[i].x += global_roi_.x;
    arrow_vertices[i].y += global_roi_.y;
  }
  for (int i = 0; i < 4; i++) {
    cv::line(img, arrow_vertices[i], arrow_vertices[(i+1)%4], cv::Scalar(0, 255, 255), 2);
  }
  // 画扇叶
  // for(auto Fan: Fans_){
  //   cv::Point2f fan_vertices[4];
  //   Fan.points(fan_vertices);
  //   for (int i = 0; i < 4; i++) {
  //     fan_vertices[i].x += global_roi_.x;
  //     fan_vertices[i].y += global_roi_.y;
  //   }
  //   for (int i = 0; i < 4; i++) {
  //     cv::line(img, fan_vertices[i], fan_vertices[(i+1)%4], cv::Scalar(0, 255, 255), 2);
  //   }
  // }
  for(int i = 0; i < fan_pts_.size() / 2; i++){
    cv::line(img, fan_pts_[2*i], fan_pts_[2*i + 1], cv::Scalar(0, 255, 255), 2);
    cv::line(img, fan_pts_[2*i], circle_center_point, cv::Scalar(0, 255, 255), 2);
    cv::line(img, fan_pts_[2*i + 1], circle_center_point, cv::Scalar(0, 255, 255), 2);
    cv::putText(img, "0", fan_pts_[2*i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::putText(img, "1", fan_pts_[2*i + 1], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  }
  // 扇叶角点的世界坐标排布
  // 画面中的坐标与世界坐标系中的不同，y轴方向相反，此处仅供pnp参考
  // std::vector<std::vector<cv::Point2f>> energy_fan_points(5);
  // float energy_fan_width = 3.7 * target_long_radius_;
  // for(int i = 0; i < 4; i++){
  //   float angle = M_PI / 2 - (i + 1) * (2 * M_PI / 5);
  //   energy_fan_points[i].emplace_back(cv::Point2f(circle_center_point.x + energy_fan_width*cos(angle - 0.33), circle_center_point.y - energy_fan_width*sin(angle - 0.33)));
  //   cv::circle(img, energy_fan_points[i][0], 3, cv::Scalar(255, 0, 0), -1);
  //   cv::line(img, energy_fan_points[i][0], circle_center_point, cv::Scalar(255, 0, 0), 2);
  //   cv::putText(img, std::to_string(i + 1) + "0", energy_fan_points[i][0], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
  //   energy_fan_points[i].emplace_back(cv::Point2f(circle_center_point.x + energy_fan_width*cos(angle + 0.33), circle_center_point.y - energy_fan_width*sin(angle + 0.33)));
  //   cv::circle(img, energy_fan_points[i][1], 3, cv::Scalar(255, 0, 0), -1);    
  //   cv::line(img, energy_fan_points[i][1], circle_center_point, cv::Scalar(255, 0, 0), 2);
  //   cv::putText(img, std::to_string(i + 1) + "1", energy_fan_points[i][1], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);   
  //   cv::line(img, energy_fan_points[i][0], energy_fan_points[i][1], cv::Scalar(255, 0, 0), 2);                                        
  // }

  // 已点亮扇叶的顺序判断
  // RCLCPP_INFO(rclcpp::get_logger("TraditionalE-Detect"),"||||||||||||||||||||");
  // for(int i = 0; i < fan_pts_.size() / 2; i++){
  //   // 记得使用平移回原图后的点计算
  //   cv::Point2f fan_center = cv::Point2f((fan_pts_[2*i].x + fan_pts_[2*i+1].x) / 2, (fan_pts_[2*i].y + fan_pts_[2*i+1].y) / 2);
  //   cv::Point v1(target_center_.x - circle_center_point.x, target_center_.y - circle_center_point.y); 
  //   cv::Point v2(fan_center.x - circle_center_point.x, fan_center.y - circle_center_point.y);
  //   double angle = get_angle(v1, v2);
  //   // 叉积，判断从v1到v2的夹角方向（>0为逆时针）
  //   double crossProduct = v1.x * v2.y - v1.y * v2.x;
  //   // 统一转为顺时针角度，范围为（0,2pi）
  //   if(crossProduct < 0){
  //     angle = 2 * M_PI - angle;
  //   }
  //   // angle = fmod(angle, 2 * M_PI);
  //   // if (angle < 0) {
  //   //   angle += 2 * M_PI;  
  //   // }
  //   // 计算 angle 是 2π/5 的多少倍
  //   double angle_multiple = angle / (2 * M_PI / 5);
  //   // 对倍数进行四舍五入
  //   int rounded_multiple = static_cast<int>(std::round(angle_multiple)); 
  //   //RCLCPP_INFO(rclcpp::get_logger("TraditionalE-Detect"),"rounded_multiple: %d", rounded_multiple);
  //   cv::putText(img, std::to_string(rounded_multiple), fan_center, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
  // }

  // 画所有轮廓
  // cv::drawContours(detect_img, contours, -1, cv::Scalar(255, 255, 0), 2, cv::LINE_8);
  // 画单个轮廓
  // const std::vector<cv::Point>& contour = i;
  // std::vector<std::vector<cv::Point>> contour_poly(1, contour);
  // cv::polylines(detect_img, contour_poly, true, cv::Scalar(255, 255, 0), 2);
}

bool TraditionalEnergyDetector::isEmpty(const cv::RotatedRect& rect) 
{
  return rect.size.width == 0 or rect.size.height == 0;
}

cv::Point2f TraditionalEnergyDetector::normalize(const cv::Point2f& pt) 
{
  float length = std::sqrt(pt.x * pt.x + pt.y * pt.y); 
  if (length > 0) {
      return cv::Point2f(pt.x / length, pt.y / length); 
  }
  return pt;
}

double TraditionalEnergyDetector::get_angle(const cv::Point v1, const cv::Point v2)
{
  // 点积
  double dotProduct = v1.x * v2.x + v1.y * v2.y;
  // 模长
  double magnitudeV1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
  double magnitudeV2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);
  // 余弦值
  double cosAngle = dotProduct / (magnitudeV1 * magnitudeV2);
  // 由于浮动误差，限制cosAngle的范围在（-1, 1）之间，且输出范围为（0，pi）
  cosAngle = std::clamp(cosAngle, -1.0, 1.0);
  double angle = std::acos(cosAngle);
  return angle;
}

void TraditionalEnergyDetector::reset()
{
  contours.clear();
  Arrows_.clear();
  R_.clear();
  Fans_.clear();
  fan_pts_.clear();
  rota_far_ = cv::RotatedRect();
  rota_close_ = cv::RotatedRect();
  circle_center_point = cv::Point2f(0, 0); 
  target_center_ = cv::Point2f(0, 0);
  target_long_radius_ = target_short_radius_ = 0.0;
  rota_R_ = cv::RotatedRect();
  find_target_ = false;
}

}  // namespace helios_cv