// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp> 
// STL
#include <algorithm>
#include <string>

namespace helios_cv
{
enum Color
{
  RED ,
  BLUE ,
  EXTINGUISH ,
  PURPLE 
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType
{
  SMALL,
  LARGE,
  // 由于还没有energy的专属类，所以energy临时寄放在这里
  ENERGY_FAN,
  ENERGY_TARGET,
  
  INVALID

};
const std::vector<std::string> ARMOR_TYPE_STR = {"small", "large", "energy_fan", "energy_target", "invalid"};

enum ArmorPriority
{
  FIRST,
  SENCOND,
  THIRD,
  FOURTH,
  FIFTH
};
const std::vector<std::string> ARMOR_PRIORITY_STR = {"first", "sencond", "third", "fourth", "fifth"};

// 从分类器开始就已经存在一套命名体系，所以这里就不重复命名了
// enum ArmorName
// {
//   ONE,
//   TWO,
//   THREE,
//   FOUR,
//   FIVE,
//   SENTRY,
//   OUTPOST,
//   BASE
// };
// const std::vector<std::string> ARMOR_NAME_STR = {"one", "two", "three", "four", "five", "sentry", "outpost", "base"};



struct Light : public cv::RotatedRect
{
  Light() = default;

  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
  {
    cv::Point2f p[4];
    box.points(p);
    std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }

  int color;
  cv::Point2f top, bottom;
  double length;
  double width;
  float tilt_angle;
};

struct Armor
{
  Armor() = default;
  Armor(const Light& l1, const Light& l2)
  {
    if (l1.center.x < l2.center.x)
    {
      left_light = l1, right_light = l2;
    }
    else
    {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
    // Angle of light center connection
    cv::Point2f diff = l1.center - l2.center;
    angle = std::atan(diff.y / diff.x) / CV_PI * 180;
    
  }

  // Light pairs part
  Light left_light, right_light;
  cv::Point2f center;
  double angle;
  ArmorType type;

  // Number part
  cv::Mat number_img;
  std::string number;
  float confidence;
  std::string classfication_result;
  double getArea() const
  {
    cv::Point2f pts[4];
    pts[0] = left_light.top;
    pts[1] = right_light.top;
    pts[2] = right_light.bottom;
    pts[3] = left_light.bottom;
    return cv::contourArea(std::vector<cv::Point2f>(pts, pts + 4));
  }
};

struct Armors
{
  std::vector<Armor> armors;
  virtual ~Armors() = default;
};

}  // namespace helios_cv

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
