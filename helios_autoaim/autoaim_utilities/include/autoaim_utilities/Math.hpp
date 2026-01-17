#pragma once
#include <cmath>
#include <vector>
#include <geometry_msgs/msg/point.hpp>


namespace math 
{
float get_angle_diff(float from, float to);

std::vector<float> xyz2ypd(std::vector<float> &xyz);

std::vector<float> xyz2ypd(float x, float y, float z);

double get_rad(double a);

double get_distance(const geometry_msgs::msg::Point& p);

}