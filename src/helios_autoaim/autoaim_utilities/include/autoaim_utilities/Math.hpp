#pragma once
#include <cmath>
#include <vector>

namespace math 
{
float shortest_angular_distance(float from, float to);

std::vector<float> xyz2ypd(std::vector<float> &xyz);

std::vector<float> xyz2ypd(float x, float y, float z);

double get_rad(double a);

}
