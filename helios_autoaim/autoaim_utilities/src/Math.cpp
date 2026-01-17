#include "autoaim_utilities/Math.hpp"


namespace math
{
float get_angle_diff(float from, float to)
{
    const float result = fmod((to - from) + M_PI, M_PI * 2.0f);
    if (result <= 0.0f)
        return result + M_PI;
    return result - M_PI;
}

std::vector<float> xyz2ypd(std::vector<float> &xyz)
{
    std::vector<float> ypd(3);
    float x = xyz[0], y = xyz[1], z = xyz[2];
    ypd[0] = -atan2(-y, x);
    ypd[1] = atan2(z, sqrt(y * y + x * x));
    ypd[2] = sqrt(x * x + y * y + z * z);
    return ypd;
}

std::vector<float> xyz2ypd(float x, float y, float z)
{
    std::vector<float> ypd(3);
    ypd[0] = -atan2(-y, x);
    ypd[1] = atan2(z, sqrt(y * y + x * x));
    ypd[2] = sqrt(x * x + y * y + z * z);
    return ypd;
}

double get_rad(double a) 
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
};

double get_cost(const geometry_msgs::msg::Point& p) {
    return std::sqrt(p.x * p.x + p.y * p.y);
};

}
