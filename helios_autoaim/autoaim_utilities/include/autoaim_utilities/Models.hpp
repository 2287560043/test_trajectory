#pragma once
#include <autoaim_utilities/AdaptiveEkf.hpp>
#include <cmath>

namespace helios_cv {

// 单 armor 预测模型
// 状态: [x, vx, y, vy, z, vz] (odoom)
// 观测: [yaw, pitch, dist] (gimbal)
struct ArmorState {
    double x, vx, y, vy, z, vz;
};

struct ArmorPredictFunctor {
    double dt;
    explicit ArmorPredictFunctor(double t) : dt(t) {}

    template<typename T>
    void operator()(const T* x, T* x_next) const {
        x_next[0] = x[0] + x[1] * T(dt); // x
        x_next[1] = x[1];                // vx
        x_next[2] = x[2] + x[3] * T(dt); // y
        x_next[3] = x[3];                // vy
        x_next[4] = x[4] + x[5] * T(dt); // z
        x_next[5] = x[5];                // vz
    }
};
// xyz 需要基于 odoom 坐标系(左手系)
struct ArmorMeasureFunctor {
    template<typename T>
    void operator()(const T* x, T* y) const {
        T r2 = x[0]*x[0] + x[2]*x[2];               // r^2         
        y[0] = ceres::atan2(x[2], x[0]);            // Yaw 
        y[1] = ceres::atan2(x[4], ceres::sqrt(r2)); // Pitch
        y[2] = ceres::sqrt(r2 + x[4]*x[4]);         // Dist
    }
};

using ArmorEkf = AdaptiveEkf<6, 3>;

// ==========================================
// 2. Top Model (陀螺模式 - 拟合圆心)
// ==========================================
// 状态: [cx, cy, r, theta, omega] (简化版 2D 陀螺模型)
// 观测: [x_armor, y_armor] (观测到的装甲板位置)

struct TopPredictFunctor {
    double dt;
    explicit TopPredictFunctor(double t) : dt(t) {}

    template<typename T>
    void operator()(const T* s, T* s_next) const {
        s_next[0] = s[0]; // cx 固定 (短期)
        s_next[1] = s[1]; // cy 固定
        s_next[2] = s[2]; // r 固定
        s_next[3] = s[3] + s[4] * T(dt); // theta = theta + w * dt
        s_next[4] = s[4]; // w 固定
    }
};

struct TopMeasureFunctor {
    template<typename T>
    void operator()(const T* s, T* meas) const {
        // x = cx + r * cos(theta)
        // y = cy + r * sin(theta)
        meas[0] = s[0] + s[2] * ceres::cos(s[3]);
        meas[1] = s[1] + s[2] * ceres::sin(s[3]);
    }
};

using TopEkf = AdaptiveEkf<5, 2>;

} // namespace helios_cv