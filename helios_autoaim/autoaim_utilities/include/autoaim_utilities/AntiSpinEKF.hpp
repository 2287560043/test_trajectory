#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <angles/angles.h>

namespace helios_cv {

class AntiSpinEKF {
public:
    // 状态: [xc, vxc, yc, vyc, za, vza, yaw, v_yaw, r]
    // 观测: [yaw_pos, pitch_pos, dist, yaw_orient]
    using Matrix9d = Eigen::Matrix<double, 9, 9>;
    using Vector9d = Eigen::Matrix<double, 9, 1>;
    using Matrix49d = Eigen::Matrix<double, 4, 9>;
    using Vector4d = Eigen::Matrix<double, 4, 1>;
    using Matrix4d = Eigen::Matrix<double, 4, 4>;

    Vector9d x;
    Matrix9d P;

    AntiSpinEKF() {
        x.setZero();
        P.setIdentity();
    }

    void init(const Eigen::Vector3d& armor_pos, double armor_yaw, double r0 = 0.2) {
        x.setZero();
        x(0) = armor_pos.x() - r0 * std::cos(armor_yaw); // xc
        x(2) = armor_pos.y() - r0 * std::sin(armor_yaw); // yc
        x(4) = armor_pos.z();                            // za
        x(6) = armor_yaw;                                // yaw
        x(8) = r0;                                       // r
        
        P.setIdentity();
        P.diagonal() << 10, 10, 10, 10, 1, 10, 0.1, 100, 0.1;
    }

    // [temp] 先用着再说
    void init(const Vector9d& x0, const Matrix9d& P0) {
        x = x0;
        P = P0;
    }

    Eigen::VectorXd get_state() const {
        return x;
    }

    void set_state(const Eigen::VectorXd& x_new) {
        x = x_new;
    }

    // 预测：接收 dt 和 过程噪声 Q
    void predict(double dt, const Matrix9d& Q) {
        Matrix9d F = Matrix9d::Identity();

        F(0, 1) = dt; // xc += vxc * dt
        F(2, 3) = dt; // yc += vyc * dt
        F(4, 5) = dt; // za += vza * dt
        F(6, 7) = dt; // yaw += v_yaw * dt

        x = F * x;
        P = F * P * F.transpose() + Q;
        
        x(6) = angles::normalize_angle(x(6));
    }

    // 更新：接收 观测 z 和 测量噪声 R
    void update(const Vector4d& z, const Matrix4d& R) {
        double xc = x(0), yc = x(2), za = x(4);
        double yaw = x(6), r = x(8);

        double xa_pred = xc + r * std::cos(yaw);
        double ya_pred = yc + r * std::sin(yaw);
        double za_pred = za;

        // 转为球坐标
        double dist_xy = std::sqrt(xa_pred * xa_pred + ya_pred * ya_pred);
        double dist = std::sqrt(dist_xy * dist_xy + za_pred * za_pred);
        
        // h(x)
        Vector4d z_pred;
        z_pred(0) = std::atan2(ya_pred, xa_pred); // yaw_pos
        z_pred(1) = std::atan2(za_pred, dist_xy); // pitch_pos
        z_pred(2) = dist;                         // distance
        z_pred(3) = yaw;                          // yaw_orient

        Vector4d y = z - z_pred;
        y(0) = angles::normalize_angle(y(0));
        y(1) = angles::normalize_angle(y(1));
        y(3) = angles::normalize_angle(y(3));

        // 计算雅可比 H = dh/dx
        // [temp] 先看看效果，后续再改动并写成 clang 格式
        // A. d(Cartesian)/dx  (3x9 矩阵)
        Eigen::Matrix<double, 3, 9> J_cart_x = Eigen::Matrix<double, 3, 9>::Zero();
        // xa = xc + r cos(yaw)
        J_cart_x(0, 0) = 1;                     // dxa/dxc
        J_cart_x(0, 6) = -r * std::sin(yaw);    // dxa/dyaw
        J_cart_x(0, 8) = std::cos(yaw);         // dxa/dr
        // ya = yc + r sin(yaw)
        J_cart_x(1, 2) = 1;                     // dya/dyc
        J_cart_x(1, 6) = r * std::cos(yaw);     // dya/dyaw
        J_cart_x(1, 8) = std::sin(yaw);         // dya/dr
        // za = za
        J_cart_x(2, 4) = 1;                     // dza/dza

        // B. d(Spherical)/d(Cartesian) (3x3 矩阵)
        Eigen::Matrix<double, 3, 3> J_sph_cart = Eigen::Matrix<double, 3, 3>::Zero();
        double d2 = dist * dist;
        double dxy2 = dist_xy * dist_xy;
        
        if (dxy2 < 1e-6 || d2 < 1e-6) {
            // 避免除以零，通常不会发生因为目标有一定距离
            return; 
        }

        // dyaw_p / d(xa, ya, za)
        J_sph_cart(0, 0) = -ya_pred / dxy2;
        J_sph_cart(0, 1) = xa_pred / dxy2;
        J_sph_cart(0, 2) = 0;

        // dpitch_p / d(xa, ya, za)
        J_sph_cart(1, 0) = -za_pred * xa_pred / (d2 * dist_xy);
        J_sph_cart(1, 1) = -za_pred * ya_pred / (d2 * dist_xy);
        J_sph_cart(1, 2) = dist_xy / d2;

        // ddist / d(xa, ya, za)
        J_sph_cart(2, 0) = xa_pred / dist;
        J_sph_cart(2, 1) = ya_pred / dist;
        J_sph_cart(2, 2) = za_pred / dist;

        // C. 组合 Jacobian H (4x9)
        Matrix49d H = Matrix49d::Zero();
        // 前3行 (位置球坐标) = J_sph_cart * J_cart_x
        H.block<3, 9>(0, 0) = J_sph_cart * J_cart_x;
        // 第4行 (Yaw orientation) = [0, 0, 0, 0, 0, 0, 1, 0, 0]
        H(3, 6) = 1.0;

        // 4. EKF 更新
        Matrix4d S = H * P * H.transpose() + R;
        Eigen::Matrix<double, 9, 4> K = P * H.transpose() * S.inverse();

        x = x + K * y;
        P = (Matrix9d::Identity() - K * H) * P;
        
        x(6) = angles::normalize_angle(x(6));
    }
};

} // namespace helios_cv