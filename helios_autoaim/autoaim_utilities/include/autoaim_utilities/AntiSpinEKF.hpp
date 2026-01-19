#include <Eigen/Dense>
#include <cmath>
#include <iostream>

// 辅助函数：角度归一化 (-PI, PI)
template <typename T>
T normalize_angle(T angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

class AntiSpinEKF {
public:
    // 状态: [xc, vxc, yc, vyc, za, vza, yaw, v_yaw, r]
    // 维度: 9
    // 观测: [yaw_pos, pitch_pos, dist, yaw_orient]
    // 维度: 4
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
        // 初始协方差：位置速度给大点，半径给小点
        P.diagonal() << 10, 10, 10, 10, 1, 1, 1, 10, 0.1;
    }

    void init(const Eigen::Vector3d& armor_pos, double armor_yaw, double r0 = 0.2) {
        x.setZero();
        // 假设车体中心在装甲板后方 r0 处
        // x_c = x_a - r * cos(yaw)
        x(0) = armor_pos.x() - r0 * std::cos(armor_yaw); // xc
        x(2) = armor_pos.y() - r0 * std::sin(armor_yaw); // yc
        x(4) = armor_pos.z();                            // za
        x(6) = armor_yaw;                                // yaw (orientation)
        x(8) = r0;                                       // r
        
        // 初始 P 阵
        P.setIdentity();
        P.diagonal() << 0.1, 10, 0.1, 10, 0.1, 10, 0.05, 100, 0.01;
    }

    void predict(double dt) {
        Matrix9d F = Matrix9d::Identity();
        // 线性运动模型
        F(0, 1) = dt; // xc += vxc * dt
        F(2, 3) = dt; // yc += vyc * dt
        F(4, 5) = dt; // za += vza * dt
        F(6, 7) = dt; // yaw += v_yaw * dt

        // 过程噪声 Q
        Matrix9d Q = Matrix9d::Identity();
        double q_pos_xy = 0.05 * dt;
        double q_vel_xy = 2.0 * dt;  // 允许车体加减速
        double q_z = 0.01 * dt;
        double q_yaw = 0.1 * dt;
        double q_v_yaw = 15.0 * dt;  // 自旋角速度变化可能很快 (小陀螺开关)
        double q_r = 0.001 * dt;     // 半径基本不变

        Q.diagonal() << q_pos_xy, q_vel_xy, q_pos_xy, q_vel_xy, 
                        q_z, q_z, q_yaw, q_v_yaw, q_r;

        x = F * x;
        P = F * P * F.transpose() + Q;
        
        x(6) = normalize_angle(x(6));
    }

    // 更新步：Z = [yaw_pos, pitch_pos, dist, yaw_orient]
    // 注意：这里的 yaw_pos, pitch_pos 是装甲板位置矢量的球坐标，不是相机云台轴
    void update(const Vector4d& z) {
        // 1. 预测观测 h(x)
        double xc = x(0), yc = x(2), za = x(4);
        double yaw = x(6), r = x(8);

        // 预测装甲板的笛卡尔坐标
        double xa_pred = xc + r * std::cos(yaw);
        double ya_pred = yc + r * std::sin(yaw);
        double za_pred = za;

        // 转为球坐标 (Position Yaw/Pitch/Dist)
        double dist_xy = std::sqrt(xa_pred * xa_pred + ya_pred * ya_pred);
        double dist = std::sqrt(dist_xy * dist_xy + za_pred * za_pred);
        
        // h(x)
        Vector4d z_pred;
        z_pred(0) = std::atan2(ya_pred, xa_pred); // yaw_pos
        z_pred(1) = std::atan2(za_pred, dist_xy); // pitch_pos
        z_pred(2) = dist;                         // distance
        z_pred(3) = yaw;                          // yaw_orient

        // 2. 计算残差 y = z - h(x)
        Vector4d y = z - z_pred;
        y(0) = normalize_angle(y(0));
        y(1) = normalize_angle(y(1));
        y(3) = normalize_angle(y(3));

        // 3. 计算雅可比 H = dh/dx
        // 需要链式法则: d(Spherical)/dx = d(Spherical)/d(Cartesian) * d(Cartesian)/dx
        
        // A. d(Cartesian)/dx  (3x9 矩阵)
        // P_cart = [xa, ya, za]
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
        // S = [yaw_p, pitch_p, dist]
        Eigen::Matrix<double, 3, 3> J_sph_cart = Eigen::Matrix<double, 3, 3>::Zero();
        double d2 = dist * dist;
        double dxy2 = dist_xy * dist_xy;
        
        // dyaw_p / d(xa, ya, za)
        J_sph_cart(0, 0) = -ya_pred / dxy2;
        J_sph_cart(0, 1) = xa_pred / dxy2;
        J_sph_cart(0, 2) = 0;

        // dpitch_p / d(xa, ya, za)
        // pitch = atan(z / dxy)
        // u = z/dxy, dp/du = 1/(1+u^2) = 1/(1 + z^2/dxy^2) = dxy^2 / d^2
        // du/dx = -z * xa / dxy^3
        // dp/dx = (dxy^2/d^2) * (-z*xa/dxy^3) = -z*xa / (d^2 * dxy)
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

        // 4. 测量噪声 R
        Matrix4d R = Matrix4d::Identity();
        // 角度测得准，距离测不准
        R.diagonal() << 0.01, 0.01, 0.1, 0.05; 

        // 5. EKF 更新
        Matrix4d S = H * P * H.transpose() + R;
        Eigen::Matrix<double, 9, 4> K = P * H.transpose() * S.inverse();

        x = x + K * y;
        P = (Matrix9d::Identity() - K * H) * P;
        
        x(6) = normalize_angle(x(6));
    }
};