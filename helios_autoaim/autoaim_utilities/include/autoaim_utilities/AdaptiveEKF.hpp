#pragma once

#include <Eigen/Dense>
#include <ceres/jet.h>
#include <iostream>

namespace helios_cv {

// N_X: 状态维数, N_Y: 观测维数
template<int N_X, int N_Y>
class AdaptiveEkf {
public:
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYX = Eigen::Matrix<double, N_Y, N_X>;
    using MatrixXY = Eigen::Matrix<double, N_X, N_Y>;
    using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
    using MatrixX1 = Eigen::Matrix<double, N_X, 1>;
    using MatrixY1 = Eigen::Matrix<double, N_Y, 1>;

private:
    MatrixX1 x_e;   // 估计状态
    MatrixXX p_mat; // 协方差矩阵

public:
    AdaptiveEkf() : x_e(MatrixX1::Zero()), p_mat(MatrixXX::Identity()) {}

    void init(const MatrixX1& x0, const MatrixXX& P0) {
        x_e = x0;
        p_mat = P0;
    }

    MatrixX1 get_state() const { return x_e; }
    MatrixXX get_P() const { return p_mat; }

    // 预测结果
    struct PredictResult {
        MatrixX1 x_p;
        MatrixXX f_mat;
    };

    // predict
    template<class PredictFunc>
    PredictResult predict(PredictFunc&& predict_func) {
        ceres::Jet<double, N_X> x_curr_jet[N_X];
        for (int i = 0; i < N_X; ++i) {
            x_curr_jet[i].a = x_e[i];
            x_curr_jet[i].v[i] = 1.0;
        }

        ceres::Jet<double, N_X> x_next_jet[N_X];
        predict_func(x_curr_jet, x_next_jet);

        MatrixX1 x_p;
        MatrixXX f_mat;
        for (int i = 0; i < N_X; ++i) {
            x_p[i] = x_next_jet[i].a;
            f_mat.block(i, 0, 1, N_X) = x_next_jet[i].v.transpose();
        }
        return {x_p, f_mat};
    }

    // 预测更新
    template<class PredictFunc>
    void predict_forward(PredictFunc&& predict_func, const MatrixXX& Q) {
        PredictResult res = predict(predict_func);
        x_e = res.x_p;
        p_mat = res.f_mat * p_mat * res.f_mat.transpose() + Q;
    }

    // 观测结果
    struct MeasureResult {
        MatrixY1 y_e;
        MatrixYX h_mat;
    };

    // measure
    template<class MeasureFunc>
    MeasureResult measure(MeasureFunc&& measure_func) {
        ceres::Jet<double, N_X> x_curr_jet[N_X];
        for (int i = 0; i < N_X; ++i) {
            x_curr_jet[i].a = x_e[i];
            x_curr_jet[i].v[i] = 1.0;
        }

        ceres::Jet<double, N_X> y_pred_jet[N_Y];
        measure_func(x_curr_jet, y_pred_jet);

        MatrixY1 y_e;
        MatrixYX h_mat;
        for (int i = 0; i < N_Y; ++i) {
            y_e[i] = y_pred_jet[i].a;
            h_mat.block(i, 0, 1, N_X) = y_pred_jet[i].v.transpose();
        }
        return {y_e, h_mat};
    }

    // 观测更新
    template<class MeasureFunc>
    void update_forward(MeasureFunc&& measure_func, const MatrixY1& y, const MatrixYY& R) {
        MeasureResult res = measure(measure_func);
        MatrixXY K = p_mat * res.h_mat.transpose() * (res.h_mat * p_mat * res.h_mat.transpose() + R).inverse();
        
        // 需保证 y 与 y_e 在同一周期
        x_e = x_e + K * (y - res.y_e);
        p_mat = (MatrixXX::Identity() - K * res.h_mat) * p_mat;
    }
};

} // namespace helios_cv