#include <Eigen/Dense>

#include <fmt/color.h>
#include <fmt/core.h>

#include <iostream>

class Kalman_Filter
{
public:
    using Matrix_zzd = Eigen::Matrix<double, 3, 3>;
    using Matrix_xxd = Eigen::Matrix<double, 6, 6>;
    using Matrix_zxd = Eigen::Matrix<double, 3, 6>;
    using Matrix_xzd = Eigen::Matrix<double, 6, 3>;
    using Matrix_z1d = Eigen::Matrix<double, 3, 1>;
    using Matrix_x1d = Eigen::Matrix<double, 6, 1>;

private:
    Matrix_x1d x_k1;  // k-1时刻的滤波值
    Matrix_xzd K;     //Kalman增益
    Matrix_xxd F;     //转移矩阵
    Matrix_zxd H;     //观测矩阵
    Matrix_xxd Q;     //预测过程的噪音
    Matrix_zzd R;     //测量噪声偏差
    Matrix_xxd P;     //估计误差协方差

    double last_t;

public:
    Kalman_Filter() = default;
    Kalman_Filter(Matrix_xxd F, Matrix_xxd Q, Matrix_zxd H, Matrix_zzd R, Matrix_x1d init, double t)
    {
        reset(F, Q, H, R, init, t);
    }

    void reset(Matrix_xxd F, Matrix_xxd Q, Matrix_zxd H, Matrix_zzd R, Matrix_x1d init, double t)
    {
        this->F = F;
        this->Q = Q;

        this->H = H;
        this->R = R;

        this->P = Matrix_xxd::Zero();

        this->x_k1 = init;

        this->last_t = t;
    }

    Matrix_x1d update(Matrix_z1d z_k, double t)
    {
        // V_X = 6, (x, y, z, v_x, v_y, v_z)
        // 更新时间 dt
        for (int i = 0; i < 6 / 2; i++) {
            F(i, i + 3) = t - last_t;
        }
        // std::cout << F << std::endl;
        last_t = t;

        // 计算 先验估计
        Matrix_x1d p_x_k = F * x_k1;

        // 求协方差
        P = F * P * F.transpose() + Q;

        // 计算 Kalman 增益
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

        // 修正结果
        x_k1 = p_x_k + K * (z_k - H * p_x_k);

        // 更新后验估计
        P = (Matrix_xxd::Identity() - K * H) * P;

        return x_k1;
    }
};

int main()
{
    //
    using KF = Kalman_Filter;

    KF::Matrix_xxd F = KF::Matrix_xxd::Identity();  //

    KF::Matrix_zxd H;  //3 x 6
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 2) = 1;

    KF::Matrix_xxd Q = KF::Matrix_xxd::Identity();

    KF::Matrix_zzd R = KF::Matrix_zzd::Identity();

    KF::Matrix_x1d init{0, 0, 0, 0, 0, 0};

    Kalman_Filter kf{F, Q, H, R, init, 0};

    double t = 1;
    double x, y, z;
    for (; t <= 20; t++) {
        fmt::print(fg(fmt::color::orange), "Epochs: {}/20\n", t);
        x = 1.5 * t;
        y = 2.0 * t;
        z = t * t;
        // z = 2 * t * t;
        KF::Matrix_z1d z_k{x, y, z};
        KF::Matrix_x1d x_k = kf.update(z_k, t);
        fmt::print("Now: [x={}, y={}, z={}]\n", x, y, z);
        fmt::print(
            "Predict: Position: [x={:.3f}, y={:.3f}, z={:.3f}]\n \t Speed: [v_x={:.3f}, v_y={:.3f}, v_z={:.3f}]\n",
            x_k(0, 0), x_k(1, 0), x_k(2, 0), x_k(3, 0), x_k(4, 0), x_k(5, 0));

        fmt::print(
            fg(fmt::color::aqua), "Speed diff: [v_x = {:.3f}, v_y = {:.3f}, v_z = {:.3f}]\n", 1.5 - x_k(3, 0),
            2.0 - x_k(4, 0), 4 * t - x_k(5, 0));
    }

    return 0;
}