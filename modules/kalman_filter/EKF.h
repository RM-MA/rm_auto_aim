#ifndef _EKF_H_
#define _EKF_H_

#include <fmt/color.h>
#include <fmt/core.h>

#include <ceres/jet.h>

#include <Eigen/Dense>

using MatrixZZ = Eigen::Matrix<double, 3, 3>;
using MatrixXZ = Eigen::Matrix<double, 6, 3>;
using MatrixZX = Eigen::Matrix<double, 3, 6>;
using MatrixXX = Eigen::Matrix<double, 6, 6>;
using VectorZ  = Eigen::Matrix<double, 3, 1>;
using VectorX  = Eigen::Matrix<double, 6, 1>;

struct Predict  // f(x)
{
    /*
     * 此处定义匀速直线运动模型
     */
    template <class T> void operator()(const T x0[6], T x1[6])
    {
        x1[0] = x0[0] + delta_t * x0[3];  // x = x' + v_x * t
        x1[1] = x0[1] + delta_t * x0[4];  // y = y' + v_y * t
        x1[2] = x0[2] + delta_t * x0[5];  // z = z' + v_z * t
        x1[3] = x0[3];                    // v_x
        x1[4] = x0[4];                    // v_y
        x1[5] = x0[5];                    // v_z
    }

    double delta_t;
};

struct Measure  // h(x)
{
    /*
     * 工具函数的类封装
     */
    template <class T>
    //结果到y
    void operator()(const T x[6], T y[3])
    {
        y[0] = x[0], y[1] = x[1], y[2] = x[2];
        // ceres::Jet<double, 3>::x
    }
};

class AdaptiveEKF
{
public:
public:
    explicit AdaptiveEKF(const VectorX & X0 = VectorX::Zero())  //Identity()初始化一个单位矩阵
    : Xe(X0), P(MatrixXX::Identity()), Q(MatrixXX::Identity()), R(MatrixZZ::Identity())
    {
    }

    void init(const VectorX & X0 = VectorX::Zero())
    {
        Xe = X0;
    }

    template <class Func> VectorX predict(Func && func)
    {
        ceres::Jet<double, 6> Xe_auto_jet[6];

        for (int i = 0; i < 6; i++) {
            Xe_auto_jet[i].a    = Xe[i];
            Xe_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, 6> Xp_auto_jet[6];
        func(Xe_auto_jet, Xp_auto_jet);
        for (int i = 0; i < 6; i++) {
            Xp[i] = Xp_auto_jet[i].a;
            //更新 F-预测雅克比
            //block(i,j,p,q) 提取块大小为(p,q),起始于(i,j)
            F.block(i, 0, 1, 6) = Xp_auto_jet[i].v.transpose();
        }
        // std::cout << "F=\n" <<  F << std::endl;
        //更新 P-状态协方差
        P = F * P * F.transpose() + Q;

        return Xp;
    }

    template <class Func> VectorX update(Func && func, const VectorZ & Z)
    {
        ceres::Jet<double, 6> Xp_auto_jet[6];
        for (int i = 0; i < 6; i++) {
            Xp_auto_jet[i].a    = Xp[i];
            Xp_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, 6> Zp_auto_jet[3];
        func(Xp_auto_jet, Zp_auto_jet);
        for (int i = 0; i < 3; i++) {
            Zp[i]               = Zp_auto_jet[i].a;
            H.block(i, 0, 1, 6) = Zp_auto_jet[i].v.transpose();
        }
        // std::cout << "H=\n" << H << std::endl;
        //S_k = (H * P * H.transpose() + R).inverse()
        K  = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        Xe = Xp + K * (Z - Zp);
        P  = (MatrixXX::Identity() - K * H) * P;
        return Xe;
    }

    VectorX Xe;  // 估计状态变量, <X_k-1> 上一次的估计值
    VectorX Xp;  // 预测状态变量 X'_k
    MatrixXX F;  // 预测雅克比
    MatrixZX H;  // 观测雅克比
    MatrixXX P;  // 状态协方差

    MatrixXX Q;  // 预测过程协方差
    MatrixZZ R;  // 观测过程协方差
    MatrixXZ K;  // 卡尔曼增益
    VectorZ Zp;  // 预测观测量
};

#endif /*_EKF_H_*/
