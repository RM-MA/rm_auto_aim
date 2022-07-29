#ifndef _PREDICTOR_EKF_HPP_
#define _PREDICTOR_EKF_HPP_

#include <Eigen/Dense>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

#include <fmt/color.h>
#include <fmt/core.h>

#include "../devices/serial/serial.hpp"
#include "../utils/robot.hpp"
#include "EKF.h"

namespace Modules
{
const auto camera_fmt =
    fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "{}", "相机坐标系");

const auto i_fmt = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "{}", "陀螺仪坐标系");

const auto world_fmt = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "{}", "惯性坐标系");

const auto predict_fmt =
    fmt::format(fg(fmt::color::light_green) | fmt::emphasis::bold, "{}", "预测");

//位姿解算
class PredictorEKF
{
public:
    explicit PredictorEKF();
    bool solve(std::vector<Robot::Armour> &, cv::Mat &);
    bool predict(
        Robot::Detection_pack &, const Devices::ReceiveData &, Devices::SendData &, cv::Mat &);

    PredictorEKF(PredictorEKF const &) = delete;
    PredictorEKF & operator=(PredictorEKF const &) = delete;

private:
    //当枪管和相机安装角度差小时，相当与变换一下xyz轴
    Eigen::Matrix3d R_CI;           // 相机坐标系 到 陀螺仪坐标系  的旋转矩阵,
    Eigen::Matrix3d F;              // 相机内参矩阵EIGEN-Matrix
    Eigen::Matrix<double, 1, 5> C;  // 相机畸变矩阵EIGEN-Matrix
    cv::Mat R_CI_MAT;  // 陀螺仪坐标系 到 相机坐标系旋转矩阵CV-Mat, 旋转+平移
    cv::Mat F_MAT;     // 相机内参矩阵CV-Mat
    cv::Mat C_MAT;     // 相机畸变矩阵CV-Mat

    // 预测
    AdaptiveEKF ekf;      // ekf预测器
    Predict predictfunc;  // f(x)
    Measure measure;      // h(x)
    double last_time = 0;
    // 弹道模型
    double k_1      = 0.01903;  // 空气阻力系数
    double K       = 0.05;     // 更新比例
    double min_ek  = 0.01;     //最小误差
    double T_k     = 0;        // 飞行时间
    int max_epochs = 10;       // 最大迭代轮数

    std::vector<cv::Point3d> small_obj, big_obj;  //大小装甲板

    // pnp解算:获取相机坐标系内装甲板坐标
    Eigen::Vector3d get_camera_points(std::vector<cv::Point2f> &, Robot::ArmourType);

    // 相机坐标系内坐标--->世界坐标系内坐标
    inline Eigen::Vector3d pc2pw(
        const Eigen::Vector3d & camera_points, const Eigen::Matrix3d & R_IW)
    {
        auto R_WC = (R_CI * R_IW).transpose();
        return R_WC * camera_points;
    }

    // 世界坐标系内坐标--->相机坐标系内坐标
    inline Eigen::Vector3d pw2pc(const Eigen::Vector3d & world_points, const Eigen::Matrix3d & R_IW)
    {
        auto R_CW = R_CI * R_IW;
        return R_CW * world_points;
    }

    // 相机坐标系内坐标--->图像坐标系内像素坐标
    inline Eigen::Vector3d pc2pu(const Eigen::Vector3d & camera_points)
    {
        return F * camera_points / camera_points(2, 0);
    }

    // 将世界坐标系内一点，投影到图像中，并绘制该点
    inline void re_project_point(
        cv::Mat & image, const Eigen::Vector3d & world_points, const Eigen::Matrix3d & R_IW,
        const cv::Scalar & color)
    {
        Eigen::Vector3d pc = pw2pc(world_points, R_IW);
        Eigen::Vector3d pu = pc2pu(pc);
        cv::circle(image, {int(pu(0, 0)), int(pu(1, 0))}, 3, color, 2);
    }
};

}  //namespace Modules

#endif /*_PREDICTOR_EKF_HPP_*/