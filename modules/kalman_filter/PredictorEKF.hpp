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

const auto gimbal_fmt =
    fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "{}", "云台坐标系");

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
    Eigen::Matrix3d R_C2G;          // 相机坐标系 到 云台坐标系  的旋转矩阵,
    Eigen::Matrix3d F;              // 相机内参矩阵EIGEN-Matrix
    Eigen::Matrix<double, 1, 5> C;  // 相机畸变矩阵EIGEN-Matrix
    cv::Mat R_C2G_MAT;  // 相机坐标系 到 云台坐标系 旋转矩阵CV-Mat, 旋转+平移
    cv::Mat F_MAT;      // 相机内参矩阵CV-Mat
    cv::Mat C_MAT;      // 相机畸变矩阵CV-Mat

    // 预测
    AdaptiveEKF ekf;      // ekf预测器
    Predict predictfunc;  // f(x)
    Measure measure;      // h(x)
    double last_time = 0;
    // 弹道模型


    std::vector<cv::Point3d> small_obj, big_obj;  //大小装甲板

    // pnp解算:获取相机坐标系内装甲板坐标
    Eigen::Vector3d get_camera_points(std::vector<cv::Point2f> &, Robot::ArmourType);

    // 相机坐标系内坐标--->世界坐标系内坐标
    inline Eigen::Vector3d pc2pw(
        const Eigen::Vector3d & camera_points, const Eigen::Matrix3d & R_G2W)
    {
        auto R_C2W = R_C2G * R_G2W;
        return R_C2W * camera_points;
    }

    // 世界坐标系内坐标--->相机坐标系内坐标
    inline Eigen::Vector3d pw2pc(
        const Eigen::Vector3d & world_points, const Eigen::Matrix3d & R_G2W)
    {
        auto R_W2C = (R_C2G * R_G2W).transpose();
        return R_W2C * world_points;
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


public:

    // 得到从云台坐标系到世界坐标系的旋转矩阵
    static Eigen::Matrix3d get_R_G2W(const Devices::ReceiveData & receive_data)
    {
        //
        float receive_pitch = (receive_data.pitch / 180.) * M_PI;  // 上+，X轴向Z轴转
        float receive_yaw   = (receive_data.yaw / 180.) / M_PI;    // 左+，X轴向Y轴转

        // 云台坐标系下- 云台坐标系的X轴
        Eigen::Vector3d X_Gimbal(1, 0, 0);
        // 世界坐标系下- 云台坐标系的X轴
        Eigen::Vector3d X_World;

        double x = X_Gimbal.norm() / (1 + pow(tan(receive_pitch), 2) + pow(tan(receive_yaw), 2));
        // 归一化
        X_World[0] = x;
        X_World[1] = x * tan(receive_yaw);
        X_World[2] = x * tan(receive_pitch);

        Eigen::Quaterniond R_G2W = Eigen::Quaterniond::FromTwoVectors(X_Gimbal, X_World);

        // cout << "X World =\n" << X_Gimbal << "\nX World = \n" << X_World << endl;
        // cout << "R Gimbal to World = \n" << R_G2W.toRotationMatrix() << endl;

        return R_G2W.toRotationMatrix();
    }


    double k;  // 空气阻力系数     = 0.01903
    const double g = 9.8;

    int max_iter      = 10;  // 最大迭代轮数
    int R_K_iter      = 60;
    double stop_error = 0.001;
    double static_yaw, static_pitch;

    bool solve_ballistic_model(const Eigen::Vector3d& world_points,const Devices::ReceiveData & receive_data, float& solve_yaw);
};

}  //namespace Modules

#endif /*_PREDICTOR_EKF_HPP_*/