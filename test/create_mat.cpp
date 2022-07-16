#include <Eigen/Dense>
// #include <opencv2/core/persistence.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <fmt/core.h>


#include <string>
#include <vector>

int main()
{
    //只表示了旋转, 没表示平移
    double roll = 0, pitch = 0, yaw = 0;
    //欧拉角, rpy = roll(绕x旋转), pitch(y), yaw(z), 弧度
    Eigen::Vector3d ea(roll, pitch, yaw);  // PI pitch/M_PI, yaw/M_PI, 0
    //    std::cout<<"ea = \n" << ea<<std::endl;
    // printf("yaw =%lf PI, pitch = %lf PI\n", yaw, pitch);
    Eigen::AngleAxisd rotation_vector3;  //一个转轴和一个转角
    rotation_vector3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());

    Eigen::Matrix3d R_CW = rotation_vector3.matrix();  //相机坐标系到陀螺仪坐标系
    cv::Mat R_CI_MAT;
    cv::eigen2cv(R_CW, R_CI_MAT);
    // cv::eigen2cv(R_CW, R_CI_MAT);
    cv::FileStorage fout{"../mat.yaml", cv::FileStorage::WRITE};
    fout << "Tcb" << R_CI_MAT;


    return 0;
}