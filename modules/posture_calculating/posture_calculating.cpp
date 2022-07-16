#include "posture_calculating.hpp"
#include "robot.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <vector>

#include <fmt/color.h>

#include <opencv2/core/hal/interface.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>

namespace Modules
{
Posture_Calculating::Posture_Calculating()
{
    cv::FileStorage file_read{
        PROJECT_DIR "/Configs/posture_calculating/posture_calculating.yaml", cv::FileStorage::READ};
    file_read["camera"]["cameraMatrix"] >> F_MAT;  //相机内参
    file_read["camera"]["distCoeffs"] >> C_MAT;    //相机畸变
    file_read["camera"]["Tcb"] >> R_CI_MAT; //陀螺仪坐标系到相机坐标系的旋转矩阵

    cv::cv2eigen(F_MAT, F);
    cv::cv2eigen(C_MAT, C);
    cv::cv2eigen(R_CI_MAT, R_CI);

    double small_half_x, small_half_y;
    double big_half_x, big_half_y;

    file_read["armour"]["small_half_x"] >> small_half_x;
    file_read["armour"]["small_half_x"] >> small_half_x;
    file_read["armour"]["big_half_x"] >> big_half_x;
    file_read["armour"]["big_half_y"] >> big_half_y;

    /*
    - point 0: [-squareLength / 2, squareLength / 2, 0]
    - point 1: [ squareLength / 2, squareLength / 2, 0]
    - point 2: [ squareLength / 2, -squareLength / 2, 0]
    - point 3: [-squareLength / 2, -squareLength / 2, 0]
    */
    small_obj = std::vector<cv::Point3f>{
        cv::Point3f(-small_half_x, small_half_y, 0),    //left top
        cv::Point3f(small_half_x, small_half_y, 0),     //right top
        cv::Point3f(small_half_x, -small_half_y, 0),    //right bottom
        cv::Point3f(-small_half_x, -small_half_y, 0)};  //left bottom
    big_obj = std::vector<cv::Point3f>{
        cv::Point3f(-big_half_x, big_half_y, 0),    //left top
        cv::Point3f(big_half_x, big_half_y, 0),     //right top
        cv::Point3f(big_half_x, -big_half_y, 0),    //right bottom
        cv::Point3f(-big_half_x, -big_half_y, 0)};  //left bottom
}

bool Posture_Calculating::solvepnp(Robot::Armour & armour)
{
    std::vector<cv::Point2f> armour_points;

    armour_points.emplace_back(armour.right_light.top);
    armour_points.emplace_back(armour.right_light.bottom);
    armour_points.emplace_back(armour.left_light.bottom);
    armour_points.emplace_back(armour.left_light.top);

    cv::Mat rvec, tvec;  //
    bool success;
    if (armour.armour_type == Robot::ArmourType::Big) {
        success =
            cv::solvePnP(big_obj, armour_points, F_MAT, C_MAT, rvec, tvec, false, cv::SOLVEPNP_P3P);
    } else {
        success = cv::solvePnP(
            small_obj, armour_points, F_MAT, C_MAT, rvec, tvec, false, cv::SOLVEPNP_P3P);
    }

    armour.camera_points.x = tvec.at<double>(0);
    armour.camera_points.y = tvec.at<double>(1);
    armour.camera_points.z = tvec.at<double>(2);

    fmt::print(
        fg(fmt::color::yellow), "{}:[{:.4f}, {:.4f}, {:.4f}]\n", camera_fmt, armour.camera_points.x,
        armour.camera_points.y, armour.camera_points.z);
    return success;
}

bool Posture_Calculating::solve(Robot::Detection_pack & detection_pack)
{
    auto & armours = detection_pack.armours;

    cv::Mat & img    = detection_pack.img;
    double timestamp = detection_pack.timestamp;

    if (armours.size() == 0) {
        return false;
    }
    //对装甲板进行排序, 优先大装甲板, 其次选最大的
    std::sort(
        armours.begin(), armours.end(), [](const Robot::Armour & a1, const Robot::Armour & a2) {
            if (a1.armour_type == Robot::ArmourType::Big &&
                a2.armour_type == Robot::ArmourType::Small) {
                return true;
            } else if (
                a1.armour_type == Robot::ArmourType::Small &&
                a2.armour_type == Robot::ArmourType::Big) {
                return false;
            }

            auto w1      = a1.right_light.top - a1.left_light.top;
            auto h1      = a1.left_light.bottom - a1.left_light.top;
            double area1 = std::fabs(w1.x * h1.y - w1.y * h1.x);

            auto w2      = a2.right_light.top - a2.left_light.top;
            auto h2      = a2.left_light.bottom - a2.left_light.top;
            double area2 = std::fabs(w2.x * h2.y - w2.y * h2.x);

            return area1 > area2;
        });

    auto select_armour = armours.front();
    solvepnp(select_armour);

    return true;
}
}  // namespace Modules