#ifndef _ROBOT_HPP_
#define _ROBOT_HPP_

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

namespace Robot
{
enum class RobotType {
    Hero,      //英雄
    Infantry,  //步兵
    Sentry,    //哨兵
};

enum class EnemyColor  //敌方颜色
{ BLUE,
  RED };

struct Light
{
};

struct Armour
{
    Light &ll, lr;      //左右灯条
    RobotType type;     //机器人类型
    cv::Point3f * cps;  //Camera Points, 相机坐标系
    cv::Point3f * wps;  //World Points, 世界坐标系
};

struct Detection_pack  //打包数据结构
{
    cv::Mat img;     //图像
    double timestamp;  //时间戳
    // cv::Point2f * pts; //
    Armour * armours;
};

enum class ShootLevel { Level1, Level2, Level3 };

}  // namespace Robot

#endif /*_ROBOT_HPP_*/