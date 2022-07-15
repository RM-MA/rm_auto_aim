#ifndef _ROBOT_HPP_
#define _ROBOT_HPP_

#include <Eigen/Dense>
// #include <opencv2/core/types.hpp>
#include <algorithm>
#include <cmath>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

namespace Robot
{
enum class RobotType {
    Hero,        //英雄
    Engineer,    //工程
    Infantry_3,  //步兵3号
    Infantry_4,  //步兵4号
    Infantry_5,  //步兵5号
    Sentry,      //哨兵
};

enum class ArmourType { Small, Big };

enum class Color  //敌方颜色
{ BLUE,
  RED };

struct Light : public cv::RotatedRect
{
    Light() = default;
    explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
    {
        cv::Point2f p[4];
        box.points(p);
        std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) {
            return a.y < b.y;
        });  //y值小的,在前面
        top    = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        //norm, 二阶范数, 相当与求距离
        length = cv::norm(top - bottom);
        width  = cv::norm(p[0] - p[1]);

        tile_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tile_angle = tile_angle / CV_PI * 180.;
    }
    Color color;
    cv::Point2f top, bottom;
    double width, length;
    float tile_angle;  //与y轴的角度
};

struct LightParams  //匹配灯条条件参数
{
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
};
struct ArmourParams  //匹配装甲板条件参数
{
    double min_light_ratio;            //两灯条长度比
    double min_small_center_distance;  //小装甲板最低长宽比
    double max_small_center_distance;  //小装甲板最高长宽比
    double min_large_center_distance;  //大装甲板最低长宽比
    double max_large_center_distance;  //大装甲板最高长宽比
    double max_angle;                  //装甲兵最大与x轴角度
};

struct Armour
{
    Armour() = default;
    Armour(const Light & light1, const Light & light2)
    {
        if (light1.center.x < light2.center.x) {
            left_light = light1, right_light = light2;
        } else {
            left_light = light2, right_light = light1;
        }
        center = (left_light.center + right_light.center) / 2;
    }

    Light left_light, right_light;  //左右灯条
    cv::Point2f center;             //中心点
    RobotType type;                 //机器人类型
    ArmourType armour_type;         //装甲板类型
    cv::Point3f cps;                //Camera Points, 相机坐标系
    cv::Point3f wps;                //World Points, 世界坐标系
    Color color;                    //装甲板颜色
};

struct Detection_pack  //每帧的打包数据结构
{
    cv::Mat img;       //图像
    double timestamp;  //时间戳
    // cv::Point2f * pts; //
    std::vector<Armour> armours;  //装甲板
};

enum class ShootLevel { Level1, Level2, Level3 };

}  // namespace Robot

#endif /*_ROBOT_HPP_*/