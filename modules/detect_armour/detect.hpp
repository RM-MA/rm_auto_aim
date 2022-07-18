#ifndef _DETECT_HPP_
#define _DETECT_HPP_

#include "../utils/robot.hpp"

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace Modules
{
class Detect
{
public:
    Detect(const Robot::Color & color);
    ~Detect();
    bool detect(Robot::Detection_pack &);

    Detect(Detect const &) = delete;
    Detect & operator=(Detect const &) = delete;

private:
    bool process(cv::Mat &, cv::Mat &);

    bool match_Lights(const cv::Mat &, const cv::Mat &, std::vector<Robot::Light> &);

    bool match_Armours(const std::vector<Robot::Light> &, std::vector<Robot::Armour> &);

    bool isLight(const Robot::Light &);

    bool containLight(
        const Robot::Light &, const Robot::Light &, const std::vector<Robot::Light> &);

    bool isArmour(Robot::Armour &);

    const Robot::Color color;//敌方颜色

    cv::Scalar lowerb, upperb;          //二值化的高低阈值
    
    Robot::LightParams light_params;    //灯条条件参数
    Robot::ArmourParams armour_params;  //装甲板条件参数
};  //Detect

}  // namespace Modules

#endif /*_DETECT_HPP_*/