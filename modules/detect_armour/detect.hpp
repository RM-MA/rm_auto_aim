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
    Detect();
    ~Detect();
    bool detect(Robot::Detection_pack &);

    Detect(Detect const &) = delete;
    Detect & operator=(Detect const &) = delete;

private:
    bool process(cv::Mat&, cv::Mat&);

    bool match_Lights(cv::Mat &, std::vector<Robot::Light> &);

    bool match_Armours(std::vector<Robot::Light> &, std::vector<Robot::Armour> &);


    cv::Scalar lowerb, upperb;//二值化的高低阈值
};  //Detect

}  // namespace Modules

#endif /*_DETECT_HPP_*/