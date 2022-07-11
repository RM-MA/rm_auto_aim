#include "detect.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include <iostream>

namespace Modules
{
Detect::Detect()
{
    /*
    //写入测试
    lowerb = cv::Scalar(0, 0, 0);
    upperb = cv::Scalar(255, 255, 255);
    cv::FileStorage write_file(PROJECT_DIR "/Configs/detect/test.yaml", cv::FileStorage::WRITE);
    write_file << "lowerb" << lowerb;
    write_file << "upperb" << upperb;
    */
    //从文件读取配置参数
    cv::FileStorage file(PROJECT_DIR "/Configs/detect/detect.yaml", cv::FileStorage::READ);
    file["Thresholds"]["lowerb"] >> lowerb;
    file["Thresholds"]["upperb"] >> upperb;

    std::cout << lowerb << "\n" << upperb << std::endl;
}

bool Detect::detect(Robot::Detection_pack & detection_pack)
{
    //变量的初始化
    cv::Mat  processImage;

    auto & armours = detection_pack.armours;
    auto & img = detection_pack.img;

    std::vector<Robot::Light> lights;

    //对图像进行处理
    process(img, processImage);
    cv::imshow("processImage", processImage);

    //匹配灯条和装甲板
    match_Lights(processImage, lights);
    match_Armours(lights, armours);

    return true;
}

bool Detect::process(cv::Mat & img, cv::Mat & processImage)
{
    cv::Mat tempImage;

    //颜色空间的转化
    cv::cvtColor(img, tempImage, cv::COLOR_BGR2HSV);
    //二值化, 在高低阈值内值为1(白色), 在阈值外则为0(黑色)
    cv::inRange(tempImage, lowerb, upperb, processImage);

    return true;
}

bool Detect::match_Lights(cv::Mat & img, std::vector<Robot::Light> & lights)
{
    return true;
}

bool Detect::match_Armours(std::vector<Robot::Light> & lights, std::vector<Robot::Armour> & armours)
{
    return true;
}

Detect::~Detect()
{
}

}  // namespace Modules