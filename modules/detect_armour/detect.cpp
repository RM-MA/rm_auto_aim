#include "detect.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include <iostream>
#include "robot.hpp"

namespace Modules
{
Detect::Detect(const Robot::Color & color) : color(color)
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
    //阈值
    if (color == Robot::Color::RED) {
        file["Thresholds"]["red"]["lowerb"] >> lowerb;
        file["Thresholds"]["red"]["upperb"] >> upperb;
    } else {
        file["Thresholds"]["blue"]["lowerb"] >> lowerb;
        file["Thresholds"]["blue"]["upperb"] >> upperb;
    }
    //灯条匹配条件参数
    file["light"]["min_ratio"] >> light_params.min_ratio;
    file["light"]["max_ratio"] >> light_params.max_ratio;
    file["light"]["max_angle"] >> light_params.max_angle;
    //装甲板匹配条件参数
    file["armour"]["max_angle"] >> armour_params.max_angle;
    file["armour"]["min_light_ratio"] >> armour_params.min_light_ratio;
    file["armour"]["min_small_center_distance"] >> armour_params.min_small_center_distance;
    file["armour"]["max_small_center_distance"] >> armour_params.max_small_center_distance;
    file["armour"]["min_large_center_distance"] >> armour_params.min_large_center_distance;
    file["armour"]["max_large_center_distance"] >> armour_params.max_large_center_distance;
    // std::cout << lowerb << "\n" << upperb << std::endl;
}

bool Detect::detect(Robot::Detection_pack & detection_pack)
{
    //变量的初始化
    cv::Mat processImage;

    // auto & armours = detection_pack.armours;
    // auto & img     = detection_pack.img;

    std::vector<Robot::Light> lights;

    //对图像进行处理
    process(detection_pack.img, processImage);
    cv::imshow("processImage", processImage);

    //匹配灯条和装甲板
    match_Lights(detection_pack.img, processImage, lights);
    match_Armours(lights, detection_pack.armours);

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

bool Detect::match_Lights(
    const cv::Mat & bgr_img, const cv::Mat & binary_img, std::vector<Robot::Light> & lights)
{
    //寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(
        binary_img, contours,
        cv::RETR_EXTERNAL,  // 只检索最外层轮廓
        cv::CHAIN_APPROX_SIMPLE);

    for (const auto & contour : contours) {
        //拟合旋转矩形
        auto r_rect = cv::minAreaRect(contour);
        //匹配灯条
        Robot::Light light = Robot::Light(r_rect);
        //按条件判断
        if (!isLight(light)) {
            continue;
        }
        //选出ROI
        auto rect = light.boundingRect();  //rect.x, rect.y为左上点
        //防止ROI超出图片边界
        if (!(rect.x >= 0 && rect.width >= 0 && rect.x + rect.width <= bgr_img.cols &&
              rect.y >= 0 && rect.height >= 0 && rect.y + rect.height <= bgr_img.rows)) {
            continue;
        }
        auto roi = bgr_img(rect);
        //根据ROI内r、b的像素点数量 判断灯条颜色
        int sum_r = 0, sum_b = 0;
        for (int i = 0; i < roi.cols; i++) {
            for (int j = 0; j < roi.rows; j++) {
                //测试是否在轮廓内
                if (cv::pointPolygonTest(contour, cv::Point2f(i + rect.x, j + rect.y), false) >=
                    0) {
                    sum_r += roi.at<cv::Vec3b>(i, j)[2];
                    sum_b += roi.at<cv::Vec3b>(i, j)[0];
                }
            }
        }
        light.color = sum_b > sum_r ? Robot::Color::BLUE : Robot::Color::RED;

        lights.emplace_back(light);
    }

    return true;
}

bool Detect::isLight(const Robot::Light & light)
{
    //灯条长宽比
    float ratio          = light.width / light.length;  //短 / 长
    bool ratio_condition = light_params.min_ratio < ratio && ratio < light_params.max_ratio;
    //灯条倾斜角
    bool angle_condition = light.tile_angle < light_params.max_angle;

    bool is_light = ratio_condition && angle_condition;
    return is_light;
}

bool Detect::match_Armours(
    const std::vector<Robot::Light> & lights, std::vector<Robot::Armour> & armours)
{
    // std::vector<Robot::Armour> red_armours;
    // std::vector<Robot::Armour> blue_armours;
    for (int i = 0; i < lights.size(); i++) {
        for (int j = i + 1; j < lights.size(); j++) {
            auto & light1 = lights[i];
            auto & light2 = lights[j];
            if (light1.color != color || light2.color != color) {
                continue;
            }
            if (containLight(light1, light2, lights)) {
                continue;
            }
            auto armour = Robot::Armour(light1, light2);
            if (isArmour(armour)) {
                armours.emplace_back(armour);
            }
        }
    }
    return true;
}

//判断两个灯条之间是否存在其他灯条
bool Detect::containLight(
    const Robot::Light & light1, const Robot::Light & light2,
    const std::vector<Robot::Light> & lights)
{
    auto points = std::vector<cv::Point2f>{light1.top, light1.bottom, light2.top, light2.bottom};
    auto bounding_rect = cv::boundingRect(points);

    for (const auto & test_light : lights) {
        if (test_light.center == light1.center || test_light.center == light2.center) {
            continue;
        }

        if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
            bounding_rect.contains(test_light.center)) {
            return true;
        }
    }
    return false;
}

bool Detect::isArmour(Robot::Armour & armour)
{
    auto ligth_1 = armour.left_light;
    auto light_2 = armour.right_light;
    //两灯条的长度比 -- (0, 1]: 短 / 长, 避免两灯条长度差别很大
    float light_length_ratio = ligth_1.length < light_2.length ? ligth_1.length / light_2.length
                                                               : light_2.length / ligth_1.length;

    bool ratio_condition = light_length_ratio > armour_params.min_light_ratio;

    //两灯条中心点的距离
    float avg_light_length = (ligth_1.length + light_2.length) / 2;
    //长边 / 短边
    float center_distance = cv::norm(ligth_1.center - light_2.center) / avg_light_length;
    //装甲板的长宽比
    bool center_distance_condition = (armour_params.min_small_center_distance < center_distance &&
                                      center_distance < armour_params.max_small_center_distance) ||
                                     (armour_params.min_large_center_distance < center_distance &&
                                      center_distance < armour_params.max_large_center_distance);
    //判断大小装甲板
    armour.armour_type = center_distance > armour_params.min_large_center_distance
                             ? Robot::ArmourType::Big
                             : Robot::ArmourType::Small;
    // fmt::print("w:h = {}\n", center_distance);
    //匹配出的装甲板的与x轴的角度
    cv::Point2f diff = ligth_1.center - light_2.center;
    float angle      = std::abs(std::atan(diff.y / diff.x));

    bool angle_condition = angle < armour_params.max_angle;

    //最终
    bool is_armour = ratio_condition && center_distance_condition && angle_condition;
    return is_armour;
}

Detect::~Detect()
{
}

}  // namespace Modules