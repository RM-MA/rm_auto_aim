#include <fmt/core.h>

#include <iostream>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <string>

#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"

int main(int argc, char **argv)
{
    std::string     path = "test.yml";
    cv::FileStorage a("./test.yaml", cv::FileStorage::READ);

    bool        record;
    cv::Size    s;
    cv::Mat     mat;
    std::string name;

    a["Record"] >> record;
    a["imageSize"] >> s;
    a["cameraMatrix"] >> mat;
    a["logger"]["name"] >> name;

    fmt::print("{}, [{}, {}], {}\n", record, s.width, s.height, name);
    std::cout << mat << std::endl;

    return 0;
}