//OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
//信号处理
#include <unistd.h>  //信号处理
#include <csignal>   //信号处理
//计时
#include <chrono>
#include <ctime>
//自
#include "devices/camera/mv_camera.hpp"
#include "devices/serial/serial.hpp"

#include "modules/detect_armour/detect.hpp"
#include "modules/posture_calculating/posture_calculating.hpp"

#include "utils/logger/logger.hpp"
#include "utils/robot.hpp"
#include "utils/timer/timer.hpp"

//fmt
#include <fmt/color.h>
#include <fmt/core.h>

#include <string>

//多线程
#include <mutex>
#include <thread>

bool main_loop_condition = true;  //主循环的条件

void signalHandler(int signum)  //信号处理函数
{
    fmt::print(fg(fmt::color::red) | fmt::emphasis::bold, "正在退出,请等待释放资源!\n");
    main_loop_condition = false;
}

/**
 * @brief 相机处理线程
 * 为了节省时间和内存，根据主线程的时间、相机读取的时间，可以进行一段时间的休眠。
 * 因为两个时间是变化的，传入引用，采取动量更新。
 * 动量更新：t_i = (1 - a)*t_i + a*t_i-1, 超参数a
 */
void camera_thread(
    bool & condition, cv::Mat & img, std::mutex & img_mutex, bool & camera_start,
    double & timestamp_ms)
{
    //初始化相机
    Devices::MV_Camera mv_camera{PROJECT_DIR "/Configs/camera/MV-SUA133GC.config"};
    utils::timer timer{"camera", 1, false};
    cv::Mat img_cpoy;
    //计时
    struct timespec tv_start;  //开始的时间戳
    struct timespec tv_end;    //结束的时间戳
    double a         = 0.99;   //进程运行时间的动量更新超参数
    double temp_time = 0;      //temp
    double this_time = 0;

    //打开相机
    mv_camera.open();
    while (condition) {
        //计时
        // clock_gettime(CLOCK_REALTIME, &tv_start);
        timer.start(0);
        // auto startTime = std::chrono::system_clock::now();
        //读取图片, 并对图片上锁
        mv_camera.read(img_cpoy, timestamp_ms);
        {
            std::lock_guard<std::mutex> l(img_mutex);  //上锁
            img = img_cpoy.clone();                    //深拷贝，
            if (!camera_start) {
                camera_start = true;
            }
        }
        //锁超出作用域，解锁
        //结束计时，更新图片相机的时间
        clock_gettime(CLOCK_REALTIME, &tv_end);
        // auto endTime = std::chrono::system_clock::now();
        // auto wasteTime =
        //     std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
        // fmt::print("[相机读取] 每次花费时间: {}\n", wasteTime);
        //动量更新
        temp_time = timer.end(0, "read image");
        // (tv_end.tv_sec - tv_start.tv_sec) + (tv_end.tv_nsec - tv_start.tv_nsec) / 1e9;  //s
        this_time = a * this_time + (1 - a) * temp_time;
        //"适当地"休眠一段时间
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // fmt::print("[相机读取] 每次花费时间: {} ms\n", this_time * 1e3);
    }
    mv_camera.close();
    camera_start = false;
}

int main(int argc, char ** argv)
{
    //注册信号量，处理 Ctrl + C 中断
    signal(SIGINT, signalHandler);

    //std::ref 输出引用
    // test s{"a"};
    // std::thread th{testForClass, std::ref(main_loop_condition)};
    // th.detach();

    bool camera_start = false;
    std::mutex img_mutex;
    std::mutex serial_mutex;

    Robot::Color color = Robot::Color::BLUE;

    if(argc == 2){
        if(argv[1][0] - '0' == 1){//1 红
            color = Robot::Color::RED;
        }else if(argv[1][0] - '0' == 0){//0 蓝
            color = Robot::Color::BLUE;
        }
    }

    logger::logger my_logger{
        "i={}, j={},{:.3f}\n",        logger::LOGGER_TYPE::ALL, PROJECT_DIR "/test.csv", true, "",
        O_WRONLY | O_CREAT | O_APPEND};
    utils::timer timer{"main", 10};

    Modules::Detect detector{color};
    Modules::Posture_Calculating solver{};

    Devices::Serial serial{"/dev/ttyACM0", serial_mutex};

    int frame = 1;  //主线程的帧数

    cv::Mat img;
    cv::Mat copyImg;
    double timestamp_ms;  //曝光时间, 单位us

    //相机进程
    std::thread cameraThread{camera_thread,          std::ref(main_loop_condition),
                             std::ref(img),          std::ref(img_mutex),
                             std::ref(camera_start), std::ref(timestamp_ms)};
    cameraThread.detach();
    //计时
    struct timespec tv_start;        //开始的时间戳
    struct timespec tv_end;          //结束的时间戳
    double a                = 0.99;  //线程运行时间的动量更新超参数
    double temp_time        = 0;     //temp
    double main_thread_time = 0;     //主线程的时间

    auto startTime = std::chrono::system_clock::now();
    serial.openSerial();

    for (; main_loop_condition; frame++) {
        if (camera_start) {
            //计时
            // timer.start(1);
            {  //上锁
                std::lock_guard<std::mutex> l(img_mutex);
                copyImg = img.clone();
            }
            // timer.end(1, "lock");

            // timer.start(1);
            auto endTime = std::chrono::system_clock::now();
            auto wasteTime =
                std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() /
                1e3;

            Robot::Detection_pack detection_pack{copyImg, wasteTime};
            // detection_pack.img       = copyImg;
            // detection_pack.timestamp = wasteTime;
            // timer.end(1, "init detection_pack");

            // timer.start(0);
            detector.detect(detection_pack);
            // timer.end(0, "detect");
            bool solve_success = solver.solve(detection_pack.armours);
            if (solve_success) {
                serial.sendData(
                    detection_pack.armours.front().center.x,
                    detection_pack.armours.front().center.x);
            } else {
                serial.noArmour();
            }
            Robot::drawArmours(detection_pack.armours, copyImg, color);

            // timer.start(0);
            cv::imshow("after_draw", copyImg);
            cv::waitKey(1);  //waitKey(1) == waitKey(20)
            // timer.end(0, "imshow");

        } else {  //相机线程未开始
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            fmt::print(fg(fmt::color::red), "[WARN] 未开始！");
        }
    }

    my_logger.close();

    while (camera_start)  //当其他线程都释放完后，退出主线程
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
    fmt::print(fg(fmt::color::red), "end! \n");
    return 0;
}