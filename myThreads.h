#ifndef _MY_THREADS_HPP_
#define _MY_THREADS_HPP_

//OpenCV
#include <cmath>
#include <functional>
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
#include "modules/kalman_filter/PredictorEKF.hpp"

#include "utils/robot.hpp"
#include "utils/timer/timer.hpp"

//fmt
#include <fmt/color.h>
#include <fmt/core.h>

#include <string>

//多线程
#include <mutex>
#include <thread>

// Assert
#define THREAD_ASSERT_WARNING(expr, info, ...)                                             \
    do {                                                                                   \
        if ((expr) == false) {                                                             \
            fmt::print(fg(fmt::color::orange), "[WARNING] " #expr info "\n"##__VA_ARGS__); \
        }                                                                                  \
    } while (0)

static bool main_loop_condition = true;  //主循环的条件

inline void signalHandler(int signum)  //信号处理函数
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
inline void camera_thread(
    bool & condition, cv::Mat & img, std::mutex & camera_mutex, double & timestamp_ms)
{
    //初始化相机
    Devices::MV_Camera mv_camera{PROJECT_DIR "/Configs/camera/MV-SUA133GC-T-Manifold.config"};
    utils::timer timer{"camera", 1, false};
    cv::Mat read_img;
    //计时
    struct timespec tv_start;  //开始的时间戳
    struct timespec tv_end;    //结束的时间戳
    double a         = 0.99;   //进程运行时间的动量更新超参数
    double temp_time = 0;      //temp
    double this_time = 0;

    //打开相机
    mv_camera.open();
    while (condition) {
        timer.start(0);
        // 读取图片,
        THREAD_ASSERT_WARNING(mv_camera.read(read_img), "读取相机图片失败");
        // if(!read_img.empty()){
        //     cv::resize(read_img, read_img, cv::Size(640, 480));
        // }

        // 对共享的图片上锁
        {
            std::lock_guard<std::mutex> l(camera_mutex);
            img = read_img.clone();  //深拷贝，
        }

        //动量更新
        temp_time = timer.end(0, "read image");
        // (tv_end.tv_sec - tv_start.tv_sec) + (tv_end.tv_nsec - tv_start.tv_nsec) / 1e9;  //s
        this_time = a * this_time + (1 - a) * temp_time;
        //"适当地"休眠一段时间
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // fmt::print("[相机读取] 每次花费时间: {} ms\n", this_time * 1e3);
    }
    mv_camera.close();

}

/**
 * @brief 读取串口线程
 * 
 */
inline void readSerial_thread(Devices::Serial & reader)
{
    while (main_loop_condition) {
        if (!reader.isOpen()) {
            reader.openSerial();
        }
        THREAD_ASSERT_WARNING(reader.readSerial(), "读取串口");

        auto res = reader.getData();
        // fmt::print("[yaw={}, pitch={}, shoot_speed={}]\n", res.yaw, res.pitch, res.shoot_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

inline void sendSerial_thread(Devices::Serial & sender, Devices::SendData & send_data)
{
    THREAD_ASSERT_WARNING(sender.sendData(send_data), "串口发送数据失败!");
}

#endif /*_MY_THREADS_HPP_*/