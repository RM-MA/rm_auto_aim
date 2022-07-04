#include "opencv2/opencv.hpp"
#include "utils/logger/logger.hpp"

#include <chrono>
#include <csignal> //信号处理
#include <fcntl.h>
#include <opencv2/highgui.hpp>
#include <unistd.h> //信号处理

#include "devices/camera/mv_camera.hpp"

#include <fmt/color.h>
#include <fmt/core.h>

#include <string>

#include <mutex>
#include <thread>

bool main_loop_condition = true; //主循环的条件

class test
{ //测试析构函数会不会运行
public:
    test(const std::string &s_) : s(s_)
    {
    }
    ~test()
    {
        fmt::print("test {}释放了~~~\n", s);
    };

private:
    std::string s;
};

void signalHandler(int signum) //信号处理函数
{
    fmt::print(fg(fmt::color::red) | fmt::emphasis::bold, "正在退出,请等待释放资源!\n");
    main_loop_condition = false;
}

//可变参数列表
template <typename... T> void Print(T &&... args)
{
    fmt::print("{} {} \n", args...);
}

void testForClass(bool &condition)
{
    test s{"b"};
    int  i = 0;
    while (condition)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // fmt::print(fg(fmt::color::green), "B\n");
        i++;
    }
    fmt::print(fg(fmt::color::green), "{}\n", i);
}

/**
 * @brief 相机处理线程
 * 为了节省时间和内存，根据主线程的时间、相机读取的时间，可以进行一段时间的休眠。
 * 因为两个时间是变化的，传入引用，采取动量更新。
 * 动量更新：t_i = (1 - a)*t_i + a*t_i-1, 超参数a
 */
void camera_thread(bool &condition, cv::Mat &img, std::mutex &img_mutex)
{
    //初始化相机
    devices::MV_Camera mv_camera{PROJECT_DIR "/configs/camera/MV-SUA133GC-T_042003320218.Config"};

    cv::Mat img_cpoy;
    double  this_time = 0;
    double  timestamp_ms; //单位0.1毫秒

    //打开相机
    mv_camera.open();
    while (condition)
    {
        //计时
        // auto start = std::chrono::
        //读取图片, 并对图片上锁
        mv_camera.read(img_cpoy, timestamp_ms);
        {
            std::lock_guard<std::mutex> l(img_mutex); //上锁
            img = img_cpoy.clone();                   //深拷贝，
        }
        //锁超出作用域，解锁

        //结束计时，更新图片相机的时间

        //"适当地"休眠一段时间
    }
}

int main(int argc, char **argv)
{
    //注册信号量，处理 Ctrl + C 中断
    signal(SIGINT, signalHandler);

    test s{"a"};
    int  i = 1;
    //std::ref 输出引用
    std::thread th{testForClass, std::ref(main_loop_condition)};
    th.detach();

    std::mutex img_mutex;

    logger::logger my_logger{"i={}, j={:.3f},{:.3f},{}\n", logger::LOGGER_TYPE::ALL, PROJECT_DIR "/test.csv", true, "", O_WRONLY | O_CREAT | O_APPEND};

    // devices::MV_Camera mv_camera{PROJECT_DIR "/configs/camera/MV-SUA133GC-T_042003320218.Config"};

    // mv_camera.open();
    cv::Mat img;
    double  timestamp_ms; //单位0.1毫秒

    while (main_loop_condition)
    {
        // mv_camera.read(img, timestamp_ms);
        my_logger.write(i, i * 2.1, timestamp_ms, PROJECT_DIR);
        // Print("1", 2);
        cv::imshow("原图", img);

        // sleep(1);
        cv::waitKey(1);
        i++;
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    my_logger.close();
    return 0;
}