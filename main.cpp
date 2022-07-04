#include "opencv2/opencv.hpp"
#include "utils/logger/logger.hpp"

#include <csignal> //信号处理
#include <fcntl.h>
#include <unistd.h> //信号处理

#include "devices/camera/mv_camera.hpp"

#include <fmt/color.h>
#include <fmt/core.h>

#include <string>

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

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);

    test s{"a"};
    int  i = 1;

    logger::logger my_logger{"i={}, j={:.3f},{:.3f},{}\n", 
    logger::LOGGER_TYPE::ALL, PROJECT_DIR "/test.csv", true, "", O_WRONLY | O_CREAT | O_APPEND};

    devices::MV_Camera mv_camera{PROJECT_DIR "/configs/camera/MV-SUA133GC-T_042003320218.Config"};

    mv_camera.open();

    cv::Mat img;
    double  timestamp_ms; //单位0.1毫秒

    while (main_loop_condition)
    {
        mv_camera.read(img, timestamp_ms);
        my_logger.write(i, i * 2.1, timestamp_ms, PROJECT_DIR);
        // Print("1", 2);
        i++;
        // sleep(1);
    }
    my_logger.close();
    return 0;
}