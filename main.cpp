#include "opencv2/opencv.hpp"
#include "utils/logger/logger.hpp"

#include <bits/types/struct_timeval.h>
#include <fcntl.h>
#include <unistd.h>  //信号处理
#include <chrono>
#include <csignal>  //信号处理
#include <opencv2/highgui.hpp>

#include "devices/camera/mv_camera.hpp"

#include <fmt/color.h>
#include <fmt/core.h>

#include <string>

#include <mutex>
#include <thread>

bool main_loop_condition = true;  //主循环的条件

class test
{  //测试析构函数会不会运行
public:
    test(const std::string & s_) : s(s_)
    {
    }
    ~test()
    {
        fmt::print("test {}释放了~~~\n", s);
    };

private:
    std::string s;
};

void signalHandler(int signum)  //信号处理函数
{
    fmt::print(fg(fmt::color::red) | fmt::emphasis::bold, "正在退出,请等待释放资源!\n");
    main_loop_condition = false;
}

//可变参数列表
template <typename... T> void Print(T &&... args)
{
    fmt::print("{} {} \n", args...);
}

void testForClass(bool & condition)
{
    test s{"b"};
    int i = 0;
    while (condition) {
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
void camera_thread(
    bool & condition, cv::Mat & img, std::mutex & img_mutex, bool & camera_start,
    double & timestamp_ms)
{
    //初始化相机
    devices::MV_Camera mv_camera{PROJECT_DIR "/Configs/camera/MV-SUA133GC.config"};

	double a = 0.99;//控制动量更新的超参数
    cv::Mat img_cpoy;
    double this_time = 0, temp_time = 0;//单位: s
	//开始、结束的时间戳
    struct timeval tv_start;
	struct timeval tv_end;
    // double timestamp_ms;  //

    //打开相机
    mv_camera.open();
    while (condition) {
        //计时
		gettimeofday(&tv_start, nullptr);
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
		gettimeofday(&tv_end, nullptr);
        // auto endTime = std::chrono::system_clock::now();
        // auto wasteTime =
        //     std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
        // fmt::print("[相机读取] 每次花费时间: {}\n", wasteTime);
		//动量更新
		temp_time = (tv_end.tv_sec - tv_start.tv_sec) + (tv_end.tv_usec - tv_start.tv_usec) / 1e6;
		this_time = (1 - a) * this_time + a * temp_time;
        //"适当地"休眠一段时间
		fmt::print("[相机读取] 每次花费时间: {} ms\n", this_time*1e3);
    }
}

int main(int argc, char ** argv)
{
    //注册信号量，处理 Ctrl + C 中断
    signal(SIGINT, signalHandler);

    // test s{"a"};
    //std::ref 输出引用
    // std::thread th{testForClass, std::ref(main_loop_condition)};
    // th.detach();

    bool camera_start = false;
    std::mutex img_mutex;

    logger::logger my_logger{
        "i={}, j={},{:.3f},{}\n",     logger::LOGGER_TYPE::ALL, PROJECT_DIR "/test.csv", true, "",
        O_WRONLY | O_CREAT | O_APPEND};

    int frame = 1;  //主线程的帧数

    cv::Mat img;
    cv::Mat copyImg;
    double timestamp_ms;  //曝光时间, 单位us

    std::thread cameraThread{camera_thread,          std::ref(main_loop_condition),
                             std::ref(img),          std::ref(img_mutex),
                             std::ref(camera_start), std::ref(timestamp_ms)};
    cameraThread.detach();

    while (main_loop_condition) {
        if (camera_start) {
            //计时
            // auto startTime = std::chrono::system_clock::now();
			
            {
                std::lock_guard<std::mutex> l(img_mutex);
                copyImg = img.clone();
            }

            cv::imshow("原图", copyImg);
            my_logger.write(img.rows, img.cols, timestamp_ms, PROJECT_DIR);
            cv::waitKey(1);
            // auto endTime = std::chrono::system_clock::now();
            // auto wasteTime =
            // std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
            // fmt::print("[主线程] 每次花费时间: {}\n", wasteTime);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            fmt::print(fg(fmt::color::red), "[WARN] 未开始！");
        }
        // sleep(1);
        frame++;
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    my_logger.close();
    return 0;
}