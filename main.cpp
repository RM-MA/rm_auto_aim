//自
#include "devices/camera/mv_camera.hpp"
#include "devices/serial/serial.hpp"

#include "modules/detect_armour/detect.hpp"
#include "modules/kalman_filter/PredictorEKF.hpp"

#include "utils/logger/logger.hpp"
#include "utils/robot.hpp"
#include "utils/timer/timer.hpp"

#include "myThreads.h"

#include <string>

int main(int argc, char ** argv)
{
    //注册信号量，处理 Ctrl + C 中断
    signal(SIGINT, signalHandler);

    // 控制量声明
    bool camera_start = false;
    std::mutex camera_mutex;
    std::mutex serial_mutex;

    // 确定敌方颜色
    Robot::Color color = Robot::Color::BLUE;
    if (argc == 2) {
        if (argv[1][0] - '0' == 1) {  //1 红
            color = Robot::Color::RED;
        } else if (argv[1][0] - '0' == 0) {  //0 蓝
            color = Robot::Color::BLUE;
        }
    }

    // 模块初始化
    logger::logger my_logger{
        "i={}, j={},{:.3f}\n",        logger::LOGGER_TYPE::ALL, PROJECT_DIR "/test.csv", true, "",
        O_WRONLY | O_CREAT | O_APPEND};
    utils::timer timer{"main", 10};

    Modules::Detect detector{color};
    // Modules::Posture_Calculating solver{};
    Modules::PredictorEKF predictor{};

    Devices::Serial serial{"/dev/ttyTHS2", serial_mutex};

    int frame = 1;  //主线程的帧数

    cv::Mat img;
    cv::Mat copyImg;
    double timestamp_ms;  //曝光时间, 单位us

    //相机进程
    std::thread cameraThread{
        camera_thread, std::ref(main_loop_condition), std::ref(img), std::ref(camera_mutex),
        std::ref(timestamp_ms)};
    std::thread readSerialThread{readSerial_thread, std::ref(serial)};

    cameraThread.detach();
    readSerialThread.detach();
    //计时
    struct timespec tv_start;        //开始的时间戳
    struct timespec tv_end;          //结束的时间戳
    double a                = 0.99;  //线程运行时间的动量更新超参数
    double temp_time        = 0;     //temp
    double main_thread_time = 0;     //主线程的时间

    auto startTime = std::chrono::system_clock::now();
    serial.openSerial();

    for (; main_loop_condition; frame++) {
        //计时
        // timer.start(1);
        {  //上锁
            if(img.empty()){
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            std::lock_guard<std::mutex> l(camera_mutex);
            copyImg = img.clone();
        }
        
        // timer.start(1);
        auto endTime = std::chrono::system_clock::now();
        auto wasteTime =
            std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() /
            1e6;//单位 s

        cv::Mat showimg   = img;
        auto receive_data = serial.getData();

        Robot::Detection_pack detection_pack{copyImg, wasteTime};
        // timer.end(1, "init detection_pack");

        // timer.start(0);
        detector.detect(detection_pack);
        // timer.end(0, "detect");

        Devices::SendData send_data{};
        predictor.predict(detection_pack, receive_data, send_data, showimg);

        std::thread sendSerialThread{sendSerial_thread, std::ref(serial), std::ref(send_data)};
        sendSerialThread.detach();
        // draw
        Robot::drawArmours(detection_pack.armours, showimg, color);

        // timer.start(0);
        cv::imshow("after_draw", showimg);
        cv::waitKey(1);  //waitKey(1) == waitKey(20)

        // timer.end(0, "imshow");
    }

    my_logger.close();

    fmt::print(fg(fmt::color::red), "end! wait for 5s\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    return 0;
}