#include "serial.hpp"

#include <fcntl.h>  //文件控制选项头文件
#include <fmt/color.h>
#include <stdio.h>
#include <termios.h>  //linux串口相关的头文件
#include <unistd.h>  //Linux/Unix系统中内置头文件，包含了许多系统服务的函数原型
#include <chrono>
#include <mutex>
#include <thread>

namespace Devices
{
Serial::Serial(const std::string & name, std::mutex & mutex)
: name(name), fd(-1), serial_mutex(mutex)
{
}

bool Serial::openSerial()
{
    if (isOpen()) {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::blink, "串口{:^15}已打开!!\n");
        return false;
    }
    //互斥信号量
    std::lock_guard<std::mutex> l(serial_mutex);
    fd = open(name.c_str(), O_RDWR | O_NOCTTY);  //加O_NDELAY，为非阻塞式读取

    if (fd == -1) {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::blink, "打开串口{:^15}失败!!\n");
        return false;
    }
    struct termios options;  // termios为类型名的结构
    tcgetattr(fd, &options);
    // 波特率115200, 8N1
    options.c_cflag     = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag     = IGNPAR;
    options.c_oflag     = 0;
    options.c_lflag     = 0;
    //当设置为 阻塞模式时生效
    options.c_cc[VTIME] = 0;//最少读取字符数
    options.c_cc[VMIN]  = 1;//超时时间, 单位: 100ms
    tcflush(fd, TCIFLUSH);             //清除缓冲区
    tcsetattr(fd, TCSANOW, &options);  //应用上面的设置

    return true;
}

bool Serial::reOpen()
{
    if (isOpen() || closeSerial()) {
        return false;
    }
    return openSerial();
}

bool Serial::closeSerial()
{
    //互斥信号量
    std::lock_guard<std::mutex> l(serial_mutex);
    int message = close(fd);
    if (message < 0) {
        return false;
    }
    fd = -1;
    return true;
}

bool Serial::isOpen()
{
    return fd > 0;
}

bool Serial::sendData(float send_yaw, float send_pitch)
{
    /*send_buffer_
    0: 帧头 0x0a
    1: 有无目标
    2-5: yaw
    6-9: pitch
    10: 帧尾 0x0d
    */
    _yaw.f = send_yaw, _pitch.f = send_pitch;
    send_buffer_[0] = 0x0a;
    send_buffer_[1] = 0x01;
    for (int i = 0; i < 4; i++) {
        send_buffer_[2 + i] = _yaw.uchars[i];
    }
    for (int i = 0; i < 4; i++) {
        send_buffer_[6 + i] = _pitch.uchars[i];
    }

    //互斥信号量
    std::lock_guard<std::mutex> l(serial_mutex);
    if (11 == write(fd, send_buffer_, 11)) {
        return true;
    } else {
        return false;
    }
}

bool Serial::readSerial()
{
    /*
    1. 读取帧头
    2. 读取其余并判断帧尾是否符合
    */
    bool read_flag = false;
    std::lock_guard<std::mutex> l(serial_mutex);  //
    //head
    if (1 == read(fd, read_buffer_, 1)) {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  //sleep 5ms
    return read_flag;
}

float Serial::yaw()
{
    return _yaw.f;
}
float Serial::pitch()
{
    return _pitch.f;
}

}  // namespace Devices