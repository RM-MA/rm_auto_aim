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
    frame_header = 0xff;  //帧头
    frame_tail   = 0xfe;  //帧尾, 设为0x0A, 在串口调试助手中可以换行
}

bool Serial::openSerial()
{
    if (isOpen()) {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::blink, "串口{:^15}已打开!!\n", name);
        return false;
    }
    //互斥信号量
    std::lock_guard<std::mutex> l(serial_mutex);
    auto terminal_command = fmt::format("echo {}| sudo -S chmod 777 {}", "dji", name);
    int message           = system(terminal_command.c_str());
    if (message < 0) {
        fmt::print(
            fg(fmt::color::red) | fmt::emphasis::bold, "执行命令{}, 失败\n", terminal_command);
    }
    fd = open(name.c_str(), O_RDWR | O_NOCTTY);  //加O_NDELAY，为非阻塞式读取

    if (fd == -1) {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::blink, "打开串口{:^15}失败!!\n", name);
        return false;
    }
    struct termios options;  // termios为类型名的结构
    tcgetattr(fd, &options);
    // 波特率460800, 8N1
    options.c_cflag = B460800 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    //当设置为 阻塞模式时生效
    options.c_cc[VTIME] = 0;           //最少读取字符数
    options.c_cc[VMIN]  = 1;           //超时时间, 单位: 100ms
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
    10: 
    11: 帧尾 0x0d
    */
    _yaw.f = send_yaw, _pitch.f = send_pitch;
    send_buffer_[0] = frame_header;
    send_buffer_[10] = frame_tail;
    for (int i = 0; i < 4; i++) {
        send_buffer_[2 + i] = _yaw.uchars[i];
        send_buffer_[6 + i] = _pitch.uchars[i];
    }
    send_buffer_[1] = 1;
    //互斥信号量
    std::lock_guard<std::mutex> l(serial_mutex);
    if (11 == write(fd, send_buffer_, 11)) {
        printf("send buffer = ");
        for(int i =0; i < 11; i++){
            printf("%hhu ", send_buffer_[i]);
        }
        printf("\n");
        return true;
    } else {
        return false;
    }
}

bool Serial::sendData(Robot::sendData & send_data)
{
    return sendData(send_data.yaw, send_data.pitch);
}

Robot::receiveData Serial::getData()
{
    std::lock_guard<std::mutex> l(serial_mutex);  //
    Robot::receiveData data{_yaw.f, _pitch.f, _shoot_speed.f};
    return data;
}
bool Serial::noArmour()
{
    return sendData(0, 0);
}

bool Serial::readSerial()
{
    /*
    1. 读取帧头
    2. 读取其余并判断帧尾是否符合
    */

    /*
    0: 0x0a 帧头
    1-4: yaw 偏移角度
    5-8: pitch 偏移角度
    9-12: shoot_speed 射速
    13: 0x0d 帧尾
    */
    std::lock_guard<std::mutex> l(serial_mutex);  //
    //head
    if (read(fd, read_buffer_, 1) != 1) {
        // 读取不到帧头
        // 当为阻塞模式时, 为超时
        fmt::print("no frame head\n");
        return false;
    }
    if (read_buffer_[0] != frame_header) {
        // 帧头验证失败
        fmt::print("frame head fault\n");
        return false;
    }
    if (read(fd, read_buffer_ + 1, 13) != 13) {
        // 读取剩余内容失败
        fmt::print("no frame tail\n");
        return false;
    }
    if (read_buffer_[13] != frame_tail) {
        // 帧尾验证失败
        fmt::print("frame tail fault\n");
        return false;
    }
    // 赋值
    for (int i = 0; i < 4; i++) {
        _yaw.uchars[i]         = read_buffer_[1 + i];
        _pitch.uchars[i]       = read_buffer_[5 + i];
        _shoot_speed.uchars[i] = read_buffer_[9 + i];
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(5));  //sleep 5ms
    return true;
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