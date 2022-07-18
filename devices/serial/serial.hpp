#ifndef _SERIAL_HPP_
#define _SERIAL_HPP_

#include "../utils/robot.hpp"

#include <fmt/core.h>
#include <opencv2/core/hal/interface.h>

#include <string>

#include <mutex>
#include <thread>


namespace Devices
{
const int BufferMaxSize = 50;

union float_uchar {
    float f;
    uchar uchars[4];
};

class Serial
{
public:
    explicit Serial(const std::string &, std::mutex& );

    bool sendData(float, float);

    bool sendData(Robot::RobotType, float, float);

    bool noArmour();

    bool openSerial();

    bool reOpen();

    bool closeSerial();

    bool isOpen();

    bool readSerial();

    float yaw();
    float pitch();

    bool sendData(Robot::sendData);

    Robot::receiveData getData();

    Serial(Serial const &) = delete;
    Serial & operator=(Serial const &) = delete;

private:
    uchar send_buffer_[BufferMaxSize];
    uchar read_buffer_[BufferMaxSize];

    float_uchar _yaw, _pitch, _shoot_speed;

    std::string name;
    std::mutex& serial_mutex;

    int fd;  //串口句柄

    uchar frame_header, frame_tail;
};

}  // namespace Devices

#endif /*_SERIAL_HPP_*/