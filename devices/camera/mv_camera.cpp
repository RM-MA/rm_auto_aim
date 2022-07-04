#include "mv_camera.hpp"
#include "camera/MVSDK/CameraApi.h"
#include <fmt/color.h>

namespace devices
{

// Assert
#define MV_ASSERT_WARNING(expr, info, ...)                                                                                                                                                             \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        if ((expr) == false)                                                                                                                                                                           \
        {                                                                                                                                                                                              \
            fmt::print(fg(fmt::color::orange), "[WARNING] " #expr info "\n"##__VA_ARGS__);                                                                                                             \
        }                                                                                                                                                                                              \
    } while (0)

#define MV_ASSERT_ERROR(expr, info, ...)                                                                                                                                                               \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        if ((expr) == false)                                                                                                                                                                           \
        {                                                                                                                                                                                              \
            fmt::print(fg(fmt::color::red), "[ERROR] " #expr info "\n", ##__VA_ARGS__);                                                                                                                \
            return false;                                                                                                                                                                              \
        }                                                                                                                                                                                              \
    } while (0)

#define MV_ASSERT_THROW(expr, info, ...)                                                                                                                                                               \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        if ((expr) == false)                                                                                                                                                                           \
        {                                                                                                                                                                                              \
            throw MindVision_FrameError(fmt::format("'" #expr "' = ({}) " info, status, ##__VA_ARGS__));                                                                                               \
        }                                                                                                                                                                                              \
    } while (0)

// check_api
#define MV_CHECK_API_WARNING(expr, info, ...)                                                                                                                                                          \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        auto status = (expr);                                                                                                                                                                          \
        if (status != CAMERA_STATUS_SUCCESS)                                                                                                                                                           \
        {                                                                                                                                                                                              \
            fmt::print(fg(fmt::color::orange), "[WARNING] '" #expr "' = ({}) " info, status, ##__VA_ARGS__);                                                                                           \
        }                                                                                                                                                                                              \
    } while (0)

#define MV_CHECK_API_ERROR(expr, info, ...)                                                                                                                                                            \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        auto status = (expr);                                                                                                                                                                          \
        if (status != CAMERA_STATUS_SUCCESS)                                                                                                                                                           \
        {                                                                                                                                                                                              \
            fmt::print(fg(fmt::color::red), "[ERROR] '" #expr "' = ({}) " info "\n", status, ##__VA_ARGS__);                                                                                           \
            return false;                                                                                                                                                                              \
        }                                                                                                                                                                                              \
    } while (0)

#define MV_CHECK_API_THROW(expr, info, ...)                                                                                                                                                            \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        auto status = (expr);                                                                                                                                                                          \
        if (status != CAMERA_STATUS_SUCCESS)                                                                                                                                                           \
        {                                                                                                                                                                                              \
            throw MindVision_FrameError(fmt::format("'" #expr "' = ({}) " info, status, ##__VA_ARGS__));                                                                                               \
        }                                                                                                                                                                                              \
    } while (0)

MV_Camera::MV_Camera(const char *camera_cfg) : camera_cfg(camera_cfg)
{
    MV_CHECK_API_WARNING(CameraSdkInit(1), "");
    handle = -1;
}

MV_Camera::~MV_Camera()
{
    close();
}

bool MV_Camera::open()
{
    if (isOpen())
    {
        if (!close())
            return false;
    }

    tSdkCameraDevInfo infos[2];

    int dev_num = 2;
    //获得设备的信息, dev_num返回找到的相机个数
    MV_CHECK_API_ERROR(CameraEnumerateDevice(infos, &dev_num), "");

    MV_ASSERT_ERROR(dev_num > 0, "未发现设备！");
    //相机初始化
    MV_CHECK_API_ERROR(CameraInit(&infos[0], -1, -1, &handle), "");

    MV_ASSERT_ERROR(handle >= 0, "相机未找到！");
    //读取相机配置文件
    MV_CHECK_API_WARNING(CameraReadParameterFromFile(handle, (char *)camera_cfg.data()), "读取相机配置文件错误，路径{}", camera_cfg);

    //设置相机触发模式
    // MV_CHECK_API_ERROR(CameraSetTriggerMode(handle, 0), "");

    MV_CHECK_API_ERROR(CameraPlay(handle), "");
/*
    tSdkCameraCapbility tCapability; //设备描述信息
    int                 channel;

    CameraGetCapability(handle, &tCapability);
    if (tCapability.sIspCapacity.bMonoSensor)
    {
        channel = 1;
        // CameraSetIspOutFormat(handle, CAMERA_MEDIA_TYPE_MONO8);
        MV_CHECK_API_ERROR(CameraSetIspOutFormat(handle, CAMERA_MEDIA_TYPE_MONO8), "");
    }
    else
    {
        channel = 3;
        // CameraSetIspOutFormat(handle, CAMERA_MEDIA_TYPE_BGR8);
        MV_CHECK_API_ERROR(CameraSetIspOutFormat(handle, CAMERA_MEDIA_TYPE_BGR8), "");
    }
    */
    MV_CHECK_API_ERROR(CameraSetIspOutFormat(handle, CAMERA_MEDIA_TYPE_BGR8), "");

    // MV_CHECK_API_ERROR(CameraSaveParameterToFile(handle, PROJECT_DIR"/configs/camera/MV-SUA133GC-T_042003320218.Config"), "");

    return true;
}

bool MV_Camera::close()
{
    MV_ASSERT_WARNING(handle >= 0, "相机已经关闭！"); //
    if (handle < 0)
        return true;
    MV_CHECK_API_ERROR(CameraUnInit(handle), "");
    handle = -1;
    return true;
}

bool MV_Camera::isOpen() const
{
    return handle >= 0;
}

bool MV_Camera::read(cv::Mat &img) const
{
    MV_ASSERT_ERROR(isOpen(), "相机未打开！");
    tSdkFrameHead head;
    BYTE *        buffer;
    // 100 为超时时间
    MV_CHECK_API_ERROR(CameraGetImageBuffer(handle, &head, &buffer, 10000), "");
    img = cv::Mat(head.iHeight, head.iWidth, CV_8UC3);
    MV_CHECK_API_ERROR(CameraImageProcess(handle, buffer, img.data, &head), "");
    MV_CHECK_API_ERROR(CameraReleaseImageBuffer(handle, buffer), "");
    return true;
}
/**
 * @brief 这个函数的double是可以获取当前帧的采集时间
 *
 */
bool MV_Camera::read(cv::Mat &img, double &timestamp_ms) const
{
    MV_ASSERT_ERROR(isOpen(), "相机未打开！");
    tSdkFrameHead head;
    BYTE *        buffer;
    // 100 为超时时间
    MV_CHECK_API_ERROR(CameraGetImageBuffer(handle, &head, &buffer, 10000), "");
    img = cv::Mat(head.iHeight, head.iWidth, CV_8UC3);
    MV_CHECK_API_ERROR(CameraImageProcess(handle, buffer, img.data, &head), "");
    // timestamp_ms = head.uiTimeStamp;//uiTimeStamp 该帧的采集时间，单位0.1毫秒
    timestamp_ms = head.uiExpTime;//当前图像的曝光值，单位为微秒us
    MV_CHECK_API_ERROR(CameraReleaseImageBuffer(handle, buffer), "");
    return true;
}

bool MV_Camera::get_exposure_us(double &us) const
{
    MV_ASSERT_ERROR(isOpen(), "相机未打开！");
    MV_CHECK_API_ERROR(CameraGetExposureTime(handle, &us), "");
    return true;
}

bool MV_Camera::set_exposure_us(double us) const
{
    MV_ASSERT_ERROR(isOpen(), "相机未打开！");
    MV_CHECK_API_ERROR(CameraSetExposureTime(handle, us), "");
    return true;
}

} // namespace devices