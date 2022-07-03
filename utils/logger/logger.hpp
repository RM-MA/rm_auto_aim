#ifndef _LOGGER_HPP_
#define _LOGGER_HPP_

#include <fmt/color.h>
#include <fmt/core.h>
#include <fmt/os.h>

#include <csignal>
#include <cstdint>
#include <ctime>
#include <ostream>
#include <string>

namespace logger {

enum class LOGGER_TYPE : std::uint8_t { WRITE, ALL };

/**
 * @brief logger
 * @param fmt 格式化字符串
 * @param type 显示方式，可选：log(只在终端进行显示)、write(只写入文件)、all(都)
 * @param path 文件保存路径
 * @param addTimestamp 是否添加时间戳
 */
class logger {
public:
    logger(const std::string& fmt_, const LOGGER_TYPE& type_, const std::string& path_, bool addTimestamp = true, const std::string& description_ = "", int oflag = O_WRONLY | O_CREAT)
        : fmt(fmt_), type(type_), path(path_), output_file(fmt::output_file(path_, oflag)), add_timestamp(addTimestamp), description(description_) {
        if (add_timestamp) {
            fmt = "{}, " + fmt;
        }
    }

    bool ok();

    template <typename... T> bool write(T&&... args) {
        switch (type) {
        case LOGGER_TYPE::WRITE: {
            if (add_timestamp) {
                std::time_t t = std::time(nullptr);
                output_file.print(fmt, t, args...);
            } else {
                output_file.print(fmt, args...);
            }
            break;
        }
        case LOGGER_TYPE::ALL: {
            if (add_timestamp) {
                std::time_t t = std::time(nullptr);
                output_file.print(fmt, t, args...);
                fmt::print(fmt, t, args...);
            } else {
                output_file.print(fmt, args...);
                fmt::print(fmt, args...);
            }
            break;
        }
        default: {
            return false;
            break;
        }
        }

        return true;
    };

    bool close();

    ~logger();

    logger(logger const&) = delete;
    logger& operator=(logger const&) = delete;

private:
    bool is_success;
    bool add_timestamp;

    LOGGER_TYPE type;

    fmt::ostream output_file;

    std::string fmt;
    std::string path;
    std::string description;
};  // class logger

}  // namespace logger

#endif /*_LOGGER_HPP_*/