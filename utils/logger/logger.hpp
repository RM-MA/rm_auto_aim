#ifndef _LOGGER_HPP_
#define _LOGGER_HPP_

#include <string>
#include <fmt/core.h>
#include <fmt/os.h>
#include <fmt/color.h>

#include <csignal>


namespace logger {


/**
 * @brief logger
 * @param fmt 格式化字符串
 * @param path 文件保存路径
 */
class logger {
public:
    logger(const std::string& fmt, const std::string& path);

    bool ok();

    template <typename... T>
    bool write(T&&... args);

    bool close();

    ~logger();

    logger(logger const&) = delete;
    logger& operator=(logger const&) = delete;
private:
    std::string fmt;
    bool is_success;
};// class logger

}  // namespace logger

#endif /*_LOGGER_HPP_*/