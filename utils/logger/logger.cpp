#include "logger.hpp"

#include <fmt/color.h>
#include <fmt/core.h>
#include <fmt/os.h>

logger::logger::logger(const std::string& fmt_, const LOGGER_TYPE& type_, const std::string& path_, bool addTimestamp_, const std::string& description_)
    : fmt(fmt_), type(type_), path(path_), output_file(fmt::output_file(path_)), add_timestamp(addTimestamp_), description(description_) {
    // auto temp = fmt::output_file(path);
    if (add_timestamp) {
        fmt = "{}, " + fmt;
    }
    // output_file = fmt::ostream(temp);
}

bool logger::logger::ok() { return true; }
/*
template <typename... T> 
bool logger::logger::write(T&&... args) {
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
}
*/
bool logger::logger::close() {
    output_file.close();
    is_success = false;
    return true;
}
logger::logger::~logger() {
    if (!is_success) {
        close();
    }
    fmt::print(fg(fmt::color::red), "logger 已释放, 文件存放: ");
    fmt::print(fg(fmt::color::blue) | fmt::emphasis::underline, "{}\n", path);
}
