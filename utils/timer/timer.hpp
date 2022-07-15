#ifndef _TIMER_HPP_
#define _TIMER_HPP_

#include <fmt/core.h>
#include <chrono>
#include <string>
#include <vector>

namespace utils
{
class timer
{
public:
    explicit timer(const std::string& , int, bool = true);
    bool start(int);
    double end(int, const std::string &);

private:
    std::vector<std::chrono::system_clock::time_point> timePoints;
    const std::string author;
    bool isPrint;
};
}  // namespace utils

#endif /*_TIMER_HPP_*/