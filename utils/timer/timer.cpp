#include "timer.hpp"
#include <fmt/color.h>
#include <fmt/core.h>

namespace utils
{
timer::timer(const std::string & author, int num, bool isPrint) : author(author), timePoints(num), isPrint(isPrint)
{
}

bool timer::start(int index)
{
    // auto start = std::chrono::
    if (index < timePoints.size() && index >= 0) {
        timePoints[index] = std::chrono::system_clock::now();
        //        std::cout <<"[index: "<< index <<"], 开始计时" << std::endl;
        if (isPrint) {
            fmt::print("[{:<8}:{:>2}], 开始计时\n", author, index);
        }
    } else {
        fmt::print(fg(fmt::color::red), "index 越界\n");
    }

    return true;
}

double timer::end(int index, const std::string & info)
{
    if (index < timePoints.size() && index >= 0) {
        auto endTime = std::chrono::system_clock::now();
        auto wasteTime =
            std::chrono::duration_cast<std::chrono::microseconds>(endTime - timePoints[index])
                .count() /
            1e3;
        //        std::cout <<"[index: "<< index <<"], [message:"<< str << "] 花费时间" << wasteTime << "ms" << std::endl;
        if (isPrint) {
            fmt::print("[{:<8}:{:>2}], 花费时间 [{:.3f}] ms, {}\n", author, index, wasteTime, info);
        }

        return wasteTime;
    } else {
        fmt::print(fg(fmt::color::red), "index 越界\n");
        return 0;
    }
}
}  // namespace utils