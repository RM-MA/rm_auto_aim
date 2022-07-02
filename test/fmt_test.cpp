#include <Eigen/Dense>
#include <bits/types/time_t.h>
#include <fmt/color.h>
#include <fmt/core.h>

#include <ctime>
#include <string>

#include <fmt/format.h>

template<> struct fmt::formatter<Eigen::MatrixX3d>{
    
};


int main() {
    time_t t = time(0);

    std::string time_str = fmt::format(fg(fmt::color::yellow), "[{}] ", t);
    std::string message  = fmt::format("{}\n", 123);

    fmt::print(time_str);
    fmt::print(message);

    return 0;
}