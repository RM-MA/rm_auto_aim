#include <fmt/color.h>
#include <fmt/core.h>
#include <fmt/format.h>

#include <Eigen/Dense>
#include <ctime>
#include <string>


/**
 * @brief 格式化用户定义的类型
 * 实现 parse 和 format方法
 */
template <> struct fmt::formatter<Eigen::Matrix<double, 3, 3>> {

    constexpr auto parse(format_parse_context& ctx) ->decltype(ctx.begin()){
        auto it = ctx.begin(), end = ctx.end();
        return it;
    }


    template <typename FormatContext> auto format(const Eigen::Matrix<double, 3, 3>& m, FormatContext& ctx) -> decltype(ctx.out()) {
        return format_to(ctx.out(), "{},{},{},{},{},{},{},{},{}", 
            m(0, 0), m(0, 1), m(0, 2), 
            m(1, 0), m(1, 1), m(1, 2), 
            m(2, 0), m(2, 1), m(2, 2));
    }
};

int main() {
    std::time_t t = std::time(nullptr);

    auto time_str = fmt::format(fg(fmt::color::yellow), "{}", t);
    auto message  = fmt::format("{}", 123);


    Eigen::Matrix<double, 3, 3> m;
    m << 1,2,3,4,5,6,7,8,9;


    fmt::print("[{}] {} {}\n", time_str, message, m);

    return 0;
}