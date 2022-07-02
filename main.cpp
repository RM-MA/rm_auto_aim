#include <fmt/color.h>
#include <fmt/core.h>
#include <unistd.h>  //信号处理

#include <csignal>  //信号处理
#include <string>

#include "opencv2/opencv.hpp"

bool main_loop_condition = true;  //主循环的条件

class test {
public:
    test(const std::string& s_) : s(s_) {}
    ~test() { fmt::print("~~~{}\n", s); };

private:
    std::string s;
};

void signalHandler(int signum) {
    fmt::print(fg(fmt::color::red) | fmt::emphasis::bold, "正在退出,请等待释放资源!\n");
    main_loop_condition = false;
    // exit(0);
}

//可变参数列表
template <typename... T> void Print(T&&... args) { fmt::print("{} {} \n", args...); }

int main(int argc, char** argv) {
    test s{"a"};
    signal(SIGINT, signalHandler);

    while (main_loop_condition) {
        Print("1", 2);
        sleep(1);
    }
    return 0;
}