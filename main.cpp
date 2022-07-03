#include <fcntl.h>
#include <fmt/color.h>
#include <fmt/core.h>
#include <unistd.h>  //信号处理

#include <csignal>  //信号处理
#include <string>

#include "opencv2/opencv.hpp"
#include "utils/logger/logger.hpp"

bool main_loop_condition = true;  //主循环的条件

class test {//测试析构函数会不会运行
public:
    test(const std::string& s_) : s(s_) {}
    ~test() { fmt::print("test {}释放了~~~\n", s); };

private:
    std::string s;
};

void signalHandler(int signum)//信号处理函数
{
    fmt::print(fg(fmt::color::red) | fmt::emphasis::bold, "正在退出,请等待释放资源!\n");
    main_loop_condition = false;
}

//可变参数列表
template <typename... T> void Print(T&&... args) { fmt::print("{} {} \n", args...); }

int main(int argc, char** argv) {
    test s{"a"};
    signal(SIGINT, signalHandler);

    int i = 1;
    logger::logger my_logger{"i={}, j={}\n", logger::LOGGER_TYPE::ALL, "test.csv", true, "", O_WRONLY | O_APPEND};
    my_logger.ok();
    while (main_loop_condition) {
        my_logger.write(i, i*2.1);
        // Print("1", 2);
        i++;
        sleep(1);
    }
    my_logger.close();
    return 0;
}