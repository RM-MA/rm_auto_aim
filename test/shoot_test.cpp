#include <cmath>
#include <complex>

#include <fmt/color.h>
#include <fmt/core.h>

struct Pose
{
    double x, y, z;
    double v_x, v_y, v_z;
};

int main()
{
    /**
     * @brief 
     * 已知位姿信息, roll=0, pitch, yaw
     * 相机坐标系: xy轴和图片坐标系相同, x轴向右，y轴向下，z轴指向前方。
     * 惯性坐标系: x轴指向前方，y轴指向左方，z轴指向上方。

    弹道模型的计算都是在惯性坐标系下
    计算高度补偿
     */
    //
    double shoot_speed = 27.1;  //射速, 单位 m/s
    double K           = 0.05;  //theta角更新比例，相当于深度学习中的学习率，太大不容易收敛，太小更新太慢
    int max_epochs     = 50;    //最大迭代轮数
    double min_ek      = 0.01;  //最小误差

    Pose a;
    a.v_x = a.v_y = a.v_z = 0;

    a.z = 0.;
    a.x = 5.;
    a.y = 0.;

    double pitch_0 = std::atan2(a.z, std::sqrt(a.x * a.x + a.y * a.y));  //初始迭代 pitch角度
    double theta_k = pitch_0;

    double k_1 = 0.01903;  //k_1 = k/m

    double T_k = 0;
    double h_k, h_r;
    double f_Tk;
    double f_Tk_;
    double e_k;
    int k = 1;

    for (; k < max_epochs; k++) {
        fmt::print(fg(fmt::color::red), "[Epochs: {}/{}]: \n", k, max_epochs);
        // 更新 T_k
        f_Tk  = 1 / k_1 * std::log(k_1 * shoot_speed * T_k + 1) - a.x - a.v_x * T_k;
        f_Tk_ = shoot_speed / (k_1 * shoot_speed * T_k + 1) - a.v_x;

        T_k = T_k - f_Tk / f_Tk_;
        fmt::print(
            "f_Tk={:.5f}, f_Tk_={:.5f}, 变化量={:.5f}, 飞行时间={:.5f}s\n", f_Tk, f_Tk_,
            -f_Tk / f_Tk_, T_k);
        // 求解 h_k，第 k 次迭代弹丸落点高度
        h_k = shoot_speed * std::sin(theta_k) - 9.8 / 2 * T_k * T_k;
        // 求解 h_r，经过弹丸飞行时间 T_k 后，目标实际高度
        h_r = a.z + a.v_z * T_k;
        // 计算误差
        e_k = h_r - h_k;
        fmt::print("h_k={:.5f}, h_r={:.5f}\n", h_k, h_r);
        if (std::fabs(e_k) < min_ek) {
            break;
        }
        // 更新俯仰角(pitch)，这里使用的 比例K 控制更新
        theta_k = theta_k + K * e_k;
    }

    fmt::print("最终迭代theta={}度\n", theta_k / M_PI * 180.);  //弧度 转 度

    return 0;
}