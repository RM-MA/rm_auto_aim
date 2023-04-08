#include <Eigen/Dense>

#include <iostream>

using namespace Eigen;
using namespace std;

int main()
{
    // AngleAxisd pitch(M_PI / 6, Vector3d(0, 1, 0));
    // AngleAxisd yaw(M_PI / 6., Vector3d(0, 0, 1));

    // Matrix3d r;
    // r = pitch;
    // auto res = r * p;

    // // cout << "picth = \n" << res << endl;
    // cout << "pitch, yaw = \n" << yaw * (pitch * p) << endl;
    // cout << "yaw, pitch = \n" << pitch * (yaw * p) << endl;

    // Vector3d Z(0, 0, 1);
    // Z = AngleAxisd(-M_PI / 6., Vector3d(0, 1, 0)) * Z;

    // // cout << "pitch, yaw = \n" << yaw * (pitch * p) << endl;
    // cout << "yaw, pitch = \n" << pitch * (yaw * p) << endl;

    // Vector3d Y(0, 1, 0);
    // Y = AngleAxisd(-M_PI / 6., Vector3d(0, 0, 1)) * Y;

    // // // 旋转坐标系
    // // - 从y轴往下看 顺时针 上
    // // + 从y轴往下看 逆时针 下
    // Matrix3d picth_ro;
    // picth_ro = AngleAxisd(M_PI / 4., Vector3d(0, 1, 0));

    // // - 从z轴往下看 顺时针 右
    // // + 从z轴往下看 逆时针 左
    // Matrix3d yaw_ro;
    // yaw_ro = AngleAxisd(-M_PI / 4., Vector3d(0, 0, 1));

    // cout << picth_ro * p << endl;

    // auto p_y_1 = picth_ro * p;

    // cout << yaw_ro * p_y_1 << endl;

    // cout << yaw_ro * picth_ro * p << endl;

    // Matrix3d p_y;
    // p_y =

    // cout << "pitch = \n" << (AngleAxisd(M_PI / 6., Vector3d(0, 1, 0)) * p) << endl;

    // cout << "Z = \n" << AngleAxisd(M_PI / 6., Vector3d(0, 1, 0)) * Vector3d(0, 0, 1) << endl;
    // Matrix3d r;
    // r =
    // AngleAxisd(M_PI / 4., Vector3d(0, 1, 0))
    // *
    // AngleAxisd(M_PI / 4., Vector3d(std::sqrt(2) / 2., 0, std::sqrt(2) / 2.0));

    // cout << "pitch, yaw = \n" << r << endl;

    // // Matrix3d r_inv;

    // // r_inv =
    // // AngleAxisd(M_PI / 4., Vector3d(std::sqrt(2) / 2., 0, std::sqrt(2) / 2.0))
    // // *
    // // AngleAxisd(M_PI / 4., Vector3d(0, 1, 0));
    // // cout << "inv = \n" <<  r_inv << endl;

    // Matrix3d rr;
    // rr =
    // AngleAxisd(M_PI / 4., Vector3d(0, 0, 1))
    //  *
    // AngleAxisd(M_PI / 4., Vector3d(-std::sqrt(2) / 2., std::sqrt(2) / 2.0, 0));

    // cout << "yaw, pitch = \n" << rr << endl;

    Vector3d p(3, 2, 1);  // (x, y, z), 世界坐标
    double static_pitch = ( 30 / 180.) * M_PI;
    double static_yaw   = ( 60 / 180.) * M_PI;

    auto axis1 = Vector3d(0, 1, 0);
    auto axis2 = (AngleAxisd(static_pitch, axis1) * Vector3d(0, 0, 1));

    cout << "axis = \n" << axis1.transpose() << "\t: " << static_pitch
     << "\n" << axis2.transpose() << "\t:" << static_yaw << endl;

    auto p1 = AngleAxisd(static_pitch, axis1) * p;
    auto p2 = (AngleAxisd(static_yaw, axis2)) * p1;

    Matrix3d r;
    r = AngleAxisd(static_yaw, axis2) * AngleAxisd(static_pitch, axis1);
    // cout << "pitch, yaw = \n" << r << endl;
    // cout << "p1 = \n" << p1 << "\np2= \n" << p2 << "\n p22 = \n" << r * p << endl;


    // Matrix3d r_inv;

    // r_inv =
    // AngleAxisd(M_PI / 4., Vector3d(std::sqrt(2) / 2., 0, std::sqrt(2) / 2.0))
    // *
    // AngleAxisd(M_PI / 4., Vector3d(0, 1, 0));
    // cout << "inv = \n" <<  r_inv << endl;

    auto axis11 = Vector3d(0, 0, 1);
    auto axis22 = (AngleAxisd(static_yaw, axis11) * Vector3d(0, 1, 0));

    cout << "axis = \n" << axis11.transpose() << "\t: " << static_yaw << "\n"
     << axis22.transpose() << "\t: " << static_pitch << endl;

    auto p11 = AngleAxisd(static_yaw, axis11) * p;
    auto p22 = (AngleAxisd(static_pitch, axis22)) * p11;

    Matrix3d rr;
    rr = AngleAxisd(static_pitch, axis22) * AngleAxisd(static_yaw, axis11);

    // cout << "axis = \n"
    //      << Vector3d(0, 0, 1).transpose() << "\n"
    //      << (AngleAxisd(M_PI / 4., Vector3d(0, 0, 1)) * Vector3d(0, 1, 0)).transpose() << endl;
    // cout << "p1 = \n" << p11 << "\np2= \n" << p22 << "\n p22 = \n" << rr * p << endl;

    cout << "pitch, yaw = \n" << r << endl;
    cout << "yaw, pitch = \n" << rr << endl;
    // cout << "inv = \n" << rr.inverse() << endl;
    // cout << r * rr << endl;

    return 0;
}