#include <Eigen/Dense>

#include <iostream>

using namespace Eigen;
using namespace std;

int main()
{

    // 
    float receive_pitch = (10. / 180.) * M_PI; // 上+，X轴向Z轴转
    float receive_yaw = (15. / 180.) / M_PI; // 左+，X轴向Y轴转

    // 云台坐标系下- 云台坐标系的X轴
    Vector3d X_Gimbal(1, 0, 0);
    // 世界坐标系下- 云台坐标系的X轴
    Vector3d X_World;

    double x = X_Gimbal.norm() / (1 + pow(tan(receive_pitch), 2) + pow(tan(receive_yaw), 2));
    // 归一化
    X_World[0] = x;
    X_World[1] = x * tan(receive_yaw);
    X_World[2] = x * tan(receive_pitch);


    Quaterniond R_G2W = Quaterniond::FromTwoVectors(X_Gimbal, X_World);

    cout << "X World =\n" << X_Gimbal << "\nX World = \n" << X_World << endl;
    cout << "R Gimbal to World = \n" << R_G2W.toRotationMatrix() << endl;




    return 0;
}