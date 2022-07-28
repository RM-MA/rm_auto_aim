#ifndef _PREDICTOR_EKF_HPP_
#define _PREDICTOR_EKF_HPP_

#include "EKF.h"

class PredictorEKF{


private:
    AdaptiveEKF ekf;// ekf预测器
    Eigen::Matrix3d R_CI; // 相机坐标系 到 枪管坐标系 的旋转矩阵
    

};


#endif /*_PREDICTOR_EKF_HPP_*/