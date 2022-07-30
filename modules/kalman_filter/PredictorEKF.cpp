#include "PredictorEKF.hpp"

// 射击延时。单位 s
double shoot_delay_time = 0.1;

Modules::PredictorEKF::PredictorEKF() : ekf()
{
    cv::FileStorage fin{PROJECT_DIR "/Configs/camera/camera.yaml", cv::FileStorage::READ};

    fin["cameraMatrix"] >> F_MAT;  //相机内参
    fin["distCoeffs"] >> C_MAT;    //相机畸变
    fin["Tcb"] >> R_CI_MAT;        //陀螺仪坐标系到相机坐标系的旋转矩阵

    std::cout << "内参=" << F_MAT << ",\n 畸变=" << C_MAT << std::endl;

    cv::cv2eigen(R_CI_MAT, R_CI);
    cv::cv2eigen(F_MAT, F);
    cv::cv2eigen(C_MAT, C);
    // std::cout << C<< std::endl;
    // std::cout<< F << std::endl;

    double small_half_x, small_half_y;
    double big_half_x, big_half_y;

    fin["armour"]["small_half_x"] >> small_half_x;
    fin["armour"]["small_half_y"] >> small_half_y;
    fin["armour"]["big_half_x"] >> big_half_x;
    fin["armour"]["big_half_y"] >> big_half_y;

    /*
    - point 0: [-squareLength / 2, squareLength / 2, 0]
    - point 1: [ squareLength / 2, squareLength / 2, 0]
    - point 2: [ squareLength / 2, -squareLength / 2, 0]
    - point 3: [-squareLength / 2, -squareLength / 2, 0]
    */
    small_obj = std::vector<cv::Point3d>{
        {-small_half_x, -small_half_y, 0},  //left top
        {small_half_x, -small_half_y, 0},   //right top
        {small_half_x, small_half_y, 0},    //right bottom
        {-small_half_x, small_half_y, 0}};  //left bottom
    big_obj = std::vector<cv::Point3d>{
        {-big_half_x, -big_half_y, 0},  //left top
        {big_half_x, -big_half_y, 0},   //right top
        {big_half_x, big_half_y, 0},    //right bottom
        {-big_half_x, big_half_y, 0}};  //left bottom

    fin.release();
}

bool Modules::PredictorEKF::predict(
    Robot::Detection_pack & detection_pack, const Devices::ReceiveData & receive_data,
    Devices::SendData & send_data, cv::Mat & showimg)
{
    double timestamp = detection_pack.timestamp;
    auto & img       = detection_pack.img;
    auto & armours   = detection_pack.armours;

    if (armours.empty()) {  //.size() == 0, 会报错
        return false;
    }
    // auto & armours = detection_pack.armours;
    //对装甲板进行排序, 优先大装甲板, 其次选最大的
    std::sort(
        armours.begin(), armours.end(), [](const Robot::Armour & a1, const Robot::Armour & a2) {
            if (a1.armour_type == Robot::ArmourType::Big &&
                a2.armour_type == Robot::ArmourType::Small) {
                return true;
            } else if (
                a1.armour_type == Robot::ArmourType::Small &&
                a2.armour_type == Robot::ArmourType::Big) {
                return false;
            }

            auto w1      = a1.right_light.top - a1.left_light.top;
            auto h1      = a1.left_light.bottom - a1.left_light.top;
            double area1 = std::fabs(w1.x * h1.y - w1.y * h1.x);

            auto w2      = a2.right_light.top - a2.left_light.top;
            auto h2      = a2.left_light.bottom - a2.left_light.top;
            double area2 = std::fabs(w2.x * h2.y - w2.y * h2.x);

            return area1 > area2;
        });

    auto select_armour = armours.front();
    auto pts           = select_armour.get_points();  //装甲板的四点，用来测距

    // 根据 传来的pitch角度构造 旋转矩阵
    double pitch = receive_data.pitch / 180. * M_PI;
    fmt::print("[read] pitch={}\n", pitch / M_PI * 180.);
    Eigen::Matrix3d R_WI;
    R_WI = Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY());

    // 得到3个坐标系下的坐标
    Eigen::Vector3d camera_points = get_camera_points(pts, select_armour.armour_type);
    Eigen::Vector3d i_points      = R_CI.transpose() * camera_points;
    Eigen::Vector3d world_points  = pc2pw(camera_points, R_WI.transpose());
    // 相机坐标系，
    // x轴向右，y轴向下，z轴延相机向前方
    fmt::print(
        "[{:<6}]: {:.3f},{:.3f},{:.3f}\n", camera_fmt, camera_points(0, 0), camera_points(1, 0),
        camera_points(2, 0));
    //x轴延枪管向前，y轴向左，z轴向上
    fmt::print(
        "[{:<6}]: {:.3f},{:.3f},{:.3f}\n", i_fmt, i_points(0, 0), i_points(1, 0), i_points(2, 0));
    //xy平面平行于地面，z轴垂直地面
    //x轴延枪管向前，y轴向左，z轴向上
    fmt::print(
        "[{:<6}]: {:.3f},{:.3f},{:.3f}\n", world_fmt, world_points(0, 0), world_points(1, 0),
        world_points(2, 0));

    // 更新时间
    predictfunc.delta_t = timestamp - last_time;
    last_time           = timestamp;
    // fmt::print("delay_time = {}s\n", predictfunc.delta_t);

    // ekf滤波出来，三维坐标和速度
    ekf.predict(predictfunc);
    // x,y,z ,v_x, v_y, v_z
    VectorX smooth_status = ekf.update(measure, world_points);

    // 根据弹道模型求出 pitch角度 和 飞行时间
    // double T_k;  // 飞行时间
    double f_tk, f_tk_;
    double h_k, h_r;
    double e_k;

    double pitch_0 = std::atan2(world_points(2, 0), world_points(0, 0));
    double pitch_k = pitch_0;
/*
    int k = 1;
    for (; k <= max_epochs; k++) {
        f_tk = 1 / k_1 * std::log(k_1 * receive_data.shoot_speed * T_k + 1) - smooth_status(0, 0) -
               smooth_status(3, 0) * T_k;
        f_tk_ = receive_data.shoot_speed / (k_1 * receive_data.shoot_speed * T_k + 1) -
                smooth_status(3, 0);
        T_k = T_k - f_tk / f_tk_;
        h_k = receive_data.shoot_speed * std::sin(pitch_k) - 9.8 / 2 * T_k * T_k;
        // 求解 h_r，经过弹丸飞行时间 T_k 后，目标实际高度
        h_r = smooth_status(2, 0) + smooth_status(5, 0) * T_k;
        // 计算误差
        e_k = h_r - h_k;
        if (std::fabs(e_k) < min_ek) {
            break;
        }

        pitch_k = pitch_k + K * e_k;
    }
*/
    int k = 1;
    for (; k <= max_epochs; k++) {
        f_tk = 1 / k_1 * std::log(k_1 * receive_data.shoot_speed * T_k + 1) - world_points(0, 0);
        f_tk_ = receive_data.shoot_speed / (k_1 * receive_data.shoot_speed * T_k + 1);
        T_k = T_k - f_tk / f_tk_;
        h_k = receive_data.shoot_speed * std::sin(pitch_k) - 9.8 / 2 * T_k * T_k;
        // 求解 h_r，经过弹丸飞行时间 T_k 后，目标实际高度
        h_r = world_points(2, 0);
        // 计算误差
        e_k = h_r - h_k;
        if (-e_k < min_ek  && e_k <= 0) {
            break;
        }

        pitch_k = pitch_k + K * e_k;
    }

    fmt::print(
        "最终迭代picth_k={:.3f}度, e_k={:.3f}, T_k={},shoot={}\n", pitch_k / M_PI * 180.,
        e_k, T_k,receive_data.shoot_speed);  // 弧度 转 度

    //
    double predict_time = shoot_delay_time + T_k;  // 射击延时 + 飞行延时

    Eigen::Vector3d predict_world_points = smooth_status.topRows<3>();
    predict_world_points(0, 0) += smooth_status(3, 0) * predict_time;
    predict_world_points(1, 0) += smooth_status(4, 0) * predict_time;
    predict_world_points(2, 0) += smooth_status(5, 0) * predict_time;

    fmt::print(
        "[{}滤波]: cx={:.3f}, cy={:.3f}, cz={:.3f}\n", predict_fmt, smooth_status(0, 0),
        smooth_status(1, 0), smooth_status(2, 0));
    fmt::print(
        "[{}滤波]: vx={:.3f}, vy={:.3f}, vz={:.3f}\n", predict_fmt, smooth_status(3, 0),
        smooth_status(4, 0), smooth_status(5, 0));

    // 将预测的装甲板中心点的世界坐标，投影到图像中，并绘制出来
    re_project_point(showimg, predict_world_points, R_WI.transpose(), cv::Scalar(255, 0, 0));

    // 求解发送的yaw和pitch角度, 单位: 度
    double send_pitch = std::atan2(i_points(2, 0), i_points(0, 0)) / M_PI * 180.;   // 向上为正
    double send_yaw   = -std::atan2(i_points(1, 0), i_points(0, 0)) / M_PI * 180.;  // 向右为正

    send_pitch = (pitch_k)/ M_PI * 180.  - receive_data.pitch;
    send_yaw += 1.5;
    fmt::print("[send]: yaw={:.3f}, pitch={:.3f}\n", send_yaw, send_pitch);

    send_data.send_pitch = send_pitch; 
    send_data.send_yaw   = send_yaw;
    send_data.goal       = 1;

    return true;
}

Eigen::Vector3d Modules::PredictorEKF::get_camera_points(
    std::vector<cv::Point2f> & armour_points, Robot::ArmourType type)
{
    cv::Mat tvec, rvec;
    if (type == Robot::ArmourType::Big) {
        cv::solvePnP(big_obj, armour_points, F_MAT, C_MAT, rvec, tvec);
        fmt::print("big armour\n");
    } else {
        cv::solvePnP(small_obj, armour_points, F_MAT, C_MAT, rvec, tvec);
        fmt::print("small armour\n");
    
    }


    Eigen::Vector3d camera_points;

    cv::cv2eigen(tvec, camera_points);

    return camera_points;
}