#include "PredictorEKF.hpp"

// 射击延时。单位 s
double shoot_delay_time = 0.1;

Modules::PredictorEKF::PredictorEKF() : ekf()
{
    cv::FileStorage fin{PROJECT_DIR "/Configs/camera/camera.yaml", cv::FileStorage::READ};

    fin["cameraMatrix"] >> F_MAT;  //相机内参
    fin["distCoeffs"] >> C_MAT;    //相机畸变
    fin["R_C2G"] >> R_C2G_MAT;     //相机坐标系 到 云台坐标系 的旋转矩阵

    // p_g = R_C2G * p_c

    std::cout << "内参=" << F_MAT << ",\n 畸变=" << C_MAT << std::endl;

    cv::cv2eigen(R_C2G_MAT, R_C2G);
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

        // 空气阻力系数
    fin["k"] >> k;
    // 静态角度补偿
    fin["static_yaw"] >> static_yaw;
    fin["static_pitch"] >> static_pitch;
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

    // 选择策略
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
    double pitch_radian = receive_data.pitch / 180. * M_PI;
    // fmt::print("[read] pitch_angle={}, yaw_angle={},shoot_speed={}\n", receive_data.pitch, receive_data.yaw,receive_data.shoot_speed );

    Eigen::Matrix3d R_G2W = PredictorEKF::get_R_G2W(receive_data);

    // 得到3个坐标系下的坐标,
    Eigen::Vector3d camera_points = get_camera_points(pts, select_armour.armour_type);
    Eigen::Vector3d gimbal_points = R_C2G * camera_points;
    Eigen::Vector3d world_points  = R_G2W * gimbal_points;


    // 相机坐标系，
    // x轴向右，y轴向下，z轴延相机向前方
    fmt::print(
        "[{:<6}]: {:.3f},{:.3f},{:.3f}\n", camera_fmt, camera_points(0, 0), camera_points(1, 0),
        camera_points(2, 0));
    //x轴延枪管向前，y轴向左，z轴向上
    fmt::print(
        "[{:<6}]: {:.3f},{:.3f},{:.3f}\n", gimbal_fmt, gimbal_points(0, 0), gimbal_points(1, 0),
        gimbal_points(2, 0));
    //xy平面平行于地面，z轴垂直地面
    //x轴延枪管向前，y轴向左，z轴向上
    fmt::print(
        "[{:<6}]: {:.3f},{:.3f},{:.3f}\n", world_fmt, world_points(0, 0), world_points(1, 0),
        world_points(2, 0));


    // show posistion, 在图片上画 三个坐标系的坐标
    std::string camera_position_fmt = fmt::format("[camera]: x={:.3f},y={:.3f},z={:.3f}", camera_points(0, 0), camera_points(1, 0),
        camera_points(2, 0));
    std::string gimbal_positino_fmt = fmt::format("[gimbal]: x={:.3f}, y={:.3f}, z={:.3f}", gimbal_points(0, 0),
        gimbal_points(1, 0), gimbal_points(2, 0));
    std::string world_position_fmt = fmt::format("[world]: x={:.3f},y={:.3f},z={:.3f}", world_points(0, 0), world_points(1, 0),
        world_points(2, 0));

    cv::putText(showimg,world_position_fmt, select_armour.left_light.bottom - cv::Point2f(0, -30), 1, cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 0, 255));
    cv::putText(showimg,gimbal_positino_fmt, select_armour.left_light.bottom - cv::Point2f(0, -60), 1, cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 0, 255));
    cv::putText(showimg,camera_position_fmt, select_armour.left_light.bottom - cv::Point2f(0, -90), 1, cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 255, 255));
    // ----------------- 卡尔曼滤波的使用 ------------------------

    // // 更新时间
    // predictfunc.delta_t = timestamp - last_time;
    // last_time           = timestamp;
    // // fmt::print("delay_time = {}s\n", predictfunc.delta_t);

    // // ekf滤波出来，三维坐标和速度
    // ekf.predict(predictfunc);
    // // x,y,z ,v_x, v_y, v_z
    // VectorX smooth_status = ekf.update(measure, world_points);
    // ----------------------------------------------------------

    double send_yaw =
        -std::atan2(gimbal_points(1, 0), gimbal_points(0, 0)) / M_PI * 180.;  // 向右为正

    float pitch_solve = std::atan2(gimbal_points(2, 0), gimbal_points(0, 0)) / M_PI * 180.;
    // 解算弹道模型
    if (solve_ballistic_model(world_points, receive_data, pitch_solve)) {
        fmt::print(
            "最终迭代picth_k={:.3f}度, shoot={}\n", pitch_solve / M_PI * 180.,
            receive_data.shoot_speed);  // 弧度 转 度

        fmt::print("[send]: yaw={:.3f}, pitch={:.3f}\n", send_yaw, pitch_solve);

        send_data.send_pitch = pitch_solve + static_pitch;
        send_data.send_yaw   = send_yaw + static_yaw;
        send_data.goal       = 1;

    } else {
        fmt::print(
            fg(fmt::color::red) | fmt::emphasis::bold, "最终迭代picth_k={:.3f}度, shoot={}\n",
            pitch_solve / M_PI * 180., receive_data.shoot_speed);

        send_data.goal       = 0;

        return false;
    }

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

    // camera_points(1, 0) =   camera_points(1, 0) + 0.07;

    return camera_points;
}

// 解算弹道模型
bool Modules::PredictorEKF::solve_ballistic_model(
    const Eigen::Vector3d & world_points, const Devices::ReceiveData & receive_data,
    float & pitch_res)
{
    // 根据弹道模型求出 pitch角度 和 飞行时间
    // double T_k;  // 飞行时间
    double f_tk, f_tk_;
    double h_k, h_r;
    double e_k;

    // 竖直- Z轴
    double dist_vertical = world_points(2, 0);
    double vertical_tmp  = dist_vertical;

    // 水平- X轴+Y轴
    double dist_horizonal = std::sqrt(
        world_points(0, 0) * world_points(0, 0) + world_points(1, 0) * world_points(1, 0));

    double pitch_0     = std::atan(dist_vertical / dist_horizonal);
    double pitch_solve = pitch_0;

    for (int i = 0; i < max_iter; i++) {
        double x       = 0.0;
        double y       = 0.0;
        double p       = std::tan(pitch_solve);
        double v       = receive_data.shoot_speed;
        double u       = v / std::sqrt(1 + pow(p, 2));
        double delta_x = dist_horizonal / R_K_iter;

        for (int j = 0; j < R_K_iter; j++) {
            double k1_u     = -k * u * sqrt(1 + pow(p, 2));
            double k1_p     = -g / pow(u, 2);
            double k1_u_sum = u + k1_u * (delta_x / 2);
            double k1_p_sum = p + k1_p * (delta_x / 2);

            double k2_u     = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
            double k2_p     = -g / pow(k1_u_sum, 2);
            double k2_u_sum = u + k2_u * (delta_x / 2);
            double k2_p_sum = p + k2_p * (delta_x / 2);

            double k3_u     = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
            double k3_p     = -g / pow(k2_u_sum, 2);
            double k3_u_sum = u + k2_u * (delta_x / 2);
            double k3_p_sum = p + k2_p * (delta_x / 2);

            double k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
            double k4_p = -g / pow(k3_u_sum, 2);

            u += (delta_x / 6) * (k1_u + k2_u + k3_u + k4_u);
            p += (delta_x / 6) * (k1_p + k2_p + k3_p + k4_p);

            x += delta_x;
            y += p * delta_x;
        }
        double error = dist_vertical - y;
        if (fabs(error) < stop_error) {
            break;
        } else {
            vertical_tmp += error;
            pitch_solve = atan(vertical_tmp / dist_horizonal);  // 弧度

            // 避免pitch太大
        }
    }

    if (std::fabs(pitch_solve) > M_PI) {
        return false;
    }

    pitch_res = pitch_solve;

    return true;
}