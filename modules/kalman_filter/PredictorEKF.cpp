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
    Devices::SendData & send_data)
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
    double pitch = receive_data.pitch;

    Eigen::Matrix3d R_WI;
    R_WI = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

    // 得到3个坐标系下的坐标
    Eigen::Vector3d camera_points = get_camera_points(pts, select_armour.armour_type);
    Eigen::Vector3d i_points      = R_CI * camera_points;
    Eigen::Vector3d world_points  = pc2pw(camera_points, R_WI.transpose());

    // 相机坐标系，
    // x轴向右，y轴向下，z轴延相机向前方
    fmt::print(
        "[相机坐标系]: {:.3f},{:.3f},{:.3f}\n", camera_points(0, 0), camera_points(1, 0),
        camera_points(2, 0));
    //x轴延枪管向前，y轴向左，z轴向上
    fmt::print(
        "[陀螺仪坐标系]: {:.3f},{:.3f},{:.3f}\n", i_points(0, 0), i_points(1, 0), i_points(2, 0));
    //xy平面平行于地面，z轴垂直地面
    //x轴延枪管向前，y轴向左，z轴向上
    fmt::print(
        "[世界坐标系]: {:.3f},{:.3f},{:.3f}\n", world_points(0, 0), world_points(1, 0),
        world_points(2, 0));

    ekf.predict(predictfunc);
    // x,y,z ,v_x, v_y, v_z
    VectorX predict_status = ekf.update(measure, world_points);

    // 求解发送的yaw和pitch角度
    double send_pitch = std::atan2(i_points(2, 0), i_points(0, 0));   // 向上为正
    double send_yaw   = -std::atan2(i_points(1, 0), i_points(0, 0));  // 向右为正

    fmt::print("[send]: yaw={:.3f}, pitch={:.3f}\n", send_yaw, send_pitch);

    send_data.send_pitch = send_pitch;
    send_data.send_yaw   = send_yaw;
    send_data.goal = 1;

    return true;
}

Eigen::Vector3d Modules::PredictorEKF::get_camera_points(
    std::vector<cv::Point2f> & armour_points, Robot::ArmourType type)
{
    cv::Mat tvec, rvec;
    if (type == Robot::ArmourType::Big) {
        cv::solvePnP(big_obj, armour_points, F_MAT, C_MAT, rvec, tvec);
    } else {
        cv::solvePnP(big_obj, armour_points, F_MAT, C_MAT, rvec, tvec);
    }

    Eigen::Vector3d camera_points;

    cv::cv2eigen(tvec, camera_points);

    return camera_points;
}