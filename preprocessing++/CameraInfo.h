#pragma once
#include <Eigen/Dense>
#include <string>

struct CameraInfo {
    int uid;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    double FovY;
    double FovX;
    int width;
    int height;
    std::string image_path;
    std::string image_name;
    bool is_test;

    // 内参和畸变参数,方便抽帧畸变处理
    double fx, fy, cx, cy;
    double k1, k2, k3, p1, p2;
};

