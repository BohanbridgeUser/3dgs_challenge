#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

inline Eigen::Matrix3d quatToMat(double qw, double qx, double qy, double qz) {
    Eigen::Quaterniond q(qw, qx, qy, qz);//创建四元数对象
    q.normalize();//归一化
    return q.toRotationMatrix();//转换为旋转矩阵
}

