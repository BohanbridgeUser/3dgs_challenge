#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

inline Eigen::Matrix3d quatToMat(double qw, double qx, double qy, double qz) {
    Eigen::Quaterniond q(qw, qx, qy, qz);//������Ԫ������
    q.normalize();//��һ��
    return q.toRotationMatrix();//ת��Ϊ��ת����
}

