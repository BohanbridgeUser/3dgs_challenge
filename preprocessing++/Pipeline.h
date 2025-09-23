#pragma once
#include "CameraInfo.h"
#include <Eigen/Dense>
#include <vector>
#include <string>

struct CaseResult {
    std::string caseDir;
    std::vector<CameraInfo> cameras;
    Eigen::MatrixXd xyzs;
    Eigen::MatrixXd rgbs;
    std::vector<std::string> framePaths; // �����֡·��
};

// �����ȡ
void loadCameras(const std::string& caseDir, std::vector<CameraInfo>& cameras_out);

// ��Ƶ֡���������������
void preprocessFrames(const std::string& caseDir, const CameraInfo& cam);
void extractFrames(const std::string& caseDir, const CameraInfo& cam, std::vector<std::string>& outFramePaths);

// ���ƶ�ȡ
void loadPoints3D(const std::string& file, Eigen::MatrixXd& xyzs, Eigen::MatrixXd& rgbs);

// ������������
void processCase(const std::string& caseDir, CaseResult& result, bool preprocessFlag = true);

// �����������а���
void processAllCases(const std::string& rootDir, std::vector<CaseResult>& results, bool preprocessFlag = true);

