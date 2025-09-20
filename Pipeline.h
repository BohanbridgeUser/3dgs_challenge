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

// -------------------- ����1�������ȡ --------------------
std::vector<CameraInfo> loadCameras(const std::string& caseDir);

// -------------------- ����2����֡ + ������� --------------------
void extractFrames(const std::string& caseDir, const CameraInfo& cam, std::vector<std::string>& outFramePaths);

// -------------------- ����3�����ƶ�ȡ --------------------
void loadPoints3D(const std::string& file, Eigen::MatrixXd& xyzs, Eigen::MatrixXd& rgbs);

// -------------------- ������������ --------------------
CaseResult processCase(const std::string& caseDir, bool extractFramesFlag = true);

// -------------------- �����������а��� --------------------
std::vector<CaseResult> processAllCases(const std::string& rootDir, bool extractFramesFlag = true);


