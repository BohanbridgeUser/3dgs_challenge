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
    std::vector<std::string> framePaths; // 保存抽帧路径
};

// -------------------- 功能1：相机读取 --------------------
std::vector<CameraInfo> loadCameras(const std::string& caseDir);

// -------------------- 功能2：抽帧 + 畸变矫正 --------------------
void extractFrames(const std::string& caseDir, const CameraInfo& cam, std::vector<std::string>& outFramePaths);

// -------------------- 功能3：点云读取 --------------------
void loadPoints3D(const std::string& file, Eigen::MatrixXd& xyzs, Eigen::MatrixXd& rgbs);

// -------------------- 单个案例处理 --------------------
CaseResult processCase(const std::string& caseDir, bool extractFramesFlag = true);

// -------------------- 批量处理所有案例 --------------------
std::vector<CaseResult> processAllCases(const std::string& rootDir, bool extractFramesFlag = true);


