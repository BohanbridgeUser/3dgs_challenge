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

// 相机读取
void loadCameras(const std::string& caseDir, std::vector<CameraInfo>& cameras_out);

// 视频帧处理（含畸变矫正）
void preprocessFrames(const std::string& caseDir, const CameraInfo& cam);
void extractFrames(const std::string& caseDir, const CameraInfo& cam, std::vector<std::string>& outFramePaths);

// 点云读取
void loadPoints3D(const std::string& file, Eigen::MatrixXd& xyzs, Eigen::MatrixXd& rgbs);

// 单个案例处理
void processCase(const std::string& caseDir, CaseResult& result, bool preprocessFlag = true);

// 批量处理所有案例
void processAllCases(const std::string& rootDir, std::vector<CaseResult>& results, bool preprocessFlag = true);

