#include "Pipeline.h"
#include "Utils.h"
#include "CameraInfo.h"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;
namespace fs = std::filesystem;

//功能1: 相机读取
vector<CameraInfo> loadCameras(const string& caseDir) {
    //caseDir为输入文件夹，如F:\Final\1747834320424
    //拼接生成相机内外参文件完整路径
    string camFile = caseDir + "/inputs/slam/cameras.txt";
    string imgFile = caseDir + "/inputs/slam/images.txt";

    // 读取相机内参
    ifstream fin(camFile);//打开相机内参文件
    if (!fin.is_open()) {
        cerr << "Cannot open " << camFile << endl;
        return {};
    }

    int cam_id, width, height;//相机ID，图像宽高
    string model;//相机模型名称
    double fx, fy, cx, cy, k1, k2, k3, p1, p2;//焦距|光心|畸变系数
    fin >> cam_id >> model >> width >> height >> fx >> fy >> cx >> cy
        >> k1 >> k2 >> k3 >> p1 >> p2;//提取内参
    fin.close();//显式关闭相机内参文件

    //计算水平竖直视场角
    double fovX = 2 * atan(width / (2.0 * fx));
    double fovY = 2 * atan(height / (2.0 * fy));

    // 读取外参
    ifstream fin2(imgFile);//打开相机外参文件
    if (!fin2.is_open()) {
        cerr << "Cannot open " << imgFile << endl;
        return {};
    }

    vector<CameraInfo> cameras_info;//创建向量
    int uid, cid;
    double qw, qx, qy, qz, tx, ty, tz;
    string ts, mat, line;

    while (getline(fin2, line)) {//循环读取
        if (line.empty()) continue;
        istringstream iss(line);//创建字符串流对象
        if (iss >> uid >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> cid >> ts) {
            getline(iss, mat);// 跳过矩阵部分
            CameraInfo ci;
            ci.uid = uid;
            ci.R = quatToMat(qw, qx, qy, qz);//转换为旋转矩阵
            ci.T = Eigen::Vector3d(tx, ty, tz);//设置平移向量
            ci.FovX = fovX;
            ci.FovY = fovY;
            ci.width = width;
            ci.height = height;
            ci.image_name = "frame_" + to_string(uid) + ".png";
            ci.image_path = caseDir + "/images/" + ci.image_name;
            ci.is_test = false;

            ci.fx = fx;
            ci.fy = fy;
            ci.cx = cx;
            ci.cy = cy;
            ci.k1 = k1;
            ci.k2 = k2;
            ci.k3 = k3;
            ci.p1 = p1;
            ci.p2 = p2;//内参+畸变

            cameras_info.push_back(ci);
        }
    }
    return cameras_info;
}

//功能2: 视频帧提取
void extractFrames(const string& caseDir, const CameraInfo& cam, vector<string>& outFramePaths) {
    fs::path casePath(caseDir);
    fs::path videoPath;

    // 自动查找 mp4 视频
    for (auto& p : fs::directory_iterator(casePath)) {
        if (p.path().extension() == ".mp4") {
            videoPath = p.path();
            break;
        }
    }
    if (videoPath.empty()) {
        cerr << "No .mp4 found in " << caseDir << endl;
        return;
    }

    // 输出目录
    fs::path outDir = casePath / "frames";
    fs::create_directories(outDir);

    // 打开视频
    cv::VideoCapture cap(videoPath.string());
    if (!cap.isOpened()) {
        cerr << "Cannot open " << videoPath << endl;
        return;
    }

    cv::Mat frame;
    // 读取第一帧，确定视频尺寸
    if (!cap.read(frame)) {
        std::cerr << "Video is empty: " << videoPath << std::endl;
        return;
    }
    cv::Size frameSize(frame.cols, frame.rows);

    // 按视频分辨率缩放内参
    double scaleX = frame.cols / double(cam.width);
    double scaleY = frame.rows / double(cam.height);
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        cam.fx * scaleX, 0, cam.cx * scaleX,
        0, cam.fy * scaleY, cam.cy * scaleY,
        0, 0, 1);

    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << cam.k1, cam.k2, cam.p1, cam.p2, cam.k3);

    // 生成新的相机矩阵，alpha=1 保留全部像素
    cv::Mat newK = cv::getOptimalNewCameraMatrix(K, distCoeffs, frameSize, 1, frameSize, 0);

    // 一次生成 map1/map2
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(K, distCoeffs, cv::Mat(), newK, frameSize, CV_16SC2, map1, map2);

    int frame_idx = 0;
    cv::Mat undistorted;

    // 处理第一帧
    cv::remap(frame, undistorted, map1, map2, cv::INTER_LINEAR);
    fs::path outPath = outDir / ("frame_" + to_string(frame_idx++) + ".png");
    cv::imwrite(outPath.string(), undistorted);
    outFramePaths.push_back(outPath.string());

    // 处理剩余帧
    while (cap.read(frame)) {
        cv::remap(frame, undistorted, map1, map2, cv::INTER_LINEAR);
        outPath = outDir / ("frame_" + to_string(frame_idx++) + ".png");
        cv::imwrite(outPath.string(), undistorted);
        outFramePaths.push_back(outPath.string());
    }



    cout << "Extracted " << frame_idx << " undistorted frames for case " << caseDir << endl;
}

//功能3: 点云读取
void loadPoints3D(const string& file, Eigen::MatrixXd& xyzs, Eigen::MatrixXd& rgbs) {
    ifstream fin(file);
    if (!fin.is_open()) {
        cerr << "Cannot open " << file << endl;
        return;
    }

    vector<Eigen::Vector3d> pts, colors;
    long id;
    double x, y, z;
    int r, g, b;
    double err;

    while (fin >> id >> x >> y >> z >> r >> g >> b >> err) {
        pts.emplace_back(x, y, z);
        colors.emplace_back(r / 255.0, g / 255.0, b / 255.0);//颜色归一化
    }
    fin.close();

    xyzs.resize(pts.size(), 3);
    rgbs.resize(colors.size(), 3);
    for (size_t i = 0; i < pts.size(); i++) {
        xyzs.row(i) = pts[i];
        rgbs.row(i) = colors[i];
    }
}

//单个案例处理
CaseResult processCase(const string& caseDir, bool extractFramesFlag) {
    CaseResult result;
    result.caseDir = caseDir;

    // 相机
    result.cameras = loadCameras(caseDir);
    if (result.cameras.empty()) return result;

    // 帧提取
    if (extractFramesFlag) {
        extractFrames(caseDir, result.cameras[0], result.framePaths);
    }

    // 点云
    fs::path pc_file = fs::path(caseDir) / "inputs" / "slam" / "points3D.txt";
    if (fs::exists(pc_file)) {
        loadPoints3D(pc_file.string(), result.xyzs, result.rgbs);
    }

    return result;
}

//批量案例处理
vector<CaseResult> processAllCases(const string& rootDir, bool extractFramesFlag) {
    vector<CaseResult> allResults;
    for (auto& entry : fs::directory_iterator(rootDir)) {
        if (fs::is_directory(entry)) {
            string caseDir = entry.path().string();
            cout << "Processing case: " << caseDir << endl;
            allResults.push_back(processCase(caseDir, extractFramesFlag));
        }
    }
    return allResults;
}
