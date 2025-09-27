#include "Pipeline.h"
#include "Utils.h"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <omp.h>

using namespace std;
namespace fs = std::filesystem;

// -------------------- 相机读取 --------------------
void loadCameras(const string& caseDir, vector<CameraInfo>& cameras_out) {
    string camFile = caseDir + "/inputs/slam/cameras.txt";
    string imgFile = caseDir + "/inputs/slam/images.txt";

    ifstream fin(camFile);
    if (!fin.is_open()) { cerr << "Cannot open " << camFile << endl; return; }

    int cam_id, width, height;
    string model;
    double fx, fy, cx, cy, k1, k2, k3, p1, p2;
    fin >> cam_id >> model >> width >> height >> fx >> fy >> cx >> cy >> k1 >> k2 >> k3 >> p1 >> p2;
    fin.close();

    double fovX = 2 * atan(width / (2.0 * fx));
    double fovY = 2 * atan(height / (2.0 * fy));

    ifstream fin2(imgFile);
    if (!fin2.is_open()) { cerr << "Cannot open " << imgFile << endl; return; }

    cameras_out.clear();
    int uid, cid;
    double qw, qx, qy, qz, tx, ty, tz;
    string ts, mat, line;

    while (getline(fin2, line)) {
        if (line.empty()) continue; // 跳过空行
        istringstream iss(line);
        if (iss >> uid >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> cid >> ts) {
            getline(iss, mat); // 跳过矩阵行
            CameraInfo ci;
            ci.uid = uid;
            ci.R = quatToMat(qw, qx, qy, qz);
            ci.T = Eigen::Vector3d(tx, ty, tz);
            ci.FovX = fovX; ci.FovY = fovY;
            ci.width = width; ci.height = height;
            ci.image_name = "";
            ci.image_path = "";
            ci.is_test = false;
            ci.fx = fx; ci.fy = fy; ci.cx = cx; ci.cy = cy;
            ci.k1 = k1; ci.k2 = k2; ci.k3 = k3; ci.p1 = p1; ci.p2 = p2;
            cameras_out.push_back(ci);
        }
    }
}

// -------------------- 视频帧预处理 --------------------
void preprocessFrames(const string& caseDir, const CameraInfo& cam, int frameStep){
    fs::path outDir = fs::path(caseDir) / "images";
    if (!fs::exists(outDir))
        fs::create_directories(outDir);

    if (!fs::is_empty(outDir)) {
        cout << "[Skip] Images exist: " << outDir << endl;
        return;
    }

    // 读取时间戳
    fs::path tsFile = fs::path(caseDir) / "inputs" / "videoInfo.txt";
    ifstream finTS(tsFile);
    if (!finTS.is_open()) { cerr << "Cannot open " << tsFile << endl; return; }
    vector<string> timestamps;
    string line;
    while (getline(finTS, line)) {
        if (line.empty()) continue;
        istringstream iss(line);
        string col1, col2;
        iss >> col1 >> col2; // 只取第二列
        timestamps.push_back(col2);
    }
    finTS.close();

    // 跳帧后对应的时间戳索引
    vector<string> selectedTS;
    for (size_t i = 0; i < timestamps.size(); i += frameStep)
        selectedTS.push_back(timestamps[i]);

    // 找视频
    fs::path videoPath;
    for (auto& p : fs::directory_iterator(caseDir))
        if (p.path().extension() == ".mp4") { videoPath = p.path(); break; }
    if (videoPath.empty()) { cerr << "No .mp4 found" << endl; return; }

    cv::VideoCapture cap(videoPath.string());
    if (!cap.isOpened()) { cerr << "Cannot open video" << endl; return; }

    cv::Mat frame;
    if (!cap.read(frame)) { cerr << "Empty video" << endl; return; }

    cv::Size frameSize(frame.cols, frame.rows);
    double scaleX = frame.cols / double(cam.width);
    double scaleY = frame.rows / double(cam.height);

    cv::Mat K = (cv::Mat_<double>(3, 3) << cam.fx * scaleX, 0, cam.cx * scaleX,
        0, cam.fy * scaleY, cam.cy * scaleY,
        0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << cam.k1, cam.k2, cam.p1, cam.p2, cam.k3);
    cv::Mat newK = cv::getOptimalNewCameraMatrix(K, distCoeffs, frameSize, 1, frameSize, 0);

    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(K, distCoeffs, cv::Mat(), newK, frameSize, CV_16SC2, map1, map2);

    size_t frameIndex = 0;// 视频帧索引
    size_t tsIndex = 0; // 已保存的时间戳索引
    vector<cv::Mat> batchFrames;
    const int batchSize = 16;

    auto processBatch = [&](vector<cv::Mat>& frames) {
        size_t n = frames.size();
        vector<cv::Mat> undistorted(n);

        #pragma omp parallel for schedule(dynamic)
        for (size_t i = 0; i < n; i++)
            cv::remap(frames[i], undistorted[i], map1, map2, cv::INTER_LINEAR);

        for (size_t i = 0; i < n; i++) {
            if (tsIndex >= timestamps.size()) break; // 防越界
            fs::path outPath = outDir / (timestamps[tsIndex] + ".jpg");
            cv::imwrite(outPath.string(), undistorted[i]);
            tsIndex++;
        }
        frames.clear();
        };

    do {
        if (frameIndex % frameStep == 0) {  // 按间隔取帧
            batchFrames.push_back(frame.clone());
            if (batchFrames.size() >= batchSize)
                processBatch(batchFrames);
        }
        frameIndex++;
    } while (cap.read(frame));

    if (!batchFrames.empty())
        processBatch(batchFrames);

    cout << "Preprocessed frames for " << caseDir << " (step=" << frameStep << ", saved=" << tsIndex << ")" << endl;
}


// -------------------- 已处理帧读取 --------------------
void extractFrames(const string& caseDir, const CameraInfo& cam, vector<string>& outFramePaths) {
    fs::path outDir = fs::path(caseDir) / "images";
    if (!fs::exists(outDir)) { cerr << "No images found" << endl; return; }

    outFramePaths.clear();
    for (auto& p : fs::directory_iterator(outDir))
        if (p.path().extension() == ".jpg") // JPG
            outFramePaths.push_back(p.path().string());
    sort(outFramePaths.begin(), outFramePaths.end());
}

// -------------------- 点云读取 --------------------
void loadPoints3D(const string& file, Eigen::MatrixXd& xyzs, Eigen::MatrixXd& rgbs) {
    ifstream fin(file);
    if (!fin.is_open()) { cerr << "Cannot open " << file << endl; return; }

    vector<Eigen::Vector3d> pts, colors;
    long id; double x, y, z; int r, g, b; double err;

    while (fin >> id >> x >> y >> z >> r >> g >> b >> err) {
        pts.emplace_back(x, y, z);
        colors.emplace_back(r / 255.0, g / 255.0, b / 255.0);
    }
    fin.close();

    xyzs.resize(pts.size(), 3);
    rgbs.resize(colors.size(), 3);
    for (size_t i = 0; i < pts.size(); i++) {
        xyzs.row(i).noalias() = pts[i];
        rgbs.row(i).noalias() = colors[i];
    }
}

// -------------------- 单个案例处理 --------------------
void processCase(const string& caseDir, CaseResult& result, bool preprocessFlag, int frameStep) {
    result.caseDir = caseDir;
    loadCameras(caseDir, result.cameras);
    if (result.cameras.empty()) return;

    if (preprocessFlag)
        preprocessFrames(caseDir, result.cameras[0], frameStep);

    extractFrames(caseDir, result.cameras[0], result.framePaths);

    fs::path pc_file = fs::path(caseDir) / "inputs" / "slam" / "points3D.txt";
    if (fs::exists(pc_file))
        loadPoints3D(pc_file.string(), result.xyzs, result.rgbs);
}


// -------------------- 批量处理 --------------------
void processAllCases(const string& rootDir, vector<CaseResult>& results, bool preprocessFlag, int frameStep) {
    results.clear();
    for (auto& entry : fs::directory_iterator(rootDir)) {
        if (fs::is_directory(entry)) {
            string caseDir = entry.path().string();
            cout << "Processing case: " << caseDir << endl;
            CaseResult res;
            processCase(caseDir, res, preprocessFlag, frameStep);
            results.push_back(std::move(res));
        }
    }
}


