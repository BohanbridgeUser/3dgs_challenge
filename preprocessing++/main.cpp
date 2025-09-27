#include "Pipeline.h"
#include <iostream>
#include <cstdlib>
using namespace std;

int main() {
    string rootDir = "F:/Final";  // 根目录
    // 默认每帧抽一次
    int frameStep = 1;
    
    if (argc >= 3) {
    string cmd = argv[1];
    if (cmd == "stepframe") {
        frameStep = atoi(argv[2]);
        if (frameStep <= 0) frameStep = 1; // 防止无效输入
    }
    else {
        cerr << "[Warning] Unknown command: " << cmd << ", using default frameStep=1" << endl;
    }
}
    
    vector<CaseResult> results;
    processAllCases(rootDir, results, true); // 第一次 true，后续可改 false

    for (const auto& res : results) {
        cout << "=== Case: " << res.caseDir << " ===" << endl;
        cout << "Cameras: " << res.cameras.size() << endl;
        cout << "Frames: " << res.framePaths.size() << endl;
        cout << "Points: " << res.xyzs.rows() << endl;
    }
    return 0;
}

