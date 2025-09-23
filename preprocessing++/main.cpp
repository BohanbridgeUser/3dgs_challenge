#include "Pipeline.h"
#include <iostream>
using namespace std;

int main() {
    string rootDir = "F:/Final";  // 根目录

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
