#include "Pipeline.h"
#include <iostream>
using namespace std;

int main() {
    string rootDir = "F:/Final";  // ��Ŀ¼

    vector<CaseResult> results;
    processAllCases(rootDir, results, true); // ��һ�� true�������ɸ� false

    for (const auto& res : results) {
        cout << "=== Case: " << res.caseDir << " ===" << endl;
        cout << "Cameras: " << res.cameras.size() << endl;
        cout << "Frames: " << res.framePaths.size() << endl;
        cout << "Points: " << res.xyzs.rows() << endl;
    }
    return 0;
}
