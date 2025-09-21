#include "Pipeline.h"
#include <iostream>
using namespace std;

int main() {
    string rootDir = "F:/Final";  // ¸ùÄ¿Â¼
    auto results = processAllCases(rootDir);

    for (const auto& res : results) {
        cout << "=== Case: " << res.caseDir << " ===" << endl;
        cout << "Cameras: " << res.cameras.size() << endl;
        cout << "Frames: " << res.framePaths.size() << endl;
        cout << "Points: " << res.xyzs.rows() << endl;
    }
    return 0;
}
