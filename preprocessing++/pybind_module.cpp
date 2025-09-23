#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "Pipeline.h"

namespace py = pybind11;

// 为 Python 封装 CameraInfo
PYBIND11_MODULE(py_preprocess, m) {
    m.doc() = "Python interface for PREPROCESS pipeline";

    // CameraInfo
    py::class_<CameraInfo>(m, "CameraInfo")
        .def_readonly("uid", &CameraInfo::uid)
        .def_readonly("FovX", &CameraInfo::FovX)
        .def_readonly("FovY", &CameraInfo::FovY)
        .def_readonly("width", &CameraInfo::width)
        .def_readonly("height", &CameraInfo::height)
        .def_readonly("image_path", &CameraInfo::image_path)
        .def_readonly("image_name", &CameraInfo::image_name)
        .def_readonly("fx", &CameraInfo::fx)
        .def_readonly("fy", &CameraInfo::fy)
        .def_readonly("cx", &CameraInfo::cx)
        .def_readonly("cy", &CameraInfo::cy)
        .def_readonly("k1", &CameraInfo::k1)
        .def_readonly("k2", &CameraInfo::k2)
        .def_readonly("k3", &CameraInfo::k3)
        .def_readonly("p1", &CameraInfo::p1)
        .def_readonly("p2", &CameraInfo::p2)
        .def_readonly("is_test", &CameraInfo::is_test);

    // CaseResult
    py::class_<CaseResult>(m, "CaseResult")
        .def_readonly("caseDir", &CaseResult::caseDir)
        .def_readonly("cameras", &CaseResult::cameras)
        .def_readonly("framePaths", &CaseResult::framePaths)
        .def_readonly("xyzs", &CaseResult::xyzs)
        .def_readonly("rgbs", &CaseResult::rgbs);

    // 处理单个案例
    m.def("process_case", [](const std::string& caseDir, bool preprocessFlag = true) {
        CaseResult res;
        processCase(caseDir, res, preprocessFlag);
        return res;
        }, py::arg("caseDir"), py::arg("preprocessFlag") = true,
            "Process a single case and return CaseResult");

    // 批量处理多个案例
    m.def("process_all_cases", [](const std::string& rootDir, bool preprocessFlag = true) {
        std::vector<CaseResult> results;
        processAllCases(rootDir, results, preprocessFlag);
        return results;
        }, py::arg("rootDir"), py::arg("preprocessFlag") = true,
            "Process all cases in rootDir and return list of CaseResult");
}
