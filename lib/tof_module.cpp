// Independent sparse-range instance of the shared localisation engine.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <cmath>
#include "localisation.h"
namespace py = pybind11;
PYBIND11_MODULE(tof_native, m) {
    m.def("start", [](float x, float y) { loc_init_map(x, y); loc_start(false); });
    m.def("stop", &loc_stop);
    m.def("reset", &loc_reset);
    m.def("set_imu_yaw", &loc_set_imu_yaw);
    m.def("predict_odometry", &loc_predict_odometry);
    m.def("get_coordinates_info", []() {
        auto p = loc_get_pose();
        return py::make_tuple(p.x, p.y, p.yaw_deg, p.confidence, p.ok);
    });
    m.def("update", [](const std::vector<std::array<float, 4>>& readings,
                       float min_range, float max_range) {
        std::vector<LocScanPoint> scan;
        for (const auto& r : readings) {
            if (!std::isfinite(r[0]) || !std::isfinite(r[1]) ||
                !std::isfinite(r[2]) || !std::isfinite(r[3]))
                throw std::invalid_argument("ToF readings must be finite");
            LocScanPoint p(r[0], r[1], 35, true);
            p.origin_forward_mm = r[2]; p.origin_right_mm = r[3];
            scan.push_back(p);
        }
        py::gil_scoped_release release;
        loc_update_scan(scan.data(), scan.size(), min_range, max_range, 5);
        return loc_get_deskew_status().accepted;
    }, py::arg("readings"), py::arg("min_range") = 40, py::arg("max_range") = 4000);
}
