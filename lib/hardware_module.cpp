#include "hardware_controller.h"

#include <filesystem>
#include <map>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using hardware::HardwareController;

namespace {
std::unique_ptr<HardwareController> from_addresses(
    const std::vector<int>& addresses, double diameter, double max_yaw_rpm,
    double max_rpm, double yaw_correct_threshold, const std::string& calibration_file,
    const std::string& i2c_device, int imu_address, int imu_report_interval_ms,
    int kicker_pin, const std::string& kicker_gpiochip, double drive_motor_current_limit,
    double dribbler_motor_current_limit, double kick_pulse_length, double kick_cooldown,
    std::shared_ptr<hardware::StatusDisplay> display) {
    // File parsing runs once with the GIL. All motor I/O and control are native C++.
    std::filesystem::path path(calibration_file);
    if (path.is_relative()) {
        const auto module_path = py::module_::import("lib.hardware_controller").attr("__file__").cast<std::string>();
        path = std::filesystem::path(module_path).parent_path().parent_path() / path;
    }
    const auto text = py::module_::import("pathlib").attr("Path")(path.string()).attr("read_text")();
    const auto data = py::module_::import("json").attr("loads")(text);
    if (!py::isinstance<py::dict>(data) || !data.contains("motors") ||
        !py::isinstance<py::list>(data["motors"]))
        throw py::value_error("Calibration must contain a motors list");
    std::map<int, hardware::MotorCalibration> by_address;
    try {
        for (const auto item : data["motors"]) {
            if (!py::isinstance<py::dict>(item))
                throw py::value_error("Each motor calibration must be an object");
            const auto row = py::reinterpret_borrow<py::dict>(item);
            hardware::MotorCalibration cal{row["address"].cast<int>(),
                row["elecangleoffset"].cast<uint32_t>(), row["sincoscentre"].cast<int32_t>()};
            if (!by_address.emplace(cal.address, cal).second)
                throw py::value_error("Duplicate motor address in calibration file");
        }
    } catch (const py::cast_error&) {
        throw py::value_error("Invalid motor calibration; expected integer address, elecangleoffset and sincoscentre");
    }
    std::vector<hardware::MotorCalibration> calibration;
    for (int address : addresses) {
        const auto found = by_address.find(address);
        if (found == by_address.end())
            throw py::value_error("Missing calibration for motor address " + std::to_string(address) +
                                  "; run calibration/motors.py");
        calibration.push_back(found->second);
    }
    py::gil_scoped_release release;
    return std::make_unique<HardwareController>(calibration,
        hardware::DriveConfig{diameter, max_yaw_rpm, max_rpm, yaw_correct_threshold}, i2c_device,
        nullptr, imu_address, imu_report_interval_ms, kicker_pin, kicker_gpiochip, nullptr,
        drive_motor_current_limit, dribbler_motor_current_limit, kick_pulse_length, kick_cooldown, std::move(display));
}
}
PYBIND11_MODULE(hardware_controller, module) {
    module.doc() = "Native hardware controller: motors, BNO08x IMU and GPIO kicker";
    py::register_exception<hardware::MotorCommunicationError>(module, "MotorCommunicationError");
    py::class_<hardware::StatusDisplay, std::shared_ptr<hardware::StatusDisplay>>(module, "StatusDisplay")
        .def(py::init<const std::string&, int>(), py::arg("i2c_device") = "/dev/i2c-1",
             py::arg("address") = 0x3c)
        .def("update", &hardware::StatusDisplay::update, py::arg("mode"), py::arg("run"),
             py::arg("state"), py::arg("detail") = "")
        .def("component", &hardware::StatusDisplay::component, py::arg("source"),
             py::arg("health"), py::arg("error") = "")
        .def_property_readonly("error", &hardware::StatusDisplay::error)
        .def("stop", &hardware::StatusDisplay::stop, py::call_guard<py::gil_scoped_release>());
    py::class_<HardwareController>(module, "HardwareController")
        .def_static("from_i2c_addresses", &from_addresses,
            py::arg("i2c_addresses"), py::arg("diameter"), py::arg("max_yaw_rpm"),
            py::arg("max_rpm"), py::arg("yaw_correct_threshold"),
            py::arg("calibration_file") = "calibration_data.json",
            py::arg("i2c_device") = "/dev/i2c-1",
            py::arg("imu_address") = 0x4a, py::arg("imu_report_interval_ms") = 10,
            py::arg("kicker_pin") = -1, py::arg("kicker_gpiochip") = "",
            py::arg("drive_motor_current_limit") = 8.0,
            py::arg("dribbler_motor_current_limit") = 1.0,
            py::arg("kick_pulse_length") = 0.02, py::arg("kick_cooldown") = 0.5,
            py::arg("display") = nullptr)
        .def("move", &HardwareController::move, py::arg("direction"), py::arg("speed"),
             py::arg("rotation"), py::arg("rotation_speed"),
             py::arg("dribbler") = 0, py::arg("kick") = false,
             py::call_guard<py::gil_scoped_release>())
        .def("health", [](const HardwareController& self) {
            const auto h = self.health();
            py::dict result;
            result["imu_healthy"] = h.imu_healthy;
            result["imu_recovery_generation"] = h.imu_recovery_generation;
            result["fault_source"] = h.fault_source;
            result["error"] = h.error; result["motor_address"] = h.motor_address;
            return result;
        })
        .def("get_raw_imu_yaw", &HardwareController::get_raw_imu_yaw)
        .def("set_drive_current_limits", &HardwareController::set_drive_current_limits,
             py::arg("constant_speed_amps"), py::arg("acceleration_amps"),
             py::call_guard<py::gil_scoped_release>())
        .def("set_startup_yaw", &HardwareController::set_startup_yaw, py::arg("raw_yaw"))
        .def("get_yaw", &HardwareController::get_yaw)
        .def("get_gyro_z_deg_s", &HardwareController::get_gyro_z_deg_s)
        .def("get_latest_quaternion", &HardwareController::get_latest_quaternion)
        .def_property_readonly("imu_update_count", &HardwareController::imu_update_count)
        .def("get_measured_body_velocity_mm_s", &HardwareController::get_measured_body_velocity_mm_s,
             py::arg("yaw_deg"), py::call_guard<py::gil_scoped_release>())
        .def("get_dribbler_rpm", &HardwareController::get_dribbler_rpm,
             py::call_guard<py::gil_scoped_release>())
        .def("stop", &HardwareController::stop, py::call_guard<py::gil_scoped_release>())
        .def_property_readonly("loop_count", &HardwareController::loop_count)
        .def_property_readonly("current_speed", &HardwareController::current_speed)
        .def_property_readonly("current_direction", &HardwareController::current_direction)
        .def("__enter__", [](HardwareController& self) -> HardwareController& { return self; },
             py::return_value_policy::reference_internal)
        .def("__exit__", [](HardwareController& self, py::object, py::object, py::object) {
            py::gil_scoped_release release;
            self.stop();
        });
}
