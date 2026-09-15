/*
 * RPLidar C1 Python Module (pybind11)
 *
 * LIDAR scan capture plus Monte Carlo localization (3-DOF: x, y, yaw).
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <cstdio>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <stdexcept>

#include "localisation.h"
#include "scan_timing.h"
#include "sl_lidar_driver.h"

namespace py = pybind11;
using namespace sl;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

static ILidarDriver* g_driver = nullptr;
static IChannel* g_channel = nullptr;
static std::atomic<bool> g_running{false};
static std::thread g_scan_thread;
static LidarScanMode g_scan_mode = {};
static std::mutex g_data_mutex;

struct ScanPoint {
    float angle_deg;
    float distance_mm;
    int quality;
    bool hit;
};

static std::vector<ScanPoint> g_latest_scan;
static double g_latest_scan_time_s = 0.0;
static std::atomic<bool> g_scan_ready{false};
static std::atomic<std::uint64_t> g_scan_generation{0};

static std::atomic<bool> g_loc_running{false};
static std::thread g_loc_thread;

static constexpr float MIN_RANGE_MM = 80.0f;
static constexpr float MAX_RANGE_MM = 6000.0f;
static constexpr int MIN_BEAM_QUALITY = 5;

static double monotonic_time_s() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static void release_driver_resources() {
    if (g_driver) {
        delete g_driver;
        g_driver = nullptr;
    }
    if (g_channel) {
        delete g_channel;
        g_channel = nullptr;
    }
}

static void scan_thread_func() {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    bool reported_bad_timestamp = false;

    while (g_running.load()) {
        size_t count = _countof(nodes);
        sl_u64 first_sample_us = 0;
        sl_result op_result = g_driver->grabScanDataHqWithTimeStamp(
            nodes, count, first_sample_us, 0);
        const double scan_end_time_s = monotonic_time_s();

        if (SL_IS_OK(op_result)) {
            const double midpoint_s = scan_midpoint_s(
                first_sample_us, count, g_scan_mode.us_per_sample, scan_end_time_s);
            if (midpoint_s < 0.0 && !reported_bad_timestamp) {
                std::fprintf(stderr, "LIDAR: invalid acquisition timestamp; "
                             "scan excluded from localisation\n");
            }
            reported_bad_timestamp = midpoint_s < 0.0;
            g_driver->ascendScanData(nodes, count);

            std::vector<ScanPoint> new_scan;
            new_scan.reserve(count);

            for (size_t i = 0; i < count; i++) {
                ScanPoint pt;
                pt.angle_deg = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
                pt.distance_mm = nodes[i].dist_mm_q2 / 4.0f;
                pt.quality = nodes[i].quality
                             >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

                // Keep explicit no-return bearings so MCL can model grazing misses.
                if (nodes[i].dist_mm_q2 == 0 || pt.quality < MIN_BEAM_QUALITY) {
                    pt.hit = false;
                    pt.distance_mm = 0.0f;
                    pt.quality = 0;
                } else {
                    pt.hit = true;
                }
                new_scan.push_back(pt);
            }

            {
                std::lock_guard<std::mutex> lock(g_data_mutex);
                g_latest_scan = std::move(new_scan);
                g_latest_scan_time_s = midpoint_s;
                g_scan_ready.store(true);
                g_scan_generation.fetch_add(1);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

static void localization_thread_func() {
    std::uint64_t last_processed_generation = 0;
    while (g_loc_running.load()) {
        if (!g_scan_ready.load() || !loc_scan_updates_allowed()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        std::uint64_t generation = g_scan_generation.load();
        if (generation == last_processed_generation) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        std::vector<ScanPoint> scan_copy;
        double scan_time_s = 0.0;
        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            scan_copy = g_latest_scan;
            scan_time_s = g_latest_scan_time_s;
            generation = g_scan_generation.load();
        }

        if (scan_copy.empty() || scan_time_s <= 0.0) {
            last_processed_generation = generation;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        std::vector<LocScanPoint> loc_scan(scan_copy.size());
        for (size_t i = 0; i < scan_copy.size(); i++) {
            loc_scan[i].angle_deg = scan_copy[i].angle_deg;
            loc_scan[i].distance_mm = scan_copy[i].distance_mm;
            loc_scan[i].quality = scan_copy[i].quality;
            loc_scan[i].hit = scan_copy[i].hit;
        }

        loc_update_scan(loc_scan.data(), (int)loc_scan.size(),
                        MIN_RANGE_MM, MAX_RANGE_MM, MIN_BEAM_QUALITY,
                        scan_time_s);
        last_processed_generation = generation;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

static bool init_lidar(const std::string& port, int baudrate) {
    if (g_driver != nullptr) {
        throw std::runtime_error(
            "LIDAR already initialized. Call shutdown() first.");
    }

    g_driver = *createLidarDriver();
    if (!g_driver) {
        throw std::runtime_error(
            "Failed to create LIDAR driver (insufficient memory)");
    }

    try {
        g_channel = *createSerialPortChannel(port.c_str(), baudrate);
        if (!g_channel) {
            throw std::runtime_error("Failed to create LIDAR channel");
        }

        if (SL_IS_FAIL(g_driver->connect(g_channel))) {
            throw std::runtime_error(
                "Failed to connect to LIDAR at " + port);
        }

        sl_lidar_response_device_info_t devinfo;
        sl_result op_result = g_driver->getDeviceInfo(devinfo);
        if (SL_IS_FAIL(op_result)) {
            throw std::runtime_error("Failed to get LIDAR device info");
        }

        sl_lidar_response_device_health_t healthinfo;
        op_result = g_driver->getHealth(healthinfo);
        if (SL_IS_FAIL(op_result) ||
            healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            throw std::runtime_error("LIDAR health check failed");
        }

        g_driver->setMotorSpeed();
        g_scan_mode = {};
        if (SL_IS_FAIL(g_driver->startScan(0, 1, 0, &g_scan_mode))) {
            throw std::runtime_error("Failed to start LIDAR scan");
        }
        if (!std::isfinite(g_scan_mode.us_per_sample)
            || g_scan_mode.us_per_sample <= 0.0f) {
            throw std::runtime_error("LIDAR returned invalid scan sample duration");
        }
    } catch (...) {
        release_driver_resources();
        throw;
    }

    g_running.store(true);
    g_scan_thread = std::thread(scan_thread_func);

    printf("LIDAR initialized successfully on %s at %d baud\n",
           port.c_str(), baudrate);
    return true;
}

static void shutdown_lidar() {
    if (g_loc_running.load()) {
        g_loc_running.store(false);
        if (g_loc_thread.joinable()) {
            g_loc_thread.join();
        }
        loc_stop();
    }

    if (g_driver == nullptr) {
        return;
    }

    g_running.store(false);
    if (g_scan_thread.joinable()) {
        g_scan_thread.join();
    }

    g_driver->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    g_driver->setMotorSpeed(0);

    release_driver_resources();

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        g_latest_scan.clear();
        g_latest_scan_time_s = 0.0;
        g_scan_ready.store(false);
    }

    printf("LIDAR shutdown complete\n");
}

static bool is_initialized() {
    return g_driver != nullptr && g_running.load();
}

static bool is_scan_ready() {
    return g_scan_ready.load();
}

static py::array_t<float> get_scan_numpy() {
    std::lock_guard<std::mutex> lock(g_data_mutex);

    size_t hit_count = 0;
    for (const auto& pt : g_latest_scan) {
        if (pt.hit) hit_count++;
    }

    if (hit_count == 0) {
        std::vector<ssize_t> empty_shape = {0, 3};
        return py::array_t<float>(empty_shape);
    }

    std::vector<ssize_t> shape = {(ssize_t)hit_count, 3};
    py::array_t<float> result(shape);
    auto buf = result.mutable_unchecked<2>();

    size_t out = 0;
    for (const auto& pt : g_latest_scan) {
        if (!pt.hit) continue;
        buf(out, 0) = pt.angle_deg;
        buf(out, 1) = pt.distance_mm;
        buf(out, 2) = (float)pt.quality;
        out++;
    }

    return result;
}

static py::list get_scan_list() {
    std::lock_guard<std::mutex> lock(g_data_mutex);

    py::list result;
    for (const auto& pt : g_latest_scan) {
        if (!pt.hit) continue;
        result.append(
            py::make_tuple(pt.angle_deg, pt.distance_mm, pt.quality));
    }
    return result;
}

static float get_distance_at_angle(float target_angle) {
    std::lock_guard<std::mutex> lock(g_data_mutex);

    bool any_hit = false;
    for (const auto& pt : g_latest_scan) {
        if (pt.hit) {
            any_hit = true;
            break;
        }
    }
    if (!any_hit) return -1.0f;

    target_angle = std::fmod(target_angle, 360.0f);
    if (target_angle < 0) target_angle += 360.0f;

    float best_distance = -1.0f;
    float min_angle_diff = 360.0f;

    for (const auto& pt : g_latest_scan) {
        if (!pt.hit) continue;

        float angle = std::fmod(pt.angle_deg, 360.0f);
        if (angle < 0) angle += 360.0f;

        float diff = std::fabs(angle - target_angle);
        if (diff > 180.0f) diff = 360.0f - diff;

        if (diff < min_angle_diff) {
            min_angle_diff = diff;
            best_distance = pt.distance_mm;
        }
    }

    return best_distance;
}

static py::list get_sector_distances(int num_sectors) {
    if (num_sectors <= 0)
        throw std::invalid_argument("num_sectors must be positive");

    std::lock_guard<std::mutex> lock(g_data_mutex);

    float sector_size = 360.0f / num_sectors;
    std::vector<float> min_distances(num_sectors, -1.0f);

    for (const auto& pt : g_latest_scan) {
        if (!pt.hit) continue;

        float angle = std::fmod(pt.angle_deg, 360.0f);
        if (angle < 0) angle += 360.0f;

        int sector = (int)(angle / sector_size);
        if (sector >= num_sectors) sector = num_sectors - 1;

        if (min_distances[sector] < 0 ||
            pt.distance_mm < min_distances[sector]) {
            min_distances[sector] = pt.distance_mm;
        }
    }

    py::list result;
    for (int i = 0; i < num_sectors; i++) {
        float center_angle = (i + 0.5f) * sector_size;
        result.append(py::make_tuple(center_angle, min_distances[i]));
    }
    return result;
}

static int get_scan_count() {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    int hit_count = 0;
    for (const auto& pt : g_latest_scan) {
        if (pt.hit) hit_count++;
    }
    return hit_count;
}

static std::uint64_t get_scan_generation() {
    return g_scan_generation.load();
}

static std::uint64_t get_mcl_update_count() {
    return loc_get_last_scan_correction().sequence;
}

static void start_coordinates(float pitch_x, float pitch_y) {
    if (g_loc_running.load()) {
        throw std::runtime_error("Localization already running.");
    }

    loc_init_map(pitch_x, pitch_y);
    loc_start();

    g_loc_running.store(true);
    g_loc_thread = std::thread(localization_thread_func);
    printf("MCL localization started (pitch %.0f x %.0f mm)\n", pitch_x, pitch_y);
}

static void set_imu_yaw(float yaw_deg) {
    loc_set_imu_yaw(yaw_deg);
}

static void predict_odometry(float vx_mm_s, float vy_mm_s, float omega_deg_s, float dt_s) {
    loc_predict_odometry(vx_mm_s, vy_mm_s, omega_deg_s, dt_s);
}

static py::tuple get_pose_py() {
    LocPose pose = loc_get_pose();
    if (!pose.ok) {
        return py::make_tuple(py::none(), py::none(), py::none(), pose.confidence);
    }
    return py::make_tuple(pose.x, pose.y, pose.yaw_deg, pose.confidence);
}

static py::tuple get_coordinates_py() {
    LocPose pose = loc_get_pose();
    if (!pose.ok) {
        return py::make_tuple(py::none(), py::none());
    }
    return py::make_tuple(pose.x, pose.y);
}

static py::tuple get_coordinates_info_py() {
    LocPose pose = loc_get_pose();
    if (!pose.ok) {
        return py::make_tuple(py::none(), py::none(), py::none(),
                              pose.confidence, pose.ok);
    }
    return py::make_tuple(pose.x, pose.y, pose.yaw_deg, pose.confidence, pose.ok);
}

static bool is_coordinates_ready() {
    return loc_is_ready();
}

static bool scan_updates_enabled() {
    return loc_scan_updates_allowed();
}

static py::tuple get_last_scan_correction_py() {
    LocScanCorrection corr = loc_get_last_scan_correction();
    if (!corr.valid) {
        return py::make_tuple(
            corr.sequence,
            py::none(), py::none(), py::none(),
            py::none(), py::none(), py::none(),
            py::none(), py::none(),
            false);
    }
    return py::make_tuple(
        corr.sequence,
        corr.predicted_x, corr.predicted_y, corr.predicted_yaw_deg,
        corr.corrected_x, corr.corrected_y, corr.corrected_yaw_deg,
        corr.error_mm, corr.yaw_error_deg,
        true);
}

static py::tuple get_recovery_status_py() {
    LocRecoveryStatus status = loc_get_recovery_status();
    return py::make_tuple(
        status.scan_quality,
        status.quality_baseline,
        status.bad_scan_count,
        status.global_particle_fraction,
        status.baseline_valid);
}

// Offline / synthetic MCL helpers (no hardware required).
static void test_mcl_start(float pitch_x, float pitch_y) {
    if (g_loc_running.load()) {
        throw std::runtime_error(
            "Live localization thread is running; shut it down before test_mcl_start.");
    }
    loc_stop();
    loc_init_map(pitch_x, pitch_y);
    loc_start();
}

static void test_mcl_stop() {
    loc_stop();
}

static void test_mcl_set_imu_yaw(float yaw_deg) {
    loc_set_imu_yaw(yaw_deg);
}

static void test_mcl_predict(float vx_mm_s, float vy_mm_s,
                             float omega_deg_s, float dt_s) {
    loc_predict_odometry(vx_mm_s, vy_mm_s, omega_deg_s, dt_s);
}

static void test_mcl_update_scan(const py::list& points) {
    std::vector<LocScanPoint> scan;
    scan.reserve(py::len(points));
    for (const auto& item : points) {
        py::tuple t = item.cast<py::tuple>();
        if (py::len(t) < 4) {
            throw std::invalid_argument(
                "Each scan point must be (angle_deg, distance_mm, quality, hit)");
        }
        LocScanPoint pt;
        pt.angle_deg = t[0].cast<float>();
        pt.distance_mm = t[1].cast<float>();
        pt.quality = t[2].cast<int>();
        pt.hit = t[3].cast<bool>();
        scan.push_back(pt);
    }
    loc_update_scan(scan.data(), (int)scan.size(),
                    MIN_RANGE_MM, MAX_RANGE_MM, MIN_BEAM_QUALITY);
}

static void test_mcl_reset() {
    loc_reset();
}

PYBIND11_MODULE(lidar, m) {
    m.doc() = "RPLidar C1 Python module — scan data and MCL localization";

    m.def("init", &init_lidar,
          py::arg("port") = "/dev/ttyUSB0",
          py::arg("baudrate") = 460800,
          "Initialize the LIDAR. Call once at startup.");

    m.def("shutdown", &shutdown_lidar,
          "Shutdown the LIDAR and localization.");

    m.def("is_initialized", &is_initialized,
          "Check if LIDAR is initialized and running.");

    m.def("is_scan_ready", &is_scan_ready,
          "Check if at least one scan has been captured.");

    m.def("get_scan_numpy", &get_scan_numpy,
          "Get latest scan as numpy array (Nx3: angle_deg, distance_mm, quality).");

    m.def("get_scan_list", &get_scan_list,
          "Get latest scan as list of (angle_deg, distance_mm, quality) tuples.");

    m.def("get_distance_at_angle", &get_distance_at_angle,
          py::arg("angle"),
          "Get distance (mm) at closest angle to target. Returns -1 if no reading.");

    m.def("get_sector_distances", &get_sector_distances,
          py::arg("num_sectors") = 8,
          "Get minimum distances in angular sectors.");

    m.def("get_scan_count", &get_scan_count,
          "Get number of points in latest scan.");

    m.def("get_scan_generation", &get_scan_generation,
          "Monotonic count of completed LIDAR scan captures.");

    m.def("get_mcl_update_count", &get_mcl_update_count,
          "Monotonic count of MCL scan updates applied.");

    m.def("start_coordinates", &start_coordinates,
          py::arg("pitch_x"), py::arg("pitch_y"),
          "Start background MCL localization thread.");

    m.def("clear_imu_yaw", &loc_clear_imu_yaw, "Remove the IMU prior without resetting localisation");
    m.def("set_imu_yaw", &set_imu_yaw,
          py::arg("yaw_deg"),
          "Set startup-relative IMU yaw for the soft MCL yaw prior.");

    m.def("predict_odometry", &predict_odometry,
          py::arg("vx_mm_s"), py::arg("vy_mm_s"),
          py::arg("omega_deg_s"), py::arg("dt_s"),
          "Propagate the particle filter between LIDAR scans.");

    m.def("set_motion_noise", &loc_set_motion_noise,
          py::arg("speed_coefficient"),
          "Set translation speed-noise coefficient in sqrt(seconds). "
          "Default 0.30; use 0 for the legacy stationary-only noise model.");

    m.def("get_pose", &get_pose_py,
          "Get (x, y, yaw_deg, confidence) from MCL.");

    m.def("get_coordinates", &get_coordinates_py,
          "Get (x, y) of the last confident pose, or (None, None).");

    m.def("get_coordinates_info", &get_coordinates_info_py,
          "Get (x, y, yaw_deg, confidence, ok).");

    m.def("is_coordinates_ready", &is_coordinates_ready,
          "True once at least one confident pose has been computed.");

    m.def("scan_updates_enabled", &scan_updates_enabled,
          "True when MCL is accepting LIDAR scans (false during fast rotation).");

    m.def("get_last_scan_correction", &get_last_scan_correction_py,
          "Get (seq, pred_x, pred_y, pred_yaw, corr_x, corr_y, corr_yaw, "
          "error_mm, yaw_error_deg, valid) for the last LIDAR scan update. "
          "error_mm is how far the odometry-interpolated pose was from the "
          "LIDAR-corrected pose.");

    m.def("get_recovery_status", &get_recovery_status_py,
          "Get (scan_quality, baseline, bad_scans, global_fraction, valid) "
          "for MCL recovery diagnostics.");

    m.def("test_mcl_start", &test_mcl_start,
          py::arg("pitch_x"), py::arg("pitch_y"),
          "Start MCL without LIDAR hardware (for synthetic tests).");
    m.def("test_mcl_stop", &test_mcl_stop,
          "Stop synthetic MCL session.");
    m.def("test_mcl_reset", &test_mcl_reset,
          "Reset synthetic MCL particles.");
    m.def("test_mcl_set_imu_yaw", &test_mcl_set_imu_yaw,
          py::arg("yaw_deg"),
          "Set IMU yaw prior for synthetic MCL.");
    m.def("test_mcl_predict", &test_mcl_predict,
          py::arg("vx_mm_s"), py::arg("vy_mm_s"),
          py::arg("omega_deg_s"), py::arg("dt_s"),
          "Propagate synthetic MCL with odometry.");
    m.def("test_mcl_update_scan", &test_mcl_update_scan,
          py::arg("points"),
          "Feed synthetic scan points: list of (angle_deg, distance_mm, quality, hit).");
}
