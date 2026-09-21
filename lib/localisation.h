#ifndef LOCALISATION_H
#define LOCALISATION_H

#include <cstdint>
#include <array>
#include <string>
#include <vector>
#include "motion_history.h"

struct LocScanPoint {
    float angle_deg;
    float distance_mm;  // valid only when hit is true
    int quality;
    bool hit;           // false = explicit no-return / miss at this bearing
    double time_s;      // monotonic acquisition time; -1 for untimed synthetic data
    LocScanPoint(float a=0, float d=0, int q=0, bool h=false, double t=-1)
        : angle_deg(a), distance_mm(d), quality(q), hit(h), time_s(t) {}
};

struct LocDeskewStatus {
    std::string mode = "off", reason = "no_scan";
    double scan_time_s = 0, duration_s = 0, age_s = 0, processing_ms = 0;
    double max_translation_mm = 0, max_rotation_deg = 0;
    double raw_residual_mm = -1, corrected_residual_mm = -1;
    bool history_ok = false, accepted = false;
    std::uint64_t sequence = 0;
};
void loc_configure_deskew(const std::string& mode, double forward=0, double left=0, double yaw=0);
LocDeskewStatus loc_get_deskew_status();
// Each IMU array contains (monotonic seconds, value), in acquisition order.
void loc_feed_motion(double vx, double vy, double time_s, double read_span_s,
                     const std::vector<motion::Value>& yaw,
                     const std::vector<motion::Value>& gyro, std::uint64_t epoch);
void loc_set_replay_time(double time_s); // -1 restores real time; offline only
void loc_seed(unsigned seed);
std::vector<std::array<double, 5>> loc_preview_scan(const std::vector<LocScanPoint>& points,
                                                  double time_s);

struct LocPose {
    float x;
    float y;
    float yaw_deg;
    float confidence;
    bool ok;
};

struct LocParticle {
    float x;
    float y;
    float yaw_deg;
    float weight;
};

// Optional independent floor-colour observations, enabled after a LIDAR fix.
// Sensor 0 is forward, clockwise order, all at radius 75 mm.
struct LocLineReadings {
    std::array<std::string, 32> colours;
    double timestamp_s = 0.0;
    unsigned long long applied_count = 0;
    double last_applied_timestamp_s = 0.0;
    bool valid = false;
};
void loc_set_line_readings(const std::vector<std::string>& colours, double timestamp_s);
void loc_clear_line_readings();
LocLineReadings loc_get_line_readings();

// Odometry-interpolated pose vs LIDAR-corrected pose for the last scan update.
struct LocScanCorrection {
    std::uint64_t sequence;  // increments on each recorded correction
    float predicted_x;
    float predicted_y;
    float predicted_yaw_deg;
    float corrected_x;
    float corrected_y;
    float corrected_yaw_deg;
    float error_mm;
    float yaw_error_deg;
    bool valid;
};

struct LocRecoveryStatus {
    float scan_quality;
    float quality_baseline;
    int bad_scan_count;
    float global_particle_fraction;
    bool baseline_valid;
};

void loc_init_map(float pitch_x, float pitch_y);
void loc_start(bool use_pcb = false);
void loc_stop();
void loc_reset();

void loc_set_imu_yaw(float yaw_deg);
void loc_clear_imu_yaw();
void loc_set_motion_noise(float speed_coefficient);
void loc_predict_odometry(float vx_mm_s, float vy_mm_s, float omega_deg_s, float dt_s);

void loc_update_scan(const LocScanPoint* points, int count,
                     float min_range_mm, float max_range_mm, int min_quality,
                     double scan_time_s = -1.0);

bool loc_scan_updates_allowed();
bool loc_is_ready();
LocPose loc_get_pose();
LocScanCorrection loc_get_last_scan_correction();
LocRecoveryStatus loc_get_recovery_status();
std::vector<LocParticle> loc_get_particles();

#endif
