#ifndef LOCALISATION_H
#define LOCALISATION_H

#include <cstdint>
#include <vector>

struct LocScanPoint {
    float angle_deg;
    float distance_mm;  // valid only when hit is true
    int quality;
    bool hit;           // false = explicit no-return / miss at this bearing
};

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
void loc_start();
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
