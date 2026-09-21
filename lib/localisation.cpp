#include "localisation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <random>
#include <stdexcept>
#include <vector>

struct Segment {
    float x1;
    float y1;
    float x2;
    float y2;
};
static LocLineReadings g_line_readings;
static bool g_use_pcb = false;
static double g_last_line_used_timestamp = 0;
static void apply_pending_line_readings_locked(double now);

struct Particle {
    float x;
    float y;
    float yaw_deg;
    float weight;
};

struct PredictedHit {
    float range_mm;
    float nx;
    float ny;
};

struct Observation {
    float angle_deg;
    float distance_mm;
    float weight;
    bool hit;
};

struct OdometryStep {
    double start_time_s;
    double end_time_s;
    float vx_mm_s;
    float vy_mm_s;
    float omega_deg_s;
};

static std::vector<Segment> g_static_segments;
static std::vector<Particle> g_particles;
static std::deque<OdometryStep> g_odometry_history;
static std::mutex g_loc_mutex;
static std::mt19937 g_rng(42);

static float g_pitch_x = 2430.0f;
static float g_pitch_y = 1820.0f;
static LocPose g_pose = {0.0f, 0.0f, 0.0f, 0.0f, false};
static LocScanCorrection g_last_scan_correction = {};
static bool g_started = false;
static bool g_ready = false;
static bool g_scan_updates_paused = false;
static float g_last_omega_deg_s = 0.0f;
static float g_omega_below_resume_s = 0.0f;
static float g_imu_yaw_deg = 0.0f;
static bool g_imu_yaw_valid = false;
static bool g_scan_quality_baseline_valid = false;
static float g_scan_quality_baseline = 0.0f;
static float g_last_scan_quality = 0.0f;
static int g_bad_scan_count = 0;
static float g_recovery_fraction = 0.0f;

static constexpr float COORD_SIGMA = 30.0f;
static constexpr float COORD_EPS = 1e-9f;
static constexpr float INLIER_THRESH = 80.0f;
static constexpr float CONF_ACQUIRE_THRESHOLD = 0.5f;
static constexpr float CONF_TRACK_THRESHOLD = 0.35f;
static constexpr int PARTICLE_COUNT = 1000;
static constexpr int MIN_OBSERVATION_COUNT = 30;
static constexpr int MIN_HIT_COUNT = 8;
static constexpr int ANGLE_BIN_COUNT = 180;  // 2° bins over 360°
static constexpr float ANGLE_BIN_DEG = 360.0f / ANGLE_BIN_COUNT;
static constexpr float TRANS_NOISE_MM = 8.0f;
// sqrt(seconds): moving at 500 mm/s gives about 47.5 mm of standard
// deviation per axis over 100 ms. Stationary diffusion remains 8 mm/sqrt(s).
static float g_speed_noise_coefficient = 0.30f;
static constexpr float YAW_NOISE_DEG = 2.0f;
// Absolute IMU yaw is accurate enough to strongly constrain MCL heading.
// Keep initialization especially tight so particles search position rather
// than wasting samples across headings that the IMU has already ruled out.
static constexpr float YAW_PRIOR_SIGMA_DEG = 8.0f;
static constexpr float YAW_INIT_SIGMA_DEG = 5.0f;
static constexpr float EXPLORATION_FRACTION = 0.02f;
static constexpr float RECOVERY_FRACTION_LOW = 0.20f;
static constexpr float RECOVERY_FRACTION_HIGH = 0.50f;
static constexpr int RECOVERY_LOW_BAD_SCANS = 2;
static constexpr int RECOVERY_HIGH_BAD_SCANS = 5;
static constexpr int RECOVERY_RESET_BAD_SCANS = 10;
static constexpr float RECOVERY_QUALITY_DROP = 1.5f;
static constexpr float QUALITY_BASELINE_RISE_ALPHA = 0.20f;
static constexpr float QUALITY_BASELINE_FALL_ALPHA = 0.002f;
static constexpr float ESS_RESAMPLE_FRACTION = 0.5f;
static constexpr float OMEGA_PAUSE_DEG_S = 50.0f;
static constexpr float OMEGA_RESUME_DEG_S = 25.0f;
static constexpr float OMEGA_SETTLE_S = 0.15f;
static constexpr double ODOMETRY_HISTORY_S = 2.0;

static float wrap_angle_deg(float angle);
static float rand_normal(float stddev);

static double monotonic_time_s() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static void propagate_particle(Particle& particle, float vx_mm_s, float vy_mm_s,
                               float omega_deg_s, float dt_s, bool add_noise) {
    float yaw_rad = particle.yaw_deg * (float)(M_PI / 180.0);
    float cos_yaw = std::cos(yaw_rad);
    float sin_yaw = std::sin(yaw_rad);
    float dx = (vx_mm_s * cos_yaw + vy_mm_s * sin_yaw) * dt_s;
    float dy = (vx_mm_s * sin_yaw - vy_mm_s * cos_yaw) * dt_s;
    float noise_dt_scale = std::sqrt(std::max(dt_s, 0.0f));
    const float translation_sigma = add_noise
        ? std::hypot(TRANS_NOISE_MM,
                     g_speed_noise_coefficient * std::hypot(vx_mm_s, vy_mm_s))
            * noise_dt_scale
        : 0.0f;

    particle.x = std::min(std::max(
        particle.x + dx
        + (add_noise ? rand_normal(translation_sigma) : 0.0f),
        0.0f), g_pitch_x);
    particle.y = std::min(std::max(
        particle.y + dy
        + (add_noise ? rand_normal(translation_sigma) : 0.0f),
        0.0f), g_pitch_y);
    particle.yaw_deg = wrap_angle_deg(
        particle.yaw_deg + omega_deg_s * dt_s
        + (add_noise ? rand_normal(YAW_NOISE_DEG * noise_dt_scale) : 0.0f));
}

static void rewind_particle(Particle& particle, const OdometryStep& step,
                            float dt_s) {
    float start_yaw = wrap_angle_deg(particle.yaw_deg - step.omega_deg_s * dt_s);
    float yaw_rad = start_yaw * (float)(M_PI / 180.0);
    float dx = (step.vx_mm_s * std::cos(yaw_rad)
                + step.vy_mm_s * std::sin(yaw_rad)) * dt_s;
    float dy = (step.vx_mm_s * std::sin(yaw_rad)
                - step.vy_mm_s * std::cos(yaw_rad)) * dt_s;
    particle.x = std::min(std::max(particle.x - dx, 0.0f), g_pitch_x);
    particle.y = std::min(std::max(particle.y - dy, 0.0f), g_pitch_y);
    particle.yaw_deg = start_yaw;
}

// Incidence model: |cos| below this is treated as fully grazing / expected miss.
static constexpr float GRAZING_COS = 0.25f;       // ~75.5° from normal
static constexpr float HEADON_COS = 0.70f;        // ~45.5° from normal
static constexpr float INCIDENCE_SIGMA_FLOOR = 0.20f;
static constexpr float OUTLIER_MIX = 0.05f;
static constexpr float MISS_EXPECTED_P = 0.85f;
static constexpr float MISS_UNEXPECTED_P = 0.08f;
static constexpr float RELIABLE_HIT_RANGE_MM = 3500.0f;
static constexpr float SPREAD_X_SCALE_MM = 120.0f;
static constexpr float SPREAD_Y_SCALE_MM = 120.0f;
static constexpr float SPREAD_YAW_SCALE_DEG = 15.0f;

// Physical goal walls
static constexpr float GOAL_LEFT_BACK_X = 226.0f;
static constexpr float GOAL_RIGHT_BACK_X = 2204.0f;
static constexpr float GOAL_LEFT_FRONT_X = 300.0f;
static constexpr float GOAL_RIGHT_FRONT_X = 2130.0f;
static constexpr float GOAL_TOP_Y = 685.0f;
static constexpr float GOAL_BOTTOM_Y = 1135.0f;
static constexpr float GOAL_BACK_BOTTOM_Y = 1140.0f;
// Goal interiors occupy only about 1.5% of the map. Pure uniform sampling
// leaves too few particles there for reliable acquisition or recovery.
static constexpr float GOAL_PARTICLE_FRACTION = 0.20f;

static float wrap_angle_deg(float angle) {
    while (angle >= 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float normalize_angle_360(float angle) {
    angle = std::fmod(angle, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle;
}

static float rand_uniform(float lo, float hi) {
    std::uniform_real_distribution<float> dist(lo, hi);
    return dist(g_rng);
}

static float rand_normal(float stddev) {
    std::normal_distribution<float> dist(0.0f, stddev);
    return dist(g_rng);
}

static inline float clamp01(float v) {
    return std::max(0.0f, std::min(1.0f, v));
}

// Visibility in [0,1]: 1 = expect a reliable return, 0 = expect a miss.
static inline float wall_visibility(float abs_cos_inc, float range_mm) {
    float incidence = clamp01((abs_cos_inc - GRAZING_COS) / (HEADON_COS - GRAZING_COS));
    float range_factor = 1.0f;
    if (range_mm > RELIABLE_HIT_RANGE_MM) {
        range_factor = clamp01(
            1.0f - (range_mm - RELIABLE_HIT_RANGE_MM) / RELIABLE_HIT_RANGE_MM);
    }
    return incidence * range_factor;
}

static inline bool ray_segment_intersection(float px, float py,
                                            float ux, float uy,
                                            const Segment& seg,
                                            float* out_t) {
    float sx = seg.x2 - seg.x1;
    float sy = seg.y2 - seg.y1;

    float denom = ux * sy - uy * sx;
    if (std::fabs(denom) <= COORD_EPS) {
        return false;
    }

    float qpx = seg.x1 - px;
    float qpy = seg.y1 - py;

    float t = (qpx * sy - qpy * sx) / denom;
    float u = (qpx * uy - qpy * ux) / denom;

    if (t > COORD_EPS && u >= -COORD_EPS && u <= 1.0f + COORD_EPS) {
        *out_t = t;
        return true;
    }
    return false;
}

static inline PredictedHit predict_hit(float x, float y, float ux, float uy,
                                       float Lx, float Ly) {
    PredictedHit best;
    best.range_mm = 1e30f;
    best.nx = 0.0f;
    best.ny = 0.0f;

    auto consider = [&](float t, float nx, float ny) {
        if (t > COORD_EPS && t < best.range_mm) {
            best.range_mm = t;
            // Prefer the inward-facing normal (toward the robot).
            float toward_robot_x = -ux;
            float toward_robot_y = -uy;
            if (nx * toward_robot_x + ny * toward_robot_y < 0.0f) {
                nx = -nx;
                ny = -ny;
            }
            best.nx = nx;
            best.ny = ny;
        }
    };

    // Outer pitch walls (axis-aligned), normals pointing into the field.
    if (ux < -COORD_EPS) {
        consider(-x / ux, 1.0f, 0.0f);
    } else if (ux > COORD_EPS) {
        consider((Lx - x) / ux, -1.0f, 0.0f);
    }
    if (uy < -COORD_EPS) {
        consider(-y / uy, 0.0f, 1.0f);
    } else if (uy > COORD_EPS) {
        consider((Ly - y) / uy, 0.0f, -1.0f);
    }

    for (const auto& seg : g_static_segments) {
        float t = 0.0f;
        if (!ray_segment_intersection(x, y, ux, uy, seg, &t)) {
            continue;
        }
        float sx = seg.x2 - seg.x1;
        float sy = seg.y2 - seg.y1;
        float len = std::sqrt(sx * sx + sy * sy);
        if (len <= COORD_EPS) {
            continue;
        }
        // Segment normal candidates; orientation fixed inside consider().
        consider(t, -sy / len, sx / len);
    }

    return best;
}

static float score_pose(float x, float y, float yaw_deg,
                        const Observation* obs, int n,
                        float Lx, float Ly, float max_range_mm) {
    float psi = yaw_deg * (float)(M_PI / 180.0);
    float log_lik = 0.0f;
    const float inv_max_range = 1.0f / std::max(max_range_mm, 1.0f);

    for (int i = 0; i < n; i++) {
        float theta = psi + obs[i].angle_deg * (float)(M_PI / 180.0);
        float ux = std::cos(theta);
        float uy = std::sin(theta);
        PredictedHit pred = predict_hit(x, y, ux, uy, Lx, Ly);
        float abs_cos = 0.0f;
        if (pred.range_mm < 1e29f) {
            abs_cos = std::fabs(ux * pred.nx + uy * pred.ny);
        }
        float visibility = wall_visibility(abs_cos, pred.range_mm);

        if (obs[i].hit) {
            float sigma = COORD_SIGMA / std::max(abs_cos, INCIDENCE_SIGMA_FLOOR);
            float e = (obs[i].distance_mm - pred.range_mm) / sigma;
            // Soft-cap extreme residuals via mixture rather than hard clamp alone.
            float p_hit = std::exp(-0.5f * e * e);
            float p = (1.0f - OUTLIER_MIX) * p_hit + OUTLIER_MIX * inv_max_range;
            // Grazing returns are down-weighted: they are often noisy or wrong-surface.
            float beam_w = obs[i].weight * (0.25f + 0.75f * visibility);
            log_lik += beam_w * std::log(std::max(p, 1e-12f));
        } else {
            // Explicit miss: expected when grazing or far; suspicious when close/head-on.
            float p_miss = MISS_EXPECTED_P * (1.0f - visibility)
                           + MISS_UNEXPECTED_P * visibility;
            log_lik += obs[i].weight * std::log(std::max(p_miss, 1e-12f));
        }
    }
    return log_lik;
}

struct PoseStats {
    float inlier_ratio;
    float mad_mm;
    float normal_x_energy;
    float normal_y_energy;
    int hit_count;
};

static PoseStats compute_stats(float x, float y, float yaw_deg,
                               const Observation* obs, int n,
                               float Lx, float Ly) {
    float psi = yaw_deg * (float)(M_PI / 180.0);
    std::vector<float> inlier_errors;
    inlier_errors.reserve(n);
    int inlier_count = 0;
    int hit_count = 0;
    float normal_x_energy = 0.0f;
    float normal_y_energy = 0.0f;
    float weight_sum = 0.0f;

    for (int i = 0; i < n; i++) {
        if (!obs[i].hit) {
            continue;
        }
        hit_count++;
        float theta = psi + obs[i].angle_deg * (float)(M_PI / 180.0);
        float ux = std::cos(theta);
        float uy = std::sin(theta);
        PredictedHit pred = predict_hit(x, y, ux, uy, Lx, Ly);
        float e = std::fabs(obs[i].distance_mm - pred.range_mm);
        float abs_cos = std::fabs(ux * pred.nx + uy * pred.ny);
        float visibility = wall_visibility(abs_cos, pred.range_mm);
        float w = obs[i].weight * visibility;
        weight_sum += w;
        if (e < INLIER_THRESH) {
            inlier_count++;
            inlier_errors.push_back(e);
            // Accumulate visible geometry axes (inlier + non-grazing only).
            normal_x_energy += w * std::fabs(pred.nx);
            normal_y_energy += w * std::fabs(pred.ny);
        }
    }

    PoseStats stats;
    stats.hit_count = hit_count;
    stats.inlier_ratio = (float)inlier_count / std::max(hit_count, 1);
    stats.normal_x_energy = normal_x_energy;
    stats.normal_y_energy = normal_y_energy;

    if (inlier_errors.empty()) {
        stats.mad_mm = 1e9f;
    } else {
        std::sort(inlier_errors.begin(), inlier_errors.end());
        stats.mad_mm = 1.4826f * inlier_errors[inlier_errors.size() / 2];
    }

    // Normalize normal energies so geometry score is scale-free.
    float denom = std::max(weight_sum, 1e-6f);
    stats.normal_x_energy /= denom;
    stats.normal_y_energy /= denom;
    return stats;
}

struct ParticleSpread {
    float std_x;
    float std_y;
    float std_yaw_deg;
};

static ParticleSpread compute_particle_spread(const std::vector<Particle>& particles) {
    float sum_w = 0.0f;
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;

    for (const auto& particle : particles) {
        sum_w += particle.weight;
        mean_x += particle.weight * particle.x;
        mean_y += particle.weight * particle.y;
        float yaw_rad = particle.yaw_deg * (float)(M_PI / 180.0);
        sin_sum += particle.weight * std::sin(yaw_rad);
        cos_sum += particle.weight * std::cos(yaw_rad);
    }

    ParticleSpread spread = {1e9f, 1e9f, 1e9f};
    if (sum_w <= 0.0f) {
        return spread;
    }

    mean_x /= sum_w;
    mean_y /= sum_w;
    float mean_yaw = std::atan2(sin_sum, cos_sum) * (180.0f / (float)M_PI);

    float var_x = 0.0f;
    float var_y = 0.0f;
    float var_yaw = 0.0f;
    for (const auto& particle : particles) {
        float w = particle.weight / sum_w;
        float dx = particle.x - mean_x;
        float dy = particle.y - mean_y;
        float dyaw = wrap_angle_deg(particle.yaw_deg - mean_yaw);
        var_x += w * dx * dx;
        var_y += w * dy * dy;
        var_yaw += w * dyaw * dyaw;
    }

    spread.std_x = std::sqrt(var_x);
    spread.std_y = std::sqrt(var_y);
    spread.std_yaw_deg = std::sqrt(var_yaw);
    return spread;
}

static float compute_confidence(const PoseStats& s, const ParticleSpread& spread) {
    float inlier_conf = 0.7f * s.inlier_ratio + 0.3f * std::exp(-s.mad_mm / 80.0f);

    // One visible wall constrains only the normal axis; require diversity.
    float axis_x = clamp01(s.normal_x_energy * 2.0f);
    float axis_y = clamp01(s.normal_y_energy * 2.0f);
    float geometry_conf = std::sqrt(std::max(axis_x * axis_y, 0.0f));
    // Partial scans still get some credit if one strong axis + tight yaw.
    geometry_conf = std::max(geometry_conf, 0.35f * std::max(axis_x, axis_y));

    float spread_conf =
        std::exp(-spread.std_x / SPREAD_X_SCALE_MM)
        * std::exp(-spread.std_y / SPREAD_Y_SCALE_MM)
        * std::exp(-spread.std_yaw_deg / SPREAD_YAW_SCALE_DEG);

    float conf = 0.45f * inlier_conf + 0.35f * geometry_conf + 0.20f * spread_conf;
    if (s.hit_count < MIN_HIT_COUNT) {
        conf *= 0.5f;
    }
    return clamp01(conf);
}

// Sample init/recovery yaw: IMU-centered when available, else full circle.
static float rand_init_yaw_deg() {
    if (g_imu_yaw_valid) {
        return wrap_angle_deg(g_imu_yaw_deg + rand_normal(YAW_INIT_SIGMA_DEG));
    }
    return rand_uniform(-180.0f, 180.0f);
}

static void sample_position(Particle& particle, float goal_fraction) {
    float region = rand_uniform(0.0f, 1.0f);
    if (region < goal_fraction * 0.5f) {
        particle.x = rand_uniform(GOAL_LEFT_BACK_X, GOAL_LEFT_FRONT_X);
        particle.y = rand_uniform(GOAL_TOP_Y, GOAL_BOTTOM_Y);
    } else if (region < goal_fraction) {
        particle.x = rand_uniform(GOAL_RIGHT_FRONT_X, GOAL_RIGHT_BACK_X);
        particle.y = rand_uniform(GOAL_TOP_Y, GOAL_BOTTOM_Y);
    } else {
        particle.x = rand_uniform(0.0f, g_pitch_x);
        particle.y = rand_uniform(0.0f, g_pitch_y);
    }
}

static void init_particles_uniform() {
    g_particles.resize(PARTICLE_COUNT);
    const float weight = 1.0f / PARTICLE_COUNT;
    for (auto& particle : g_particles) {
        sample_position(particle, GOAL_PARTICLE_FRACTION);
        particle.yaw_deg = rand_init_yaw_deg();
        particle.weight = weight;
    }
}

static void inject_random_particles(float fraction) {
    int count = std::max(1, (int)std::round(fraction * PARTICLE_COUNT));
    const float weight = 1.0f / PARTICLE_COUNT;
    for (int i = 0; i < count; i++) {
        int idx = (int)rand_uniform(0.0f, (float)(PARTICLE_COUNT - 1));
        const bool search_nearby =
            g_recovery_fraction > EXPLORATION_FRACTION
            && rand_uniform(0.0f, 1.0f) < 0.70f;
        if (search_nearby) {
            // Broaden the existing position hypothesis at the scan timestamp.
            constexpr float recovery_sigma_mm = 150.0f;
            g_particles[idx].x = std::min(
                std::max(g_particles[idx].x + rand_normal(recovery_sigma_mm), 0.0f),
                g_pitch_x);
            g_particles[idx].y = std::min(
                std::max(g_particles[idx].y + rand_normal(recovery_sigma_mm), 0.0f),
                g_pitch_y);
        } else {
            sample_position(g_particles[idx], GOAL_PARTICLE_FRACTION);
        }
        g_particles[idx].yaw_deg = rand_init_yaw_deg();
        g_particles[idx].weight = weight;
    }
}

static void reset_recovery_state() {
    g_scan_quality_baseline_valid = false;
    g_scan_quality_baseline = 0.0f;
    g_last_scan_quality = 0.0f;
    g_bad_scan_count = 0;
    g_recovery_fraction = 0.0f;
}

static void update_recovery_state(float scan_quality) {
    g_last_scan_quality = scan_quality;
    if (!g_scan_quality_baseline_valid) {
        g_scan_quality_baseline = scan_quality;
        g_scan_quality_baseline_valid = true;
        g_bad_scan_count = 0;
        g_recovery_fraction = 0.0f;
        return;
    }

    const bool poor_match =
        scan_quality < g_scan_quality_baseline - RECOVERY_QUALITY_DROP;
    if (poor_match) {
        g_bad_scan_count++;
    } else {
        g_bad_scan_count = 0;
    }

    const float baseline_alpha =
        scan_quality >= g_scan_quality_baseline
            ? QUALITY_BASELINE_RISE_ALPHA
            : QUALITY_BASELINE_FALL_ALPHA;
    g_scan_quality_baseline +=
        baseline_alpha * (scan_quality - g_scan_quality_baseline);

    if (g_bad_scan_count >= RECOVERY_HIGH_BAD_SCANS) {
        g_recovery_fraction = RECOVERY_FRACTION_HIGH;
    } else if (g_bad_scan_count >= RECOVERY_LOW_BAD_SCANS) {
        g_recovery_fraction = RECOVERY_FRACTION_LOW;
    } else {
        g_recovery_fraction = 0.0f;
    }
}

static float effective_sample_size() {
    float sum_sq = 0.0f;
    for (const auto& particle : g_particles) {
        sum_sq += particle.weight * particle.weight;
    }
    if (sum_sq <= 0.0f) {
        return 0.0f;
    }
    return 1.0f / sum_sq;
}

static void resample_particles() {
    std::vector<Particle> new_particles(PARTICLE_COUNT);
    std::uniform_real_distribution<float> dist(0.0f, 1.0f / PARTICLE_COUNT);
    float step = 1.0f / PARTICLE_COUNT;
    float cursor = dist(g_rng);

    int index = 0;
    float cumulative = g_particles[0].weight;
    for (int i = 0; i < PARTICLE_COUNT; i++) {
        while (cursor > cumulative && index < PARTICLE_COUNT - 1) {
            index++;
            cumulative += g_particles[index].weight;
        }
        new_particles[i] = g_particles[index];
        new_particles[i].weight = step;
        cursor += step;
    }
    g_particles.swap(new_particles);
}

static LocPose estimate_pose_from_particles(const Observation* obs, int n,
                                             const std::vector<Particle>& particles = g_particles) {
    float sum_w = 0.0f;
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;

    for (const auto& particle : particles) {
        sum_w += particle.weight;
        mean_x += particle.weight * particle.x;
        mean_y += particle.weight * particle.y;
        float yaw_rad = particle.yaw_deg * (float)(M_PI / 180.0);
        sin_sum += particle.weight * std::sin(yaw_rad);
        cos_sum += particle.weight * std::cos(yaw_rad);
    }

    LocPose pose;
    if (sum_w <= 0.0f) {
        pose.ok = false;
        pose.confidence = 0.0f;
        return pose;
    }

    mean_x /= sum_w;
    mean_y /= sum_w;
    pose.x = mean_x;
    pose.y = mean_y;
    pose.yaw_deg = wrap_angle_deg(std::atan2(sin_sum, cos_sum) * (180.0f / (float)M_PI));

    if (obs != nullptr && n > 0) {
        PoseStats stats = compute_stats(mean_x, mean_y, pose.yaw_deg,
                                        obs, n, g_pitch_x, g_pitch_y);
        ParticleSpread spread = compute_particle_spread(particles);
        pose.confidence = compute_confidence(stats, spread);
        // Require strong evidence for the first global fix, but tolerate a
        // short confidence dip once tracking is established. Without this
        // hysteresis, callers stop the robot and appear to restart global
        // localization whenever one partial scan falls just below threshold.
        const float threshold =
            g_ready ? CONF_TRACK_THRESHOLD : CONF_ACQUIRE_THRESHOLD;
        pose.ok = pose.confidence >= threshold;
    } else {
        pose.confidence = g_pose.confidence;
        pose.ok = g_pose.ok;
    }
    return pose;
}

static void reset_rotation_gate() {
    g_scan_updates_paused = false;
    g_last_omega_deg_s = 0.0f;
    g_omega_below_resume_s = 0.0f;
}

static void update_rotation_gate(float omega_deg_s, float dt_s) {
    g_last_omega_deg_s = omega_deg_s;
    float abs_omega = std::fabs(omega_deg_s);

    if (abs_omega > OMEGA_PAUSE_DEG_S) {
        g_scan_updates_paused = true;
        g_omega_below_resume_s = 0.0f;
        return;
    }

    if (!g_scan_updates_paused) {
        g_omega_below_resume_s = 0.0f;
        return;
    }

    if (abs_omega < OMEGA_RESUME_DEG_S) {
        g_omega_below_resume_s += dt_s;
        if (g_omega_below_resume_s >= OMEGA_SETTLE_S) {
            g_scan_updates_paused = false;
            g_omega_below_resume_s = 0.0f;
        }
    } else {
        g_omega_below_resume_s = 0.0f;
    }
}

static std::vector<Observation> bin_observations(const LocScanPoint* points, int count,
                                                 float min_range_mm, float max_range_mm,
                                                 int min_quality) {
    struct BinAccum {
        bool observed = false;
        bool hit = false;
        float distance_mm = 0.0f;
        int quality = 0;
    };

    std::vector<BinAccum> bins(ANGLE_BIN_COUNT);
    for (int i = 0; i < count; i++) {
        const LocScanPoint& pt = points[i];
        int bin = (int)(normalize_angle_360(pt.angle_deg) / ANGLE_BIN_DEG);
        if (bin < 0) bin = 0;
        if (bin >= ANGLE_BIN_COUNT) bin = ANGLE_BIN_COUNT - 1;

        BinAccum& b = bins[bin];
        b.observed = true;

        bool is_hit = pt.hit
                      && pt.quality >= min_quality
                      && pt.distance_mm >= min_range_mm
                      && pt.distance_mm <= max_range_mm;
        if (!is_hit) {
            // Explicit miss only sticks if this bin has no valid hit yet.
            continue;
        }

        if (!b.hit || pt.quality > b.quality
            || (pt.quality == b.quality && pt.distance_mm < b.distance_mm)) {
            b.hit = true;
            b.distance_mm = pt.distance_mm;
            b.quality = pt.quality;
        }
    }

    // Second pass: mark bins that were observed only via misses.
    for (int i = 0; i < count; i++) {
        const LocScanPoint& pt = points[i];
        int bin = (int)(normalize_angle_360(pt.angle_deg) / ANGLE_BIN_DEG);
        if (bin < 0) bin = 0;
        if (bin >= ANGLE_BIN_COUNT) bin = ANGLE_BIN_COUNT - 1;
        BinAccum& b = bins[bin];
        if (b.hit) {
            continue;
        }
        bool is_hit = pt.hit
                      && pt.quality >= min_quality
                      && pt.distance_mm >= min_range_mm
                      && pt.distance_mm <= max_range_mm;
        if (!is_hit) {
            b.observed = true;
            b.hit = false;
        }
    }

    std::vector<Observation> obs;
    obs.reserve(ANGLE_BIN_COUNT);
    for (int bin = 0; bin < ANGLE_BIN_COUNT; bin++) {
        const BinAccum& b = bins[bin];
        if (!b.observed) {
            continue;
        }
        Observation o;
        o.angle_deg = (bin + 0.5f) * ANGLE_BIN_DEG;
        o.hit = b.hit;
        o.distance_mm = b.distance_mm;
        if (b.hit) {
            float w = (float)(b.quality - min_quality) / 30.0f;
            if (w > 1.0f) w = 1.0f;
            if (w < 0.05f) w = 0.05f;
            o.weight = w;
        } else {
            o.weight = 0.6f;  // miss observations are slightly softer than strong hits
        }
        obs.push_back(o);
    }
    return obs;
}

void loc_init_map(float pitch_x, float pitch_y) {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    g_pitch_x = pitch_x;
    g_pitch_y = pitch_y;
    g_static_segments.clear();

    // Outer walls are handled analytically in predict_hit(); only goal hardware here.
    g_static_segments.push_back({GOAL_LEFT_FRONT_X, GOAL_TOP_Y, 0.0f, GOAL_TOP_Y});
    g_static_segments.push_back({GOAL_LEFT_BACK_X, GOAL_TOP_Y, GOAL_LEFT_BACK_X, GOAL_BACK_BOTTOM_Y});
    g_static_segments.push_back({0.0f, GOAL_BOTTOM_Y, GOAL_LEFT_FRONT_X, GOAL_BOTTOM_Y});
    g_static_segments.push_back({GOAL_RIGHT_FRONT_X, GOAL_TOP_Y, pitch_x, GOAL_TOP_Y});
    g_static_segments.push_back({GOAL_RIGHT_BACK_X, GOAL_TOP_Y, GOAL_RIGHT_BACK_X, GOAL_BACK_BOTTOM_Y});
    g_static_segments.push_back({pitch_x, GOAL_BOTTOM_Y, GOAL_RIGHT_FRONT_X, GOAL_BOTTOM_Y});
}

void loc_clear_imu_yaw() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    g_imu_yaw_valid = false;
}

void loc_set_imu_yaw(float yaw_deg) {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    g_imu_yaw_deg = wrap_angle_deg(yaw_deg);
    g_imu_yaw_valid = true;
}

void loc_start(bool use_pcb) {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    g_use_pcb = use_pcb;
    g_last_line_used_timestamp = 0;
    g_line_readings = {};
    init_particles_uniform();
    g_odometry_history.clear();
    reset_recovery_state();
    g_pose = {0.0f, 0.0f, 0.0f, 0.0f, false};
    g_last_scan_correction = {};
    g_ready = false;
    g_started = true;
    reset_rotation_gate();
}

void loc_stop() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    g_use_pcb = false;
    g_last_line_used_timestamp = 0;
    g_line_readings = {};
    g_started = false;
    g_ready = false;
    g_imu_yaw_valid = false;
    g_particles.clear();
    g_odometry_history.clear();
    reset_recovery_state();
    g_pose = {0.0f, 0.0f, 0.0f, 0.0f, false};
    g_last_scan_correction = {};
    reset_rotation_gate();
}

void loc_reset() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    g_last_line_used_timestamp = 0;
    g_line_readings = {};
    if (!g_started) {
        return;
    }
    init_particles_uniform();
    g_odometry_history.clear();
    reset_recovery_state();
    g_pose = {0.0f, 0.0f, 0.0f, 0.0f, false};
    g_last_scan_correction = {};
    g_ready = false;
    reset_rotation_gate();
}

void loc_set_motion_noise(float speed_coefficient) {
    if (!std::isfinite(speed_coefficient) || speed_coefficient < 0.0f) {
        throw std::invalid_argument("Motion noise coefficient must be finite and nonnegative");
    }
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    g_speed_noise_coefficient = speed_coefficient;
}

void loc_predict_odometry(float vx_mm_s, float vy_mm_s, float omega_deg_s, float dt_s) {
    if (dt_s <= 0.0f) {
        return;
    }

    std::lock_guard<std::mutex> lock(g_loc_mutex);
    if (!g_started || g_particles.empty()) {
        return;
    }

    update_rotation_gate(omega_deg_s, dt_s);

    // Process noise is specified per sqrt(second), so diffusion remains
    // independent of how often predict_odometry() is called.
    for (auto& particle : g_particles) {
        propagate_particle(particle, vx_mm_s, vy_mm_s, omega_deg_s, dt_s, true);
    }

    const double end_time_s = monotonic_time_s();
    // In PCB mode, consecutive history intervals must meet even if lock wait
    // or scoring time changes between calls. Scale the recorded rates so the
    // whole interval exactly reproduces the motion integrated with caller dt.
    // Keep the legacy timestamp path untouched when PCB is disabled.
    const double start_time_s = g_use_pcb && !g_odometry_history.empty()
        ? g_odometry_history.back().end_time_s : end_time_s - dt_s;
    const double history_scale = g_use_pcb ? dt_s / (end_time_s - start_time_s) : 1.0;
    g_odometry_history.push_back({
        start_time_s, end_time_s, float(vx_mm_s * history_scale),
        float(vy_mm_s * history_scale), float(omega_deg_s * history_scale)
    });
    while (!g_odometry_history.empty()
           && g_odometry_history.front().end_time_s
                  < end_time_s - ODOMETRY_HISTORY_S) {
        g_odometry_history.pop_front();
    }

    if (g_ready) {
        LocPose updated = estimate_pose_from_particles(nullptr, 0);
        g_pose.x = updated.x;
        g_pose.y = updated.y;
        g_pose.yaw_deg = updated.yaw_deg;
    }
    apply_pending_line_readings_locked(end_time_s);
}

static float floor_segment_distance_sq(float x, float y, float ax, float ay, float bx, float by) {
    const float dx = bx - ax, dy = by - ay;
    const float t = std::max(0.0f, std::min(1.0f, ((x-ax)*dx + (y-ay)*dy) / (dx*dx + dy*dy)));
    const float ex = x - ax - t*dx, ey = y - ay - t*dy;
    return ex*ex + ey*ey;
}

static int floor_colour(float x, float y) {
    // White rectangle is drawn inward from its outer edge, with 50 mm width.
    if (x >= 250 && x <= g_pitch_x-250 && y >= 250 && y <= g_pitch_y-250 &&
        (x <= 300 || x >= g_pitch_x-300 || y <= 300 || y >= g_pitch_y-300))
        return 2;
    for (float offset : {-300.0f, 0.0f, 300.0f}) {
        const float dx = x-g_pitch_x/2, dy = y-g_pitch_y/2-offset;
        if (dx*dx + dy*dy <= 100) return 0;
    }
    const float top = g_pitch_y/2-450, bottom = g_pitch_y/2+450;
    for (float side : {x, g_pitch_x-x}) {
        if (side < 290 || side > 620 || y < top-10 || y > bottom+20) continue;
        if (floor_segment_distance_sq(side,y,300,top,610,top) <= 100 ||
            floor_segment_distance_sq(side,y,600,top,600,bottom+10) <= 100 ||
            floor_segment_distance_sq(side,y,600,bottom,300,bottom) <= 100) return 0;
    }
    return 1;
}

static Particle line_pose_at_time(Particle p, double from, double to) {
    if (to >= from) {
        for (const auto& step : g_odometry_history) {
            const double dt = std::min(to, step.end_time_s) - std::max(from, step.start_time_s);
            if (dt > 0) propagate_particle(p, step.vx_mm_s, step.vy_mm_s, step.omega_deg_s, dt, false);
        }
    } else {
        for (auto it = g_odometry_history.rbegin(); it != g_odometry_history.rend(); ++it) {
            const double dt = std::min(from, it->end_time_s) - std::max(to, it->start_time_s);
            if (dt > 0) rewind_particle(p, *it, dt);
        }
    }
    return p;
}

static float score_floor_colours(const Particle& p, const LocLineReadings& sample) {
    float score = 0;
    for (size_t i = 0; i < sample.colours.size(); ++i) {
        const float angle = (p.yaw_deg + i*11.25f) * (3.14159265358979323846f / 180.0f);
        const float x = p.x + 75*std::cos(angle), y = p.y + 75*std::sin(angle);
        const int observed = sample.colours[i] == "black" ? 0 : sample.colours[i] == "white" ? 2 : 1;
        int matches = 0;
        // A 10 mm footprint softens paint edges and small placement errors.
        for (float dx : {-10.0f, 0.0f, 10.0f})
            for (float dy : {-10.0f, 0.0f, 10.0f})
                matches += floor_colour(x+dx,y+dy) == observed;
        score += std::log(0.05f + 0.85f * matches/9.0f);
    }
    // Nearby sensors are correlated: cap the entire ring at four observations.
    return score * (4.0f / 32.0f);
}

static void apply_pending_line_readings_locked(double now) {
    if (!g_use_pcb || !g_started || !g_ready || g_particles.empty() ||
        !g_line_readings.valid || g_line_readings.timestamp_s <= g_last_line_used_timestamp)
        return;
    const double timestamp = g_line_readings.timestamp_s;
    if (timestamp > now || now - timestamp > 0.5 ||
        g_odometry_history.empty() || timestamp < g_odometry_history.front().start_time_s) {
        g_line_readings.valid = false;
        return;
    }
    const double present = g_odometry_history.back().end_time_s;
    if (timestamp > present) return;  // Keep newest sample until prediction catches up.
    const double interval = g_last_line_used_timestamp > 0
        ? timestamp - g_last_line_used_timestamp : 0.02;
    const float strength = std::min(interval / 0.1, 1.0);
    float max_log = -INFINITY;
    for (auto& particle : g_particles) {
        const Particle observed = line_pose_at_time(particle, present, timestamp);
        particle.weight = std::log(std::max(particle.weight, 1e-30f)) +
            strength * score_floor_colours(observed, g_line_readings);
        max_log = std::max(max_log, particle.weight);
    }
    float sum = 0;
    for (auto& particle : g_particles) {
        particle.weight = std::exp(particle.weight - max_log);
        sum += particle.weight;
    }
    for (auto& particle : g_particles) particle.weight /= sum;
    // Publish the weighted estimate before resampling introduces sampling noise.
    // No observations here: retain LIDAR's confidence and validity unchanged.
    g_pose = estimate_pose_from_particles(nullptr, 0);
    if (effective_sample_size() < ESS_RESAMPLE_FRACTION * PARTICLE_COUNT)
        resample_particles();
    g_last_line_used_timestamp = timestamp;
    ++g_line_readings.applied_count;
    g_line_readings.last_applied_timestamp_s = timestamp;
}

void loc_update_scan(const LocScanPoint* points, int count,
                     float min_range_mm, float max_range_mm, int min_quality,
                     double scan_time_s) {
    {
        std::lock_guard<std::mutex> lock(g_loc_mutex);
        if (g_scan_updates_paused || !g_started || g_particles.empty()) {
            return;
        }
    }

    std::vector<Observation> obs = bin_observations(
        points, count, min_range_mm, max_range_mm, min_quality);

    int hit_count = 0;
    for (const auto& o : obs) {
        if (o.hit) hit_count++;
    }

    const int n = (int)obs.size();
    if (n < MIN_OBSERVATION_COUNT || hit_count < MIN_HIT_COUNT) {
        return;
    }

    std::lock_guard<std::mutex> lock(g_loc_mutex);
    if (g_scan_updates_paused || !g_started || g_particles.empty()) {
        return;
    }

    // Score the scan at its acquisition time, not at processing time. A completed
    // rotating scan is already old by the time this thread receives it.
    // Untimestamped synthetic scans use -1. A real scan must fit the retained
    // history; partially rewinding an older scan would associate it with a
    // position newer than its acquisition time.
    if (!std::isfinite(scan_time_s)
        || (scan_time_s >= 0.0
            && (scan_time_s == 0.0 || scan_time_s > monotonic_time_s()
                || (!g_odometry_history.empty()
                    && scan_time_s < g_odometry_history.front().start_time_s)))) {
        return;
    }
    const LocPose predicted_pose = g_pose;
    const bool had_prior_pose = g_ready && g_pose.ok;
    const bool compensate_delay =
        scan_time_s > 0.0 && !g_odometry_history.empty();

    if (compensate_delay && !g_use_pcb) {
        for (auto step_it = g_odometry_history.rbegin();
             step_it != g_odometry_history.rend(); ++step_it) {
            if (step_it->end_time_s <= scan_time_s) {
                break;
            }
            const double overlap_start =
                std::max(step_it->start_time_s, scan_time_s);
            const float replay_dt =
                (float)(step_it->end_time_s - overlap_start);
            for (auto& particle : g_particles) {
                rewind_particle(particle, *step_it, replay_dt);
            }
        }
    }

    // Keep a small global exploration set even while tracking. If several
    // scans fit every current particle substantially worse than the recent
    // healthy baseline, increase this set aggressively. This lets LIDAR
    // recover from slip or an unreported move without disabling odometry
    // prediction between scans.
    if (g_bad_scan_count >= RECOVERY_RESET_BAD_SCANS) {
        init_particles_uniform();
        g_ready = false;
        reset_recovery_state();
    } else {
        inject_random_particles(std::max(
            EXPLORATION_FRACTION, g_recovery_fraction));
    }

    // Store log-weights first, then log-sum-exp so relative weights stay
    // usable even when absolute log-likelihoods underflow float exp().
    float max_log = -1e30f;
    float max_lidar_log = -1e30f;
    const double present = g_odometry_history.empty() ? monotonic_time_s()
        : g_odometry_history.back().end_time_s;
    for (auto& particle : g_particles) {
        const Particle observed = g_use_pcb && compensate_delay
            ? line_pose_at_time(particle, present, scan_time_s) : particle;
        float log_lik = score_pose(observed.x, observed.y, observed.yaw_deg,
                                   obs.data(), n, g_pitch_x, g_pitch_y, max_range_mm);
        if (g_imu_yaw_valid) {
            float yaw_err = wrap_angle_deg(observed.yaw_deg - g_imu_yaw_deg);
            float e = yaw_err / YAW_PRIOR_SIGMA_DEG;
            log_lik += -0.5f * e * e;
        }
        // Preserve the existing LIDAR/IMU recovery metric, excluding floor evidence.
        max_lidar_log = std::max(max_lidar_log, log_lik);
        if (g_use_pcb) log_lik += std::log(std::max(particle.weight, 1e-30f));
        particle.weight = log_lik;
        if (log_lik > max_log) {
            max_log = log_lik;
        }
    }

    float observation_weight_sum = 0.0f;
    for (const auto& observation : obs) {
        observation_weight_sum += observation.weight;
    }
    const float scan_quality =
        max_lidar_log / std::max(observation_weight_sum, 1e-6f);
    update_recovery_state(scan_quality);

    float weight_sum = 0.0f;
    for (auto& particle : g_particles) {
        particle.weight = std::exp(particle.weight - max_log);
        weight_sum += particle.weight;
    }

    for (auto& particle : g_particles) {
        particle.weight /= weight_sum;
    }

    std::vector<Particle> scan_particles;
    if (g_use_pcb && compensate_delay) {
        scan_particles.reserve(g_particles.size());
        for (const auto& particle : g_particles)
            scan_particles.push_back(line_pose_at_time(particle, present, scan_time_s));
    }
    LocPose scan_pose = estimate_pose_from_particles(obs.data(), n,
        scan_particles.empty() ? g_particles : scan_particles);
    if (scan_pose.ok) {
        g_ready = true;
    }

    // Preserve particle diversity on under-constrained (partial) scans.
    if (effective_sample_size() < ESS_RESAMPLE_FRACTION * PARTICLE_COUNT) {
        resample_particles();
    }

    // Bring the scan-corrected particles back to the present without adding
    // process noise a second time.
    if (compensate_delay && !g_use_pcb) {
        for (const auto& step : g_odometry_history) {
            if (step.end_time_s <= scan_time_s) {
                continue;
            }
            const double overlap_start = std::max(step.start_time_s, scan_time_s);
            const float replay_dt = (float)(step.end_time_s - overlap_start);
            for (auto& particle : g_particles) {
                propagate_particle(
                    particle, step.vx_mm_s, step.vy_mm_s,
                    step.omega_deg_s, replay_dt, false);
            }
        }
    }

    g_pose = estimate_pose_from_particles(nullptr, 0);
    g_pose.confidence = scan_pose.confidence;
    g_pose.ok = scan_pose.ok;
    if (g_pose.ok && had_prior_pose) {
        float dx = g_pose.x - predicted_pose.x;
        float dy = g_pose.y - predicted_pose.y;
        g_last_scan_correction.sequence += 1;
        g_last_scan_correction.predicted_x = predicted_pose.x;
        g_last_scan_correction.predicted_y = predicted_pose.y;
        g_last_scan_correction.predicted_yaw_deg = predicted_pose.yaw_deg;
        g_last_scan_correction.corrected_x = g_pose.x;
        g_last_scan_correction.corrected_y = g_pose.y;
        g_last_scan_correction.corrected_yaw_deg = g_pose.yaw_deg;
        g_last_scan_correction.error_mm = std::sqrt(dx * dx + dy * dy);
        g_last_scan_correction.yaw_error_deg =
            wrap_angle_deg(g_pose.yaw_deg - predicted_pose.yaw_deg);
        g_last_scan_correction.valid = true;
    }
}

void loc_set_line_readings(const std::vector<std::string>& colours, double timestamp_s) {
    if (colours.size() != 32 || !std::isfinite(timestamp_s) || timestamp_s <= 0)
        throw std::invalid_argument("Expected 32 colours and a positive finite timestamp");
    for (const auto& colour : colours)
        if (colour != "black" && colour != "green" && colour != "white")
            throw std::invalid_argument("Line colour must be black, green, or white");
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    const double now = monotonic_time_s();
    if (timestamp_s > now || now - timestamp_s > 0.5 ||
        timestamp_s <= g_line_readings.timestamp_s ||
        (g_use_pcb && !g_odometry_history.empty() &&
         timestamp_s < g_odometry_history.front().start_time_s)) return;
    std::copy(colours.begin(), colours.end(), g_line_readings.colours.begin());
    g_line_readings.timestamp_s = timestamp_s;
    g_line_readings.valid = true;
    apply_pending_line_readings_locked(now);
}

void loc_clear_line_readings() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    g_line_readings.valid = false;
}

LocLineReadings loc_get_line_readings() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    auto result = g_line_readings;
    const double age = monotonic_time_s() - result.timestamp_s;
    result.valid = result.valid && age >= 0 && age <= 0.5;
    return result;
}

bool loc_scan_updates_allowed() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    return !g_scan_updates_paused;
}

bool loc_is_ready() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    return g_ready && g_pose.ok;
}

LocPose loc_get_pose() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    return g_pose;
}

LocScanCorrection loc_get_last_scan_correction() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    return g_last_scan_correction;
}

LocRecoveryStatus loc_get_recovery_status() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    return {
        g_last_scan_quality,
        g_scan_quality_baseline,
        g_bad_scan_count,
        std::max(EXPLORATION_FRACTION, g_recovery_fraction),
        g_scan_quality_baseline_valid
    };
}

std::vector<LocParticle> loc_get_particles() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    std::vector<LocParticle> result;
    result.reserve(g_particles.size());
    for (const auto& particle : g_particles) {
        result.push_back({
            particle.x, particle.y, particle.yaw_deg, particle.weight
        });
    }
    return result;
}
