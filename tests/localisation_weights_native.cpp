// Exercise the actual scan update, including exploration and ESS resampling.
#include "../lib/localisation.cpp"
#include <cassert>
#include <iostream>

static constexpr float MIN_RANGE = 80;
static constexpr float MAX_RANGE = 6000;
static constexpr int MIN_QUALITY = 5;

static void setup(bool pcb) {
    loc_stop();
    g_rng.seed(42);
    loc_init_map(2430, 1820);
    loc_start(pcb);
}

static std::vector<LocScanPoint> scan_at(const Particle& pose, int first = 1,
                                       int last = 359) {
    std::vector<LocScanPoint> scan;
    for (int angle = first; angle <= last; angle += 2) {
        const float radians = (angle + pose.yaw_deg) * float(M_PI / 180);
        const auto hit = predict_hit(pose.x, pose.y, std::cos(radians),
                                     std::sin(radians), g_pitch_x, g_pitch_y);
        scan.push_back({float(angle), hit.range_mm, 63, true});
    }
    return scan;
}

static void update(const std::vector<LocScanPoint>& scan) {
    loc_update_scan(scan.data(), scan.size(), MIN_RANGE, MAX_RANGE, MIN_QUALITY);
    double total = 0;
    for (const auto& p : g_particles) {
        assert(std::isfinite(p.weight) && p.weight >= 0);
        total += p.weight;
    }
    assert(std::abs(total - 1) < 0.0001);
}

static bool same_pose(const Particle& a, const Particle& b) {
    return a.x == b.x && a.y == b.y && a.yaw_deg == b.yaw_deg;
}

static int count_pose(const Particle& pose) {
    int count = 0;
    for (const auto& p : g_particles) count += same_pose(p, pose);
    return count;
}

static float weight_at(const Particle& pose) {
    for (const auto& p : g_particles)
        if (same_pose(p, pose)) return p.weight;
    assert(false && "Expected hypothesis was lost");
    return 0;
}

static void two_hypotheses(const Particle& a, const Particle& b) {
    // Equal numbers of particles, but historical evidence favours A by 4:1.
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        g_particles[i] = i % 2 ? b : a;
        g_particles[i].weight = (i % 2 ? 0.4f : 1.6f) / PARTICLE_COUNT;
    }
    assert(effective_sample_size() > ESS_RESAMPLE_FRACTION * PARTICLE_COUNT);
}

static void ambiguous_scans_preserve_odds(bool pcb) {
    setup(pcb);
    const Particle a{1000, 400, 0, 0}, b{1100, 400, 0, 0};
    two_hypotheses(a, b);
    // Both positions have exactly the same distance/bearing to this top wall.
    const auto scan = scan_at(a, 241, 299);
    for (int i = 0; i < 4; ++i) {
        update(scan);
        assert(std::abs(weight_at(a) / weight_at(b) - 4) < 0.0001);
        // Exploration removes a few particles, but ESS must not trigger resampling.
        assert(count_pose(a) > 400 && count_pose(b) > 400);
    }
}

static void weak_scans_accumulate_once(bool pcb, bool imu) {
    setup(pcb);
    const Particle a{1000, 400, 0, 0}, b{1000, 402, 0.2f, 0};
    two_hypotheses(a, b);
    if (imu) loc_set_imu_yaw(0);
    const auto scan = scan_at(a, 241, 299);
    const auto obs = bin_observations(scan.data(), scan.size(), MIN_RANGE,
                                      MAX_RANGE, MIN_QUALITY);
    const auto score = [&](const Particle& p) {
        return score_pose(p.x, p.y, p.yaw_deg, obs.data(), obs.size(),
                          g_pitch_x, g_pitch_y, MAX_RANGE);
    };
    const double yaw_term = imu ? 0.5 * std::pow(0.2 / YAW_PRIOR_SIGMA_DEG, 2) : 0;
    const double likelihood_ratio = std::exp(score(a) - score(b) + yaw_term);
    assert(likelihood_ratio > 1);
    double expected = 4;
    for (int i = 0; i < 3; ++i) {
        update(scan);
        expected *= likelihood_ratio;
        assert(std::abs(weight_at(a) / weight_at(b) / expected - 1) < 0.0001);
        assert(count_pose(a) > 400 && count_pose(b) > 400);
    }
}

static void resampling_transfers_evidence_to_counts(bool pcb) {
    setup(pcb);
    const Particle a{1000, 400, 0, 0}, b{1100, 400, 0, 0};
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        const bool preferred = i < 100;
        g_particles[i] = preferred ? a : b;
        g_particles[i].weight = preferred ? 0.009f : 0.1f / 900;
    }
    update(scan_at(a, 241, 299));
    assert(count_pose(a) > 800 && count_pose(b) < 200);
    for (const auto& p : g_particles)
        assert(std::abs(p.weight - 1.0f / PARTICLE_COUNT) < 1e-8);
    // Once weights are uniform, history lives in multiplicity, not old weights.
    update(scan_at(a, 241, 299));
    assert(count_pose(a) > 780 && count_pose(b) < 200);
    assert(std::abs(weight_at(a) / weight_at(b) - 1) < 0.0001);
}

static void recovery_quality_ignores_prior_weights(bool pcb) {
    setup(pcb);
    const Particle a{1000, 400, 0, 0}, b{1100, 400, 0, 0};
    two_hypotheses(a, b);
    const auto initial = g_particles;
    const auto rng = g_rng;
    const auto scan = scan_at(a, 241, 299);
    update(scan);
    const float quality = g_last_scan_quality;
    const float baseline = g_scan_quality_baseline;
    g_particles = initial;
    g_rng = rng;
    reset_recovery_state();
    for (auto& p : g_particles) p.weight = 1.0f / PARTICLE_COUNT;
    update(scan);
    assert(g_last_scan_quality == quality && g_scan_quality_baseline == baseline);
}

static void recovery_proposals_have_fresh_weights(bool pcb) {
    setup(pcb);
    const Particle old{1000, 400, 0, 0};
    for (auto& p : g_particles) p = old;
    // Most replaced particles have zero weight: new hypotheses must not inherit it.
    g_particles[0].weight = 1;
    g_recovery_fraction = RECOVERY_FRACTION_HIGH;
    const auto initial = g_particles;
    const auto rng = g_rng;
    inject_random_particles(g_recovery_fraction);
    Particle truth{};
    bool found = false;
    for (const auto& p : g_particles) {
        if (same_pose(p, old)) continue;
        assert(p.weight == 1.0f / PARTICLE_COUNT);
        if (p.x > 600 && p.x < 1800 && p.y > 700 && p.y < 1500) {
            truth = p;
            found = true;
        }
    }
    assert(found);
    const auto scan = scan_at(truth);
    // Recreate exactly those proposals inside the real scan-update path.
    g_particles = initial;
    g_rng = rng;
    g_bad_scan_count = RECOVERY_HIGH_BAD_SCANS;
    update(scan);
    assert(count_pose(truth) > PARTICLE_COUNT / 2);
    assert(std::hypot(g_pose.x - truth.x, g_pose.y - truth.y) < 30);
}

static void repeated_imu_scans_with_prediction(bool pcb) {
    setup(pcb);
    loc_set_imu_yaw(0);
    for (auto& p : g_particles)
        p = {1000 + rand_normal(10), 400 + rand_normal(10),
             rand_normal(2), 1.0f / PARTICLE_COUNT};
    const auto scan = scan_at({1000, 400, 0, 0});
    for (int i = 0; i < 100; ++i) {
        loc_predict_odometry(0, 0, 0, 0.1);
        update(scan);
    }
    const auto spread = compute_particle_spread(g_particles);
    assert(g_pose.ok && std::hypot(g_pose.x - 1000, g_pose.y - 400) < 10);
    assert(std::abs(g_pose.yaw_deg) < 1);
    // Process noise must still leave diversity after repeated yaw/scan evidence.
    assert(spread.std_yaw_deg > 0.01 && spread.std_x > 0.1 && spread.std_y > 0.1);
}

int main() {
    for (bool pcb : {false, true}) {
        ambiguous_scans_preserve_odds(pcb);
        weak_scans_accumulate_once(pcb, false);
        weak_scans_accumulate_once(pcb, true);
        resampling_transfers_evidence_to_counts(pcb);
        recovery_quality_ignores_prior_weights(pcb);
        recovery_proposals_have_fresh_weights(pcb);
        repeated_imu_scans_with_prediction(pcb);
    }
    loc_stop();
    std::cout << "Particle weight regressions passed in both PCB modes\n";
}
