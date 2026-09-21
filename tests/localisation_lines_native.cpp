#include "../lib/localisation.cpp"
#include <cassert>

constexpr bool USE_PCB = false;

static LocLineReadings sample_for(const Particle& p, double timestamp) {
    LocLineReadings result;
    result.timestamp_s = timestamp;
    result.valid = true;
    for (size_t i = 0; i < 32; ++i) {
        const float a = (p.yaw_deg + i*11.25f) * 3.14159265358979323846f/180;
        const int colour = floor_colour(p.x + 75*std::cos(a), p.y + 75*std::sin(a));
        result.colours[i] = colour == 0 ? "black" : colour == 2 ? "white" : "green";
    }
    return result;
}

static void submit(const LocLineReadings& sample) {
    loc_set_line_readings({sample.colours.begin(), sample.colours.end()}, sample.timestamp_s);
}

static double setup(bool enabled = true, bool ready = true) {
    loc_stop();
    g_rng.seed(42);
    loc_start(enabled);
    for (size_t i = 0; i < g_particles.size(); ++i)
        g_particles[i] = {350.0f + (i%2)*40, 900, (i%2)*5.0f, 1.0f/PARTICLE_COUNT};
    g_ready = ready;
    g_pose = {370,900,2.5,0.8f,ready};
    const double now = monotonic_time_s();
    g_odometry_history.push_back({now-1,now-0.02,0,0,0});
    return now;
}

static void timing_and_readiness() {
    double now = setup(true, false);
    submit(sample_for({350,900,0,1},now-0.02));
    assert(!loc_is_ready() && loc_get_line_readings().applied_count == 0);
    loc_predict_odometry(0,0,0,0.02);
    assert(!loc_is_ready() && loc_get_line_readings().applied_count == 0);

    now = setup();
    const auto sample = sample_for({350,900,0,1},now-0.02);
    submit(sample);
    assert(loc_get_line_readings().applied_count == 1);
    assert(loc_get_pose().x < 370 && loc_get_pose().yaw_deg < 2.5);
    assert(loc_get_pose().ok && loc_get_pose().confidence == 0.8f);
    assert(g_last_scan_correction.sequence == 0 && g_last_scan_quality == 0);
    const auto particles = g_particles;
    submit(sample); // Duplicate and out-of-order samples never apply.
    submit(sample_for({350,900,0,1},now-0.03));
    submit(sample_for({350,900,0,1},now+1));
    submit(sample_for({350,900,0,1},now-1));
    assert(loc_get_line_readings().applied_count == 1);
    for (size_t i=0; i<particles.size(); ++i) assert(g_particles[i].weight == particles[i].weight);

    // New data may be newer than the last prediction, but cannot be future-dated.
    submit(sample_for({350,900,0,1},monotonic_time_s()));
    assert(loc_get_line_readings().applied_count == 1);
    const double newest = monotonic_time_s();
    submit(sample_for({350,900,0,1},newest));
    loc_predict_odometry(0,0,100,0.02);
    assert(loc_scan_updates_allowed());
    assert(loc_get_line_readings().applied_count == 2);
    assert(loc_get_line_readings().last_applied_timestamp_s == newest);
    loc_predict_odometry(0,0,100,0.02);
    assert(loc_get_line_readings().applied_count == 2);

    // A current but out-of-history sample is rejected; stale pending data expires.
    now = setup();
    g_odometry_history.front().start_time_s = now-0.1;
    submit(sample_for({350,900,0,1},now-0.2));
    assert(loc_get_line_readings().applied_count == 0);
    submit(sample_for({350,900,0,1},now-0.01));
    apply_pending_line_readings_locked(now+1);
    assert(!loc_get_line_readings().valid && loc_get_line_readings().applied_count == 0);

    // Even after a LIDAR confidence dip, PCB may move the estimate but not validate it.
    now = setup();
    g_pose.ok = false;
    submit(sample_for({350,900,0,1},now-0.02));
    assert(loc_get_pose().x < 370 && !loc_is_ready());
    assert(loc_get_pose().confidence == 0.8f);
}

static void delayed_sample() {
    const double now = setup();
    g_odometry_history.clear();
    g_odometry_history.push_back({now-0.2,now-0.1,500,0,90});
    const auto at_sample = g_particles;
    for (auto& p : g_particles) propagate_particle(p,500,0,90,0.1,false);
    const auto present = g_particles;
    const auto sample = sample_for(at_sample[0],now-0.2);
    const double ratio = std::exp(0.2 * (score_floor_colours(at_sample[0],sample)
                                       - score_floor_colours(at_sample[1],sample)));
    submit(sample);
    assert(g_line_readings.applied_count == 1);
    assert(std::abs(g_particles[0].weight/g_particles[1].weight-ratio) < 0.0001);
    for (size_t i=0; i<present.size(); ++i) {
        assert(g_particles[i].x == present[i].x && g_particles[i].y == present[i].y);
        assert(g_particles[i].yaw_deg == present[i].yaw_deg);
    }
}

static void prediction_history_is_contiguous() {
    setup();
    const double previous_end = g_odometry_history.back().end_time_s;
    loc_predict_odometry(100, -20, 90, 0.1);
    const auto& step = g_odometry_history.back();
    assert(step.start_time_s == previous_end);
    const double duration = step.end_time_s - step.start_time_s;
    assert(std::abs(step.vx_mm_s * duration - 10) < 0.0001);
    assert(std::abs(step.vy_mm_s * duration + 2) < 0.0001);
    assert(std::abs(step.omega_deg_s * duration - 9) < 0.0001);
}

static float rate_run(double interval, int count) {
    double now = setup();
    const double start = now;
    // Use virtual time to exercise a long outage without sleeping or hardware.
    for (int i=0; i<count; ++i) {
        now = start + i*interval;
        g_odometry_history.clear();
        g_odometry_history.push_back({now-1,now,0,0,0});
        const auto sample = sample_for({350,900,0,1},now);
        g_line_readings.colours = sample.colours;
        g_line_readings.timestamp_s = now;
        g_line_readings.valid = true;
        apply_pending_line_readings_locked(now);
        assert(g_line_readings.applied_count == static_cast<unsigned>(i+1));
        assert(g_pose.ok && g_pose.confidence == 0.8f);
        assert(g_last_scan_correction.sequence == 0);
    }
    return g_pose.x;
}

static void resumed_scan(bool enabled) {
    const double now = setup(enabled);
    for (auto& p : g_particles) { p.yaw_deg = 0; p.y = 400; }
    g_pose.y = 400;
    g_odometry_history.clear();
    g_odometry_history.push_back({now-0.2,now-0.02,100,0,20});
    submit(sample_for({350,400,0,1},now-0.02));
    const auto prior = g_particles;
    const auto used = g_line_readings.applied_count;
    std::vector<LocScanPoint> scan;
    for (int angle = 240; angle <= 300; angle += 2) {
        const Particle scan_pose = line_pose_at_time({370,400,0,1},now-0.02,now-0.1);
        const float a = (angle+scan_pose.yaw_deg)*3.14159265358979323846f/180;
        const auto hit = predict_hit(scan_pose.x,scan_pose.y,std::cos(a),std::sin(a),2430,1820);
        scan.push_back({float(angle),hit.range_mm,63,true});
    }
    const auto obs = bin_observations(scan.data(),scan.size(),80,6000,5);
    loc_update_scan(scan.data(),scan.size(),80,6000,5,now-0.1);
    assert(g_line_readings.applied_count == used); // No line likelihood in LIDAR updates.
    // Exploration replaces a few particles. Compare untouched representatives.
    int a=-1, b=-1;
    for (size_t i=0; i<g_particles.size(); ++i) {
        const auto& p = g_particles[i];
        if (std::abs(p.x-prior[i].x) < 0.001 && std::abs(p.y-prior[i].y) < 0.001 &&
            std::abs(p.yaw_deg-prior[i].yaw_deg) < 0.001) {
            if (i%2) b=i; else a=i;
        }
    }
    assert(a>=0 && b>=0);
    const auto likelihood = [&](const Particle& current) {
        const Particle p = line_pose_at_time(current,now-0.02,now-0.1);
        return score_pose(p.x,p.y,p.yaw_deg,obs.data(),obs.size(),2430,1820,6000);
    };
    double ratio = std::exp(likelihood(prior[a])-likelihood(prior[b]));
    ratio *= prior[a].weight/prior[b].weight;
    assert(std::abs(g_particles[a].weight/g_particles[b].weight-ratio) < 0.0001);
}

int main() {
    loc_init_map(2430,1820);
    setup(USE_PCB);
    assert(!g_use_pcb);
    assert(floor_colour(275,900)==2 && floor_colour(350,900)==1);
    assert(floor_colour(600,900)==0 && floor_colour(1215,610)==0);
    assert(floor_colour(2155,900)==2 && floor_colour(1830,900)==0);
    const Particle truth{350,900,0,1}, wrong{430,900,0,1};
    const auto sample = sample_for(truth,monotonic_time_s());
    assert(sample.colours[16]=="white" && sample.colours[0]=="green");
    assert(score_floor_colours(truth,sample) > score_floor_colours(wrong,sample));
    assert(sample_for({350,900,90,1},monotonic_time_s()).colours[8]=="white");
    submit(sample);
    loc_predict_odometry(0,0,0,0.02);
    assert(g_line_readings.applied_count == 0);
    timing_and_readiness();
    delayed_sample();
    prediction_history_is_contiguous();
    assert(std::abs(rate_run(0.02,21)-rate_run(0.1,5)) < 0.01);
    assert(rate_run(0.02,3001) < 351); // One minute without any LIDAR updates.
    resumed_scan(true);
    resumed_scan(false);
    loc_stop();
    assert(!g_use_pcb && !loc_get_line_readings().valid);
    assert(loc_get_line_readings().applied_count == 0);
}
