#include "../lib/localisation.cpp"
#include <cassert>
#include <iostream>
#include <limits>

static const LocPose truth{1100,800,12,1,true};

static void setup(double vx=500, double vy=0, double omega=40) {
    loc_stop(); loc_set_replay_time(10.3); loc_seed(42);
    loc_init_map(2430,1820); loc_start(false);
    loc_configure_deskew("full",40,15,3);
    for (int i=0; i<=40; ++i) {
        const double t=9.8+i*0.01;
        motion::History::append(g_motion.gyro,t,omega);
        motion::History::append(g_motion.yaw,t,motion::wrap(truth.yaw_deg+(t-10)*omega));
        g_motion.wheels.push_back({t,vx,vy});
    }
}

static std::vector<LocScanPoint> moving_scan(double vx, double vy, double omega) {
    std::vector<LocScanPoint> scan;
    for (int i=0; i<180; ++i) {
        const double t=9.95+i*0.1/179;
        motion::Transform pose;
        pose.x=truth.x; pose.y=truth.y; pose.yaw=truth.yaw_deg;
        motion::advance(pose,vx,vy,omega,t-10);
        const double c=std::cos(pose.yaw*motion::RAD), s=std::sin(pose.yaw*motion::RAD);
        const double angle=2*i+0.3, a=(pose.yaw+angle+3)*motion::RAD;
        const auto hit=predict_hit(pose.x+c*40+s*15,pose.y+s*40-c*15,
                                   std::cos(a),std::sin(a),2430,1820);
        scan.emplace_back(angle,hit.range_mm,63,true,t);
    }
    return scan;
}

static double residual(const std::vector<LocScanPoint>& scan, const std::string& mode) {
    auto obs=bin_observations(scan.data(),scan.size(),80,6000,5);
    assert(prepare_observations(obs,10,mode));
    return wall_residual(truth,obs);
}

static void geometry() {
    for (double omega : {-90.0,0.0,90.0}) {
        setup(0,0,omega);
        const auto scan=moving_scan(0,0,omega);
        assert(residual(scan,"full") < 0.01);
        assert(residual(scan,"rotation") < 0.01);
        if (omega != 0) assert(residual(scan,"off") > 10);
        else assert(residual(scan,"off") < 0.01);
    }
    for (double vy : {-300.0,0.0,300.0}) {
        setup(500,vy,40);
        const auto scan=moving_scan(500,vy,40);
        assert(residual(scan,"full") < 0.01);
        assert(residual(scan,"rotation") > 3);
        assert(residual(scan,"off") > 10);
    }
    // A blocked robot with spinning wheels demonstrates the documented limit:
    // wrong wheel motion worsens translation deskew; rotation-only still works.
    setup(500,0,40);
    const auto slip_scan=moving_scan(0,0,40);
    assert(residual(slip_scan,"rotation") < 0.01);
    assert(residual(slip_scan,"full") > 3);
}

static void bearings_and_unknowns() {
    const std::vector<LocScanPoint> scan{{0.1f,1000,20,true,9.99},
        {1.9f,900,63,true,10.01},{3,20,63,true,10},{5,1000,1,true,10},
        {7,0,0,false,10.02}};
    const auto obs=bin_observations(scan.data(),scan.size(),80,6000,5);
    assert(obs.size()==2 && obs[0].angle_deg==1.9f && obs[0].time_s==10.01);
    assert(!obs[1].hit && obs[1].angle_deg==7 && obs[1].time_s==10.02);
    auto moved=obs;
    setup();
    assert(prepare_observations(moved,10,"full"));
    // Misses carry an origin and direction too, not a fabricated endpoint.
    assert(moved[1].origin_x != float(g_mount_forward));
    assert(moved[1].angle_deg != obs[1].angle_deg);
}

static void history_checks() {
    motion::History history;
    motion::History::append(history.yaw,10,179);
    motion::History::append(history.yaw,10.02,-179);
    double angle=0;
    assert(motion::History::interpolate(history.yaw,10.01,angle,true));
    assert(std::abs(std::abs(angle)-180) < 1e-9);
    assert(!motion::History::interpolate(history.yaw,9.99,angle,true));
    motion::History::append(history.yaw,10.5,0);
    assert(!motion::History::interpolate(history.yaw,10.3,angle,true));
    setup();
    motion::Transform f,b;
    assert(g_motion.relative(9.95,10.05,true,f));
    assert(g_motion.relative(10.05,9.95,true,b));
    const auto inverse=motion::inverse(f);
    assert(std::hypot(inverse.x-b.x,inverse.y-b.y) < 1e-9);
    assert(!g_motion.relative(9.7,10.05,true,f));
    g_motion.gyro.erase(g_motion.gyro.begin()+10,g_motion.gyro.begin()+30);
    assert(!g_motion.relative(9.95,10.05,true,f));
    setup();
    g_motion.wheels.clear();
    assert(g_motion.relative(9.95,10.05,false,f));
    assert(!g_motion.relative(9.95,10.05,true,f));
}

static LocPose feed_run(double delay, bool pcb=false) {
    loc_stop(); loc_start(pcb); loc_configure_deskew("full"); loc_seed(123);
    for (auto& p : g_particles) p={1100,800,0,1.0f/PARTICLE_COUNT};
    g_pose={1100,800,0,1,true}; g_ready=true;
    for (int i=0; i<=20; ++i) {
        const double t=20+i*0.02;
        loc_set_replay_time(t+delay);
        loc_feed_motion(500,0,t,0.001,{{t,40*(t-20)}},{{t,40}},1);
    }
    assert(g_odometry_history.size()==20);
    for (size_t i=1; i<g_odometry_history.size(); ++i)
        assert(g_odometry_history[i].start_time_s==g_odometry_history[i-1].end_time_s);
    return loc_get_pose();
}

static void timing_and_resets() {
    const auto a=feed_run(0.001), b=feed_run(0.18), c=feed_run(0.18,true);
    assert(a.x==b.x && a.y==b.y && a.yaw_deg==b.yaw_deg);
    assert(a.x==c.x && a.y==c.y && a.yaw_deg==c.yaw_deg);
    bool mixed=false;
    try { loc_predict_odometry(0,0,0,0.02); }
    catch (const std::logic_error&) { mixed=true; }
    assert(mixed);
    loc_set_replay_time(20.5);
    loc_feed_motion(500,0,20.5,0.001,{{20.5,0}},{{20.5,0}},2);
    assert(g_motion.yaw.size()==1 && g_odometry_history.empty());
    loc_set_replay_time(20.52);
    loc_feed_motion(500,0,20.52,0.001,{{20.52,0}},{{20.52,0}},2);
    assert(g_odometry_history.size()==1);
    loc_set_replay_time(21);
    loc_feed_motion(500,0,21,0.001,{{21,0}},{{21,0}},2);
    assert(g_odometry_history.empty() && !g_pose.ok);
}

static void high_speed_diagnostics_and_prior() {
    setup(0,0,90);
    g_timed_motion=true; g_pose_time=10.2;
    for (int i=0; i<40; ++i)
        g_odometry_history.push_back({9.8+i*0.01,9.8+(i+1)*0.01,0,0,90});
    g_ready=true; g_pose=truth; g_pose.yaw_deg+=18;
    const auto scan=moving_scan(0,0,90);
    loc_update_scan(scan.data(),scan.size(),80,6000,5,10);
    const auto status=loc_get_deskew_status();
    assert(status.reason=="accepted" && status.accepted && status.history_ok);
    assert(status.raw_residual_mm>10 && status.corrected_residual_mm<0.02);
    assert(std::abs(status.max_rotation_deg-4.5)<0.001);
    assert(std::abs(status.duration_s-0.1)<1e-9);
    // A delayed scan must use interpolated yaw=12, not current yaw=30.
    g_imu_yaw_valid=true; g_imu_yaw_deg=30;
    for (auto& p : g_particles) p={truth.x,truth.y,30,1.0f/PARTICLE_COUNT};
    loc_update_scan(scan.data(),scan.size(),80,6000,5,10);
    assert(loc_get_deskew_status().accepted);
    auto obs=bin_observations(scan.data(),scan.size(),80,6000,5);
    assert(prepare_observations(obs,10,"full"));
    double weight_sum=0;
    for (const auto& o : obs) weight_sum+=o.weight;
    const double expected=score_pose(truth.x,truth.y,truth.yaw_deg,
        obs.data(),obs.size(),2430,1820,6000)/weight_sum;
    assert(std::abs(g_last_scan_quality-expected)<0.0001);
    assert(g_imu_yaw_deg==30); // Acquisition-time prior is scoped to the scan.
    g_motion.yaw.clear();
    loc_update_scan(scan.data(),scan.size(),80,6000,5,10);
    assert(loc_get_deskew_status().reason=="missing_yaw");
    auto bad=scan;
    bad[0].time_s=std::numeric_limits<double>::quiet_NaN();
    loc_update_scan(bad.data(),bad.size(),80,6000,5,10);
    assert(loc_get_deskew_status().reason=="invalid_scan_time");
}

int main() {
    geometry(); bearings_and_unknowns(); history_checks(); timing_and_resets();
    high_speed_diagnostics_and_prior();
    loc_stop(); loc_configure_deskew("off"); loc_set_replay_time(-1);
    std::cout << "Deskew geometry, slip, timing, reset and gate checks passed\n";
}
