#include "../lib/localisation.cpp"
#include <cassert>
#include <iostream>

static void setup(double omega=0, double vx=0, double vy=0) {
    loc_stop(); loc_init_map(2430,1820); loc_start(false);
    loc_set_replay_time(10.05);
    loc_configure_deskew("full",40,15,3);
    g_timed_motion=true; g_pose_time=10.04;
    g_pose={600,700,20,0.9,true};
    for (int i=0; i<=20; ++i) {
        const double t=9.9+i*0.01;
        motion::History::append(g_motion.gyro,t,omega);
        motion::History::append(g_motion.yaw,t,0);
        g_motion.wheels.push_back({t,vx,vy});
    }
}

int main() {
    for (double omega : {-90.0,0.0,90.0}) {
        setup(omega,400,150);
        // Two supports within one old 2-degree bin MUST both survive.
        std::vector<LocScanPoint> scan{{0.1f,800,30,true,9.99},
                                      {0.3f,800,30,true,10.01}};
        const auto out=loc_fusion_context(scan,10,0.05);
        assert(out.reason=="ok" && out.points.size()==2);
        motion::Transform next; next.x=out.pose.x; next.y=out.pose.y;
        next.yaw=out.pose.yaw_deg;
        motion::advance(next,400,150,omega,0.04);
        assert(std::hypot(next.x-600,next.y-700) < 0.001);
        assert(std::abs(next.yaw-20) < 0.001);
        for (size_t i=0; i<scan.size(); ++i) {
            motion::Transform beam;
            motion::advance(beam,400,150,omega,scan[i].time_s-10);
            const double c=std::cos(beam.yaw*motion::RAD),s=std::sin(beam.yaw*motion::RAD);
            const double a=(scan[i].angle_deg+3+beam.yaw)*motion::RAD;
            assert(std::abs(out.points[i][0]-(beam.x+c*40+s*15+800*std::cos(a))) < 0.001);
            assert(std::abs(out.points[i][1]-(beam.y+s*40-c*15+800*std::sin(a))) < 0.001);
        }
    }
    setup(); loc_configure_deskew("full",0,0,0);
    g_pose={1000,800,0,0.9,true};
    auto wall=loc_fusion_context({{0,1430,30,true,10}},10,0.05);
    assert(wall.points.size()==1 && std::abs(wall.points[0][4]) < 0.001);
    // Goal front endpoint (post) is part of the static-map veto.
    const double dx=GOAL_RIGHT_FRONT_X-1000,dy=GOAL_TOP_Y-800;
    auto post=loc_fusion_context({{float(std::atan2(dy,dx)/motion::RAD),
                                  float(std::hypot(dx,dy)),30,true,10}},10,0.05);
    assert(post.points.size()==1 && post.points[0][4] < 0.001);
    auto filtered=loc_fusion_context({{0,1000,30,false,10},{0,0,30,true,10},
                                     {0,1000,1,true,10},{0,1000,30,true,9.8}},10,0.05);
    assert(filtered.points.empty());
    assert(loc_fusion_context({},9,0.05).reason=="stale_camera");
    assert(loc_fusion_context({},11,0.05).reason=="stale_camera");
    g_pose.confidence=0.4;
    assert(loc_fusion_context({},10,0.05).reason=="uncertain_pose");
    g_pose.confidence=0.9; g_motion.wheels.clear();
    assert(loc_fusion_context({},10,0.05).reason=="missing_motion");
    setup(); clear_motion_history();
    assert(loc_fusion_context({},10,0.05).reason=="missing_motion");
    std::cout << "Native bot fusion geometry/timing passed\n";
}
