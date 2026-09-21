#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <vector>

namespace motion {
constexpr double RAD = 3.14159265358979323846 / 180.0;
struct Value { double time, value; };
struct Wheel { double time, vx, vy; };
// Local x is forward, local y is right; input body vy is LEFT.
struct Transform { double x = 0, y = 0, yaw = 0; };
inline double wrap(double a) { return std::remainder(a, 360.0); }
inline void advance(Transform& p, double vx, double vy, double omega, double dt) {
    const double theta = omega * RAD * dt;
    const double sinc = std::abs(theta) < 1e-6 ? 1 - theta*theta/6 : std::sin(theta)/theta;
    const double cosc = std::abs(theta) < 1e-6 ? theta/2 : (1-std::cos(theta))/theta;
    const double dx = dt * (vx*sinc + vy*cosc), dy = dt * (vx*cosc - vy*sinc);
    const double c = std::cos(p.yaw*RAD), s = std::sin(p.yaw*RAD);
    p.x += c*dx - s*dy; p.y += s*dx + c*dy;
    p.yaw = wrap(p.yaw + omega*dt);
}
inline Transform inverse(const Transform& p) {
    Transform out;
    const double c = std::cos(p.yaw*RAD), s = std::sin(p.yaw*RAD);
    out.x = -c*p.x - s*p.y; out.y = s*p.x - c*p.y; out.yaw = -p.yaw;
    return out;
}

class History {
public:
    std::deque<Value> yaw, gyro;
    std::deque<Wheel> wheels;
    static void append(std::deque<Value>& values, double time, double value) {
        if (!std::isfinite(time) || time <= 0 || !std::isfinite(value))
            throw std::invalid_argument("Invalid timestamped IMU sample");
        if (!values.empty() && time <= values.back().time) return;
        values.push_back({time,value});
        while (values.size() > 2 && values[1].time < time-2) values.pop_front();
    }
    static bool interpolate(const std::deque<Value>& values, double time,
                            double& result, bool angular = false) {
        if (values.empty() || time < values.front().time || time > values.back().time) return false;
        auto hi = std::lower_bound(values.begin(), values.end(), time,
            [](const Value& v, double t) { return v.time < t; });
        if (hi->time == time) { result = hi->value; return true; }
        auto lo = hi-1;
        if (hi->time-lo->time > 0.1) return false;
        const double delta = angular ? wrap(hi->value-lo->value) : hi->value-lo->value;
        result = lo->value + delta*(time-lo->time)/(hi->time-lo->time);
        if (angular) result = wrap(result);
        return true;
    }
    bool velocity(double time, double& vx, double& vy) const {
        if (wheels.empty() || time < wheels.front().time || time > wheels.back().time) return false;
        auto hi = std::lower_bound(wheels.begin(), wheels.end(), time,
            [](const Wheel& v, double t) { return v.time < t; });
        if (hi->time == time) { vx=hi->vx; vy=hi->vy; return true; }
        auto lo=hi-1;
        if (hi->time-lo->time > 0.25) return false;
        const double f=(time-lo->time)/(hi->time-lo->time);
        vx=lo->vx+f*(hi->vx-lo->vx); vy=lo->vy+f*(hi->vy-lo->vy);
        return true;
    }
    bool relative(double from, double to, bool translation, Transform& out) const {
        out = {};
        if (from == to) return true;
        const double start=std::min(from,to), end=std::max(from,to);
        double unused=0, vx=0, vy=0;
        if (!interpolate(gyro,start,unused) || !interpolate(gyro,end,unused) ||
            (translation && (!velocity(start,vx,vy) || !velocity(end,vx,vy)))) return false;
        std::vector<double> cuts{start,end};
        for (const auto& v : gyro) if (v.time > start && v.time < end) cuts.push_back(v.time);
        if (translation)
            for (const auto& v : wheels) if (v.time > start && v.time < end) cuts.push_back(v.time);
        std::sort(cuts.begin(),cuts.end());
        for (size_t i=1; i<cuts.size(); ++i) {
            const double dt=cuts[i]-cuts[i-1], mid=(cuts[i]+cuts[i-1])*0.5;
            double omega=0;
            if (!interpolate(gyro,mid,omega) || (translation && !velocity(mid,vx,vy))) return false;
            advance(out,vx,vy,omega,dt);
        }
        if (to < from) out=inverse(out);
        return true;
    }
};
} // namespace motion
