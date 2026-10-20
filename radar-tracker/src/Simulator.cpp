#include "Simulator.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

Simulator::Simulator(const RadarConfig& cfg, unsigned seed) : cfg_(cfg), rng_(seed) {}

bool Simulator::inCoverage(const Target& t) const {
    return std::hypot(t.x, t.y) <= cfg_.maxRange;
}

void Simulator::addRandomTargets(int n) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> angle(-kPi, kPi);
    std::uniform_real_distribution<double> speed(80.0, 250.0);  
    std::uniform_real_distribution<double> turn(0.005, 0.02);   // rad/s

    for (int i = 0; i < n; ++i) {
        Target t;
        t.id = nextId_++;
        const double r = 0.8 * cfg_.maxRange * std::sqrt(unit(rng_));
        const double a = angle(rng_);
        t.x = r * std::cos(a);
        t.y = r * std::sin(a);

        const double heading = angle(rng_);
        const double s = speed(rng_);
        t.vx = s * std::cos(heading);
        t.vy = s * std::sin(heading);

        if (unit(rng_) < 0.3) t.turnRate = turn(rng_) * (unit(rng_) < 0.5 ? -1.0 : 1.0);
        targets_.push_back(t);
    }
}

void Simulator::step(double dt) {
    for (Target& t : targets_) {
        if (t.turnRate != 0.0) {
            const double th = t.turnRate * dt;
            const double c = std::cos(th), s = std::sin(th);
            const double vx = c * t.vx - s * t.vy;
            const double vy = s * t.vx + c * t.vy;
            t.vx = vx;
            t.vy = vy;
        }
        t.x += t.vx * dt;
        t.y += t.vy * dt;
    }
}

std::vector<Detection> Simulator::sweep() {
    std::vector<Detection> out;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::normal_distribution<double> rangeNoise(0.0, cfg_.rangeSigma);
    std::normal_distribution<double> bearingNoise(0.0, cfg_.bearingSigma);


    for (const Target& t : targets_) {
        if (!inCoverage(t)) continue;
        if (unit(rng_) > cfg_.probDetection) continue;  

        Detection d;
        d.range = std::hypot(t.x, t.y) + rangeNoise(rng_);
        d.bearing = std::atan2(t.y, t.x) + bearingNoise(rng_);
        d.x = d.range * std::cos(d.bearing);
        d.y = d.range * std::sin(d.bearing);
        d.truthId = t.id;
        out.push_back(d);
    }

    int nClutter = 0;
    if (cfg_.clutterPerSweep > 0.0) {
        std::poisson_distribution<int> clutterCount(cfg_.clutterPerSweep);
        nClutter = clutterCount(rng_);
    }
    for (int i = 0; i < nClutter; ++i) {
        Detection d;
        d.range = cfg_.maxRange * std::sqrt(unit(rng_));  
        d.bearing = (2.0 * unit(rng_) - 1.0) * kPi;
        d.x = d.range * std::cos(d.bearing);
        d.y = d.range * std::sin(d.bearing);
        d.truthId = -1;
        out.push_back(d);
    }

    std::shuffle(out.begin(), out.end(), rng_);
    return out;
}
