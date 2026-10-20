#include "Hungarian.hpp"
#include "KalmanFilter.hpp"
#include "Simulator.hpp"
#include "Tracker.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

static int g_failures = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
            ++g_failures;                                                    \
        }                                                                    \
    } while (0)

static void testHungarianSquare() {
    std::printf("Hungarian square\n");
    const std::vector<std::vector<double>> cost = {
        {4, 1, 3},
        {2, 0, 5},
        {3, 2, 2},
    };
    const auto a = hungarian(cost);
    CHECK(a.size() == 3);
    CHECK(a[0] == 1);
    CHECK(a[1] == 0);
    CHECK(a[2] == 2);
}

static void testHungarianRectangular() {
    std::printf("Hungarian rectangular\n");
    const std::vector<std::vector<double>> wide = {
        {9, 1, 8},
        {1, 9, 9},
    };
    const auto w = hungarian(wide);
    CHECK(w.size() == 2);
    CHECK(w[0] == 1);
    CHECK(w[1] == 0);

    const std::vector<std::vector<double>> tall = {
        {5, 9},
        {1, 2},
        {9, 1},
    };
    const auto t = hungarian(tall);
    CHECK(t.size() == 3);
    CHECK(t[0] == -1);
    CHECK(t[1] == 0);
    CHECK(t[2] == 1);
}

static Mat2 diagR(double sigma) {
    return Mat2{{sigma * sigma, 0.0, 0.0, sigma * sigma}};
}

static KalmanFilter convergedFilter(double vx, double vy, double& px, double& py) {
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 20.0);
    const Mat2 R = diagR(20.0);

    px = 1000.0;
    py = 2000.0;
    KalmanFilter kf(0.1);
    kf.init(Vec2{{px + noise(rng), py + noise(rng)}}, R);
    for (int k = 0; k < 40; ++k) {
        px += vx;
        py += vy;
        kf.predict(1.0);
        kf.update(Vec2{{px + noise(rng), py + noise(rng)}}, R);
    }
    return kf;
}

static void testKalmanVelocityConvergence() {
    std::printf("Kalman velocity convergence\n");
    double px = 0.0, py = 0.0;
    const KalmanFilter kf = convergedFilter(150.0, -60.0, px, py);
    CHECK(std::abs(kf.state()[2] - 150.0) < 5.0);
    CHECK(std::abs(kf.state()[3] + 60.0) < 5.0);
    CHECK(std::hypot(kf.state()[0] - px, kf.state()[1] - py) < 40.0);
}

static void testGatingRejectsFarMeasurement() {
    std::printf("Gating rejects far measurement\n");
    const double kGate = 13.8;
    double px = 0.0, py = 0.0;
    KalmanFilter kf = convergedFilter(150.0, -60.0, px, py);
    kf.predict(1.0);
    px += 150.0;
    py -= 60.0;
    const Mat2 R = diagR(20.0);
    CHECK(kf.mahalanobis2(Vec2{{px, py}}, R) < kGate);
    CHECK(kf.mahalanobis2(Vec2{{px + 2000.0, py}}, R) > kGate);
}

static void testEndToEndTwoTargets() {
    std::printf("End-to-end: 2 targets, no clutter\n");
    RadarConfig radar;
    radar.clutterPerSweep = 0.0;
    Simulator sim(radar, 123);

    Target a;
    a.id = 0;
    a.x = 10'000.0;
    a.vy = 150.0;
    sim.addTarget(a);

    Target b;
    b.id = 1;
    b.x = -20'000.0;
    b.y = 5'000.0;
    b.vx = 200.0;
    b.turnRate = 0.01;
    sim.addTarget(b);

    Tracker tracker;
    for (int k = 0; k < 60; ++k) {
        sim.step(1.0);
        tracker.process(sim.sweep(), 1.0);
    }
    CHECK(tracker.confirmedCount() == 2);
}

int main() {
    testHungarianSquare();
    testHungarianRectangular();
    testKalmanVelocityConvergence();
    testGatingRejectsFarMeasurement();
    testEndToEndTwoTargets();

    if (g_failures == 0) {
        std::printf("All tests passed\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
