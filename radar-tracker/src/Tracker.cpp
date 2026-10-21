#include "Tracker.hpp"

#include "Hungarian.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace {
constexpr double kOutOfGate = 1e9;
}

Tracker::Tracker(const TrackerConfig& cfg) : cfg_(cfg) {}

int Tracker::confirmedCount() const {
    return static_cast<int>(std::count_if(tracks_.begin(), tracks_.end(), [](const Track& t) {
        return t.status == TrackStatus::Confirmed;
    }));
}

Mat2 Tracker::measurementCov(const Detection& d) const {
    const double sr = cfg_.rangeSigma;
    const double sc = d.range * cfg_.bearingSigma;
    const Mat2 polar{{sr * sr, 0.0, 0.0, sc * sc}};

    const double c = std::cos(d.bearing), s = std::sin(d.bearing);
    const Mat2 G{{c, -s, s, c}};
    return G * polar * G.transpose();
}

void Tracker::spawn(const Detection& d, const Mat2& R) {
    Track t;
    t.id = nextId_++;
    t.kf = KalmanFilter(cfg_.processNoise);
    t.kf.init(Vec2{{d.x, d.y}}, R, cfg_.maxSpeed);
    t.age = 1;
    t.hits = 1;
    tracks_.push_back(t);
}

void Tracker::process(const std::vector<Detection>& detections, double dt) {
    for (Track& t : tracks_) {
        t.kf.predict(dt);
        ++t.age;
    }

    std::vector<Mat2> R(detections.size());
    std::vector<Vec2> z(detections.size());
    for (std::size_t j = 0; j < detections.size(); ++j) {
        R[j] = measurementCov(detections[j]);
        z[j] = Vec2{{detections[j].x, detections[j].y}};
    }

    std::vector<int> assignment(tracks_.size(), -1);
    if (!tracks_.empty() && !detections.empty()) {
        std::vector<std::vector<double>> cost(tracks_.size(),
                                              std::vector<double>(detections.size(), kOutOfGate));
        for (std::size_t i = 0; i < tracks_.size(); ++i)
            for (std::size_t j = 0; j < detections.size(); ++j) {
                const double d2 = tracks_[i].kf.mahalanobis2(z[j], R[j]);
                if (d2 <= cfg_.gateThreshold)
                    cost[i][j] = d2 + std::log(determinant(tracks_[i].kf.innovationCov(R[j])));
            }

        assignment = hungarian(cost);
        for (std::size_t i = 0; i < tracks_.size(); ++i) {
            const int j = assignment[i];
            if (j >= 0 && cost[i][static_cast<std::size_t>(j)] >= kOutOfGate) assignment[i] = -1;
        }
    }

    std::vector<bool> detectionUsed(detections.size(), false);
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
        Track& t = tracks_[i];
        const int j = assignment[i];
        if (j >= 0) {
            const auto ju = static_cast<std::size_t>(j);
            t.kf.update(z[ju], R[ju]);
            ++t.hits;
            t.consecutiveMisses = 0;
            detectionUsed[ju] = true;
        } else {
            ++t.consecutiveMisses;
        }

        if (t.status == TrackStatus::Tentative && t.hits >= cfg_.confirmHits &&
            t.age <= cfg_.confirmWindow)
            t.status = TrackStatus::Confirmed;
    }

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [&](const Track& t) {
                                     const Vec4& x = t.kf.state();
                                     if (std::hypot(x[0], x[1]) > cfg_.maxRange) return true;
                                     if (t.status == TrackStatus::Confirmed)
                                         return t.consecutiveMisses > cfg_.maxConfirmedMisses;
                                     return t.consecutiveMisses > cfg_.maxTentativeMisses ||
                                            t.age >= cfg_.confirmWindow;
                                 }),
                  tracks_.end());

    for (std::size_t j = 0; j < detections.size(); ++j)
        if (!detectionUsed[j]) spawn(detections[j], R[j]);
}
