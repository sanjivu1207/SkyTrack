#pragma once

#include "KalmanFilter.hpp"
#include "Simulator.hpp"

#include <vector>

struct TrackerConfig {
    double gateThreshold = 13.8;
    double processNoise = 5.0;
    double maxSpeed = 300.0;
    double rangeSigma = 50.0;
    double bearingSigma = 0.002;
    int confirmHits = 3;
    int confirmWindow = 5;
    int maxTentativeMisses = 1;
    int maxConfirmedMisses = 5;
};

enum class TrackStatus { Tentative, Confirmed };

struct Track {
    int id = 0;
    TrackStatus status = TrackStatus::Tentative;
    KalmanFilter kf;
    int age = 0;
    int hits = 0;
    int consecutiveMisses = 0;
};

class Tracker {
public:
    explicit Tracker(const TrackerConfig& cfg = TrackerConfig{});

    void process(const std::vector<Detection>& detections, double dt);

    const std::vector<Track>& tracks() const { return tracks_; }
    int confirmedCount() const;

    Mat2 measurementCov(const Detection& d) const;

private:
    void spawn(const Detection& d, const Mat2& R);

    TrackerConfig cfg_;
    std::vector<Track> tracks_;
    int nextId_ = 0;
};
