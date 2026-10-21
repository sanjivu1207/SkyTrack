#include "Hungarian.hpp"
#include "Simulator.hpp"
#include "Tracker.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {

constexpr double kDt = 1.0;
constexpr double kScoringRadius = 1000.0;
constexpr double kNoMatch = 1e9;

int parseInt(const char* arg, const char* name, int minValue) {
    try {
        std::size_t used = 0;
        const int v = std::stoi(arg, &used);
        if (used == std::string(arg).size() && v >= minValue) return v;
    } catch (const std::exception&) {
    }
    std::fprintf(stderr, "Invalid %s: '%s' (expected an integer >= %d)\n", name, arg, minValue);
    std::exit(1);
}

std::vector<int> matchTracksToTruth(const std::vector<const Track*>& tracks,
                                    const std::vector<const Target*>& truth) {
    if (tracks.empty() || truth.empty()) return std::vector<int>(tracks.size(), -1);
    std::vector<std::vector<double>> cost(tracks.size(), std::vector<double>(truth.size()));
    for (std::size_t i = 0; i < tracks.size(); ++i)
        for (std::size_t j = 0; j < truth.size(); ++j) {
            const Vec4& s = tracks[i]->kf.state();
            const double d = std::hypot(s[0] - truth[j]->x, s[1] - truth[j]->y);
            cost[i][j] = d <= kScoringRadius ? d : kNoMatch;
        }
    std::vector<int> a = hungarian(cost);
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i] >= 0 && cost[i][static_cast<std::size_t>(a[i])] >= kNoMatch) a[i] = -1;
    return a;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 4 || (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help"))) {
        std::printf("usage: radar_tracker [numTargets] [numSweeps] [seed]\n");
        return argc > 4 ? 1 : 0;
    }
    const int numTargets = argc > 1 ? parseInt(argv[1], "numTargets", 1) : 20;
    const int numSweeps = argc > 2 ? parseInt(argv[2], "numSweeps", 1) : 200;
    const unsigned seed = argc > 3 ? static_cast<unsigned>(parseInt(argv[3], "seed", 0)) : 42u;

    RadarConfig radar;
    Simulator sim(radar, seed);
    sim.addRandomTargets(numTargets);

    TrackerConfig tcfg;
    tcfg.rangeSigma = radar.rangeSigma;
    tcfg.bearingSigma = radar.bearingSigma;
    tcfg.maxRange = radar.maxRange;
    Tracker tracker(tcfg);

    std::ofstream truthCsv("truth.csv"), detCsv("detections.csv"), trackCsv("tracks.csv");
    if (!truthCsv || !detCsv || !trackCsv) {
        std::fprintf(stderr, "Could not open output CSV files for writing\n");
        return 1;
    }
    truthCsv << "sweep,id,x,y,vx,vy,in_coverage\n";
    detCsv << "sweep,x,y,truth_id\n";
    trackCsv << "sweep,id,status,x,y,vx,vy\n";

    std::printf("Radar tracker: %d targets, %d sweeps, seed %u\n\n", numTargets, numSweeps, seed);
    std::printf("%6s %9s %6s %8s %10s %10s %9s\n", "sweep", "coverage", "dets", "clutter",
                "tentative", "confirmed", "time[ms]");

    double radarErr2 = 0.0, trackErr2 = 0.0;
    long radarErrN = 0, trackErrN = 0;
    long confirmedTrackSweeps = 0, falseTrackSweeps = 0;
    int idSwitches = 0;
    std::map<int, int> lastTrackForTruth;
    std::set<int> falseTrackIds, confirmedIds;
    double totalTrackerSeconds = 0.0;

    for (int k = 1; k <= numSweeps; ++k) {
        sim.step(kDt);
        const std::vector<Detection> dets = sim.sweep();

        const auto t0 = std::chrono::steady_clock::now();
        tracker.process(dets, kDt);
        const auto t1 = std::chrono::steady_clock::now();
        const double sweepSeconds = std::chrono::duration<double>(t1 - t0).count();
        totalTrackerSeconds += sweepSeconds;

        std::map<int, const Target*> truthById;
        std::vector<const Target*> covered, all;
        for (const Target& t : sim.targets()) {
            const bool in = sim.inCoverage(t);
            truthCsv << k << ',' << t.id << ',' << t.x << ',' << t.y << ',' << t.vx << ',' << t.vy
                     << ',' << (in ? 1 : 0) << '\n';
            truthById[t.id] = &t;
            all.push_back(&t);
            if (in) covered.push_back(&t);
        }

        int clutter = 0;
        for (const Detection& d : dets) {
            detCsv << k << ',' << d.x << ',' << d.y << ',' << d.truthId << '\n';
            if (d.truthId < 0) {
                ++clutter;
                continue;
            }
            const Target* t = truthById[d.truthId];
            radarErr2 += (d.x - t->x) * (d.x - t->x) + (d.y - t->y) * (d.y - t->y);
            ++radarErrN;
        }

        std::vector<const Track*> confirmed;
        int tentative = 0;
        for (const Track& tr : tracker.tracks()) {
            const Vec4& s = tr.kf.state();
            const bool isConfirmed = tr.status == TrackStatus::Confirmed;
            trackCsv << k << ',' << tr.id << ',' << (isConfirmed ? "confirmed" : "tentative") << ','
                     << s[0] << ',' << s[1] << ',' << s[2] << ',' << s[3] << '\n';
            if (isConfirmed) {
                confirmed.push_back(&tr);
                confirmedIds.insert(tr.id);
            } else {
                ++tentative;
            }
        }

        const std::vector<int> match = matchTracksToTruth(confirmed, all);
        for (std::size_t i = 0; i < confirmed.size(); ++i) {
            ++confirmedTrackSweeps;
            if (match[i] < 0) {
                ++falseTrackSweeps;
                falseTrackIds.insert(confirmed[i]->id);
                continue;
            }
            const Target* t = all[static_cast<std::size_t>(match[i])];
            const Vec4& s = confirmed[i]->kf.state();
            trackErr2 += (s[0] - t->x) * (s[0] - t->x) + (s[1] - t->y) * (s[1] - t->y);
            ++trackErrN;

            const auto prev = lastTrackForTruth.find(t->id);
            if (prev != lastTrackForTruth.end() && prev->second != confirmed[i]->id) ++idSwitches;
            lastTrackForTruth[t->id] = confirmed[i]->id;
        }

        if (k % 20 == 0 || k == numSweeps)
            std::printf("%6d %9zu %6zu %8d %10d %10zu %9.3f\n", k, covered.size(), dets.size(),
                        clutter, tentative, confirmed.size(), sweepSeconds * 1e3);
    }

    std::size_t inCoverage = 0;
    for (const Target& t : sim.targets())
        if (sim.inCoverage(t)) ++inCoverage;

    const double radarRms = radarErrN ? std::sqrt(radarErr2 / static_cast<double>(radarErrN)) : 0.0;
    const double trackRms = trackErrN ? std::sqrt(trackErr2 / static_cast<double>(trackErrN)) : 0.0;
    const double falseRate = confirmedTrackSweeps
                                 ? 100.0 * static_cast<double>(falseTrackSweeps) /
                                       static_cast<double>(confirmedTrackSweeps)
                                 : 0.0;

    std::printf("\n=== Results ===\n");
    std::printf("Targets in coverage (final sweep): %zu / %d\n", inCoverage, numTargets);
    std::printf("Confirmed tracks (final sweep):    %d\n", tracker.confirmedCount());
    std::printf("Position RMS error, tracker:       %.1f m\n", trackRms);
    std::printf("Position RMS error, raw radar:     %.1f m  (%.0f%% reduction)\n", radarRms,
                radarRms > 0 ? 100.0 * (1.0 - trackRms / radarRms) : 0.0);
    std::printf("False-track rate:                  %.2f%% of confirmed track-sweeps "
                "(%zu of %zu confirmed track IDs ever false)\n",
                falseRate, falseTrackIds.size(), confirmedIds.size());
    std::printf("Track ID switches:                 %d\n", idSwitches);
    std::printf("Avg tracker time per sweep:        %.3f ms\n",
                1e3 * totalTrackerSeconds / static_cast<double>(numSweeps));
    std::printf("\nWrote truth.csv, detections.csv, tracks.csv\n");
    return 0;
}
