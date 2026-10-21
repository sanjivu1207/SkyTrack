# SkyTrack

**A radar multi-target tracking system written from scratch in C++17.**

SkyTrack simulates a sky full of aircraft and a realistic, imperfect radar that watches them.
The radar adds noise, misses some aircraft, and reports fake blips. A tracker then turns
that output back into clean, stable tracks with an ID for each aircraft. This is the same
core problem solved in air-traffic control, defence surveillance, and self-driving-car
perception.

![Tracker output: true aircraft paths, radar detections, clutter and confirmed tracks](radar-tracker/tracks.png)

## Highlights

- **Kalman filtering**: estimates each aircraft's position *and* velocity from noisy
  position-only radar measurements.
- **Statistical gating**: uses Mahalanobis distance and a chi-square test (99.9%) to rule
  out pairings of a track with a blip that are statistically implausible.
- **Optimal data association**: the Hungarian algorithm (O(n³)) matches tracks to
  detections one-to-one at the lowest total cost.
- **Track management**: tracks are confirmed only after 3 hits within 5 sweeps, so false
  tracks from radar clutter are rare.
- **Realistic radar model**: noise is added to range and bearing; each detection's error
  ellipse is rotated to its bearing and grows with distance.
- **No dependencies**: includes its own compile-time-sized matrix library; builds clean
  under `-Wall -Wextra -Wpedantic`, with unit and end-to-end tests run by CTest.

## Results

20 aircraft, 200 radar sweeps, about 5 false blips per sweep. Averages over 20 random runs:

| Metric | Result |
|---|---|
| Position error, tracker vs. raw radar | **44.8 m vs. 78.5 m** (~43% more accurate) |
| False-track rate | **0.02%** of confirmed track-sweeps |
| Track ID switches | **0.25 per run** |
| Processing time | **~0.03 ms per sweep** |

## Quick start

```bash
cd radar-tracker
cmake -S . -B build && cmake --build build
ctest --test-dir build --output-on-failure   # run the tests
./build/radar_tracker                        # run the simulation
python3 plot_tracks.py                       # plot it (needs matplotlib)
```

## Learn more

The full technical write-up is in [radar-tracker/README.md](radar-tracker/README.md). It
covers how each pipeline stage works, the build options, the CSV output format, how the
results are scored, how the tracker was tuned, and future work.

## Tech

C++17 · CMake · CTest · Python / matplotlib
