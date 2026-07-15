// Copyright 2026 Roland Arsenault
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Headless verification of the sidescan ingest core: open a bag, build the ping
// index, and print a summary (channel counts, distance span, pose-resolution
// rate, geo-reference availability, and one projected sample). No Qt — this is
// the runnable end-to-end check for PR1 against a real bag, the sidescan analogue
// of sea_surface_tuner's `--probe`.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

#include "mbes_window_reader.hpp"
#include "sidescan_bag_session.hpp"
#include "sidescan_geometry.hpp"

namespace
{
const char * channel_name(marine_perception_tools::SidescanChannel ch)
{
  using marine_perception_tools::SidescanChannel;
  switch (ch) {
    case SidescanChannel::Port: return "port";
    case SidescanChannel::Starboard: return "starboard";
    case SidescanChannel::Down: return "down";
  }
  return "?";
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr,
      "usage: sidescan_probe <bag_uri>\n"
      "       sidescan_probe --mbes-window <bag_uri> <t_start_ns> <t_end_ns>\n");
    return 2;
  }
  namespace mpt = marine_perception_tools;

  // One-shot windowed MBES read (#21): the headless check for the multi-pass
  // cloud's ingest path — no session, no full-bag metadata scan. Timed, since
  // avoiding the full scan is the point.
  if (std::string(argv[1]) == "--mbes-window") {
    if (argc < 5) {
      std::fprintf(stderr,
        "usage: sidescan_probe --mbes-window <bag_uri> <t_start_ns> <t_end_ns>\n");
      return 2;
    }
    try {
      const auto t0 = std::chrono::steady_clock::now();
      const auto res = mpt::read_mbes_window(
        argv[2], std::strtoll(argv[3], nullptr, 10), std::strtoll(argv[4], nullptr, 10));
      const auto t1 = std::chrono::steady_clock::now();
      std::printf(
        "mbes window: %zu soundings from %d pings (%d skipped, no TF) in %.0f ms\n"
        "  world frame: %s • geo anchor (earth<-world): %s\n",
        res.world_soundings.size(), res.used_pings, res.skipped_pings,
        std::chrono::duration<double, std::milli>(t1 - t0).count(),
        res.world_frame.c_str(), res.has_geo ? "captured" : "ABSENT");
      if (!res.world_soundings.empty()) {
        const auto & s = res.world_soundings.front();
        std::printf("  first sounding world=(%.2f, %.2f, %.2f) dB=%.1f\n",
          s.x, s.y, s.z, static_cast<double>(s.intensity));
      }
      return 0;
    } catch (const std::exception & e) {
      std::fprintf(stderr, "error: %s\n", e.what());
      return 1;
    }
  }
  using mpt::project_sample;
  using mpt::SidescanChannel;
  try {
    mpt::SidescanBagSession session(argv[1]);
    session.buildIndex();   // synchronous full build (one final snapshot)
    std::printf("pings: %zu total\n", session.pingCount());
    std::printf("  port=%zu starboard=%zu down=%zu\n",
      session.channelCount(SidescanChannel::Port),
      session.channelCount(SidescanChannel::Starboard),
      session.channelCount(SidescanChannel::Down));
    std::printf("along-track distance: %.1f m\n", session.totalDistance());
    std::printf("poses: %zu resolved, %zu skipped\n",
      session.posesResolved(), session.posesSkipped());
    std::printf("decode errors (skipped messages): %zu\n", session.decodeErrors());
    std::printf("geo-reference (earth->world): %s\n",
      session.hasGeoReference() ? "available" : "ABSENT (datum fallback needed)");
    std::printf("altitude source: %s\n",
      session.usedNadirDepth() ? "nadir_depth (driver)" : "amplitude estimator (fallback)");

    // Re-read the first 50 m window's samples and project a sample, exercising
    // the windowed read path end-to-end.
    const auto win = session.readWindow(0.0, 50.0, 600);
    if (!win.empty()) {
      const auto & p = win.front();
      const std::size_t mid = p.amplitudes.size() / 2;
      const auto gp = project_sample(p.geometry, mid);
      std::printf(
        "window read (first 50 m): %zu pings • %s ping samples=%zu alt=%.2f m\n"
        "  mid-sample[%zu] ground_range=%.2f m -> map (%.2f, %.2f) valid=%d\n",
        win.size(), channel_name(p.channel), p.amplitudes.size(),
        p.geometry.altitude, mid, gp.ground_range, gp.x, gp.y, gp.valid);
    }

    // M3 multibeam: index count + a windowed read exercising the project + lift.
    const auto snap = session.snapshot();
    std::printf(
      "mbes detections: %zu pings indexed\n", snap ? snap->mbes_pings.size() : 0);
    const auto mwin = session.readMbesWindow(0.0, 50.0, 600);
    if (!mwin.empty()) {
      std::size_t soundings = 0;
      for (const auto & mp : mwin) {
        soundings += mp.world_soundings.size();
      }
      const auto & m0 = mwin.front();
      std::printf(
        "mbes window read (first 50 m): %zu pings • %zu soundings • beams/ping=%zu\n",
        mwin.size(), soundings, m0.intensities.size());
      if (!m0.world_soundings.empty()) {
        const auto & s = m0.world_soundings.front();
        std::printf(
          "  first sounding world=(%.2f, %.2f, %.2f) dB=%.1f\n",
          s.x, s.y, s.z, static_cast<double>(s.intensity));
      }
    }

    // Time a window read at each end of the track — this is the per-scrub cost on
    // the UI thread. An end-of-track read that is far slower than a start read
    // means seek() is not repositioning (it falls back to a scan from the start).
    const double total = session.totalDistance();
    auto time_window = [&session](double lo, double hi) {
        const auto t0 = std::chrono::steady_clock::now();
        const auto w = session.readWindow(lo, hi, 600);
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        return std::make_pair(w.size(), ms);
      };
    const auto [n_start, ms_start] = time_window(0.0, 100.0);
    const auto [n_end, ms_end] = time_window(total - 100.0, total);
    std::printf("readWindow timing: start[0,100]=%.0f ms (%zu pings), "
      "end[%.0f,%.0f]=%.0f ms (%zu pings)\n",
      ms_start, n_start, total - 100.0, total, ms_end, n_end);
    return 0;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
