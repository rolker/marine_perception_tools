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

#include <cstdio>
#include <exception>
#include <string>

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
    std::fprintf(stderr, "usage: sidescan_probe <bag_uri>\n");
    return 2;
  }
  namespace mpt = marine_perception_tools;
  using mpt::project_sample;
  using mpt::SidescanChannel;
  try {
    mpt::SidescanBagSession session(argv[1]);
    const auto & pings = session.pings();
    std::printf("pings: %zu total\n", pings.size());
    std::printf("  port=%zu starboard=%zu down=%zu\n",
      session.channelCount(SidescanChannel::Port),
      session.channelCount(SidescanChannel::Starboard),
      session.channelCount(SidescanChannel::Down));
    std::printf("along-track distance: %.1f m\n", session.totalDistance());
    std::printf("poses: %zu resolved, %zu skipped\n",
      session.posesResolved(), session.posesSkipped());
    std::printf("geo-reference (earth->world): %s\n",
      session.hasGeoReference() ? "available" : "ABSENT (datum fallback needed)");
    std::printf("altitude source: %s\n",
      session.usedNadirDepth() ? "nadir_depth (driver)" : "amplitude estimator (fallback)");

    // Project the middle sample of the first paintable port/starboard ping.
    for (const auto & p : pings) {
      if (!p.has_pose || p.amplitudes.empty() ||
        p.channel == SidescanChannel::Down) {continue;}
      const std::size_t mid = p.amplitudes.size() / 2;
      const auto gp = project_sample(p.geometry, mid);
      std::printf(
        "sample probe: %s ping t=%.2fs samples=%zu sound_speed=%.1f "
        "m/s alt=%.2f m\n  mid-sample[%zu] ground_range=%.2f m -> map (%.2f, %.2f) valid=%d\n",
        channel_name(p.channel), p.stamp_s, p.amplitudes.size(), p.sound_speed,
        p.geometry.altitude, mid, gp.ground_range, gp.x, gp.y, gp.valid);
      break;
    }
    return 0;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
