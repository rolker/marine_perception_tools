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

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "sidescan_drape.hpp"

namespace
{

using marine_perception_tools::CubeSurface;
using marine_perception_tools::SidescanChannel;
using marine_perception_tools::WindowPing;
using marine_perception_tools::drape_pass;

// A synthetic surface: nx x ny grid at 0.5 m cells, flat at `z` (every node
// estimated), origin at (0, 0).
CubeSurface flatSurface(int nx, int ny, float z)
{
  CubeSurface s;
  s.origin_x = 0.0;
  s.origin_y = 0.0;
  s.cell_m = 0.5;
  s.nx = nx;
  s.ny = ny;
  const std::size_t n = static_cast<std::size_t>(nx) * ny;
  s.depth.assign(n, z);
  s.uncertainty.assign(n, 0.1f);
  s.intensity.assign(n, std::nanf(""));
  return s;
}

// One starboard ping at (x, y) heading +x (yaw 0; starboard = -y side),
// 4 m above the seabed, samples 0.1 m apart with amplitude == sample index
// so the slant->sample mapping is observable.
WindowPing makePing(double x, double y, double altitude = 4.0)
{
  WindowPing p;
  p.channel = SidescanChannel::Starboard;
  p.geometry.sensor_x = x;
  p.geometry.sensor_y = y;
  p.geometry.yaw = 0.0;
  p.geometry.sample0 = 0;
  p.geometry.metres_per_sample = 0.1;
  p.geometry.altitude = altitude;
  p.geometry.lateral_sign = -1;   // starboard
  p.amplitudes.resize(300);
  for (std::size_t i = 0; i < p.amplitudes.size(); ++i) {
    p.amplitudes[i] = static_cast<float>(i);
  }
  return p;
}

TEST(SidescanDrape, FlatSeabedMapsSlantRangesOntoTheTrack)
{
  // Sensor mid-grid at (10, 18), z = -10 + 4 = -6; the starboard march runs
  // toward -y across the grid.
  const auto surface = flatSurface(40, 40, -10.0f);
  const auto drape = drape_pass(surface, {makePing(10.0, 18.0)});
  ASSERT_TRUE(drape.ok());
  EXPECT_EQ(drape.pings_used, 1u);
  EXPECT_EQ(drape.pings_skipped, 0u);

  // A cell 6 m across-track: slant = sqrt(36 + 16) ~ 7.21 m -> sample ~72.
  const int cx = static_cast<int>(std::lround(10.0 / 0.5));
  const int cy = static_cast<int>(std::lround((18.0 - 6.0) / 0.5));
  const float a = drape.amplitude[static_cast<std::size_t>(cy) * 40 + cx];
  ASSERT_TRUE(std::isfinite(a));
  EXPECT_NEAR(a, std::sqrt(36.0 + 16.0) / 0.1, 1.5);
  // No terrain, no shadows.
  for (const auto sh : drape.shadow) {
    EXPECT_EQ(sh, 0);
  }
}

TEST(SidescanDrape, RidgeCastsAnAcousticShadow)
{
  // A 3 m ridge across the swath at y in [12, 12.5): cells beyond it (lower
  // y) that fall below the grazing ray are marked shadow, not painted.
  auto surface = flatSurface(40, 40, -10.0f);
  for (int x = 0; x < 40; ++x) {
    for (int y = 24; y <= 25; ++y) {   // world y = 12.0, 12.5
      surface.depth[static_cast<std::size_t>(y) * 40 + x] = -7.0f;
    }
  }
  const auto drape = drape_pass(surface, {makePing(10.0, 18.0)});
  ASSERT_TRUE(drape.ok());
  // The ridge top itself is visible (and painted).
  const int cx = 20;
  EXPECT_TRUE(std::isfinite(drape.amplitude[25u * 40 + cx]));
  // Immediately behind the ridge (world y ~ 11.0): shadowed.
  const int cy_behind = static_cast<int>(std::lround(11.0 / 0.5));
  EXPECT_EQ(drape.shadow[static_cast<std::size_t>(cy_behind) * 40 + cx], 1);
  EXPECT_FALSE(
    std::isfinite(drape.amplitude[static_cast<std::size_t>(cy_behind) * 40 + cx]));
}

TEST(SidescanDrape, NearerSlantWinsCellConflicts)
{
  // Two pings of the pass over the same cells: the nearer sensor's smaller
  // slant range must win.
  const auto surface = flatSurface(40, 40, -10.0f);
  const auto near_ping = makePing(10.0, 18.0);
  const auto far_ping = makePing(10.0, 19.5);
  const auto drape = drape_pass(surface, {far_ping, near_ping});
  const int cx = 20;
  const int cy = static_cast<int>(std::lround(12.0 / 0.5));
  const std::size_t ci = static_cast<std::size_t>(cy) * 40 + cx;
  ASSERT_TRUE(std::isfinite(drape.amplitude[ci]));
  // Expected from the NEAR ping (across-track 6 m), not the far one (7.5 m).
  EXPECT_NEAR(drape.amplitude[ci], std::sqrt(36.0 + 16.0) / 0.1, 1.5);
}

TEST(SidescanDrape, PingStripsTileTheAlongTrackGaps)
{
  // Two pings 1 m apart along-track: each paints a strip half the spacing
  // wide on both sides, so the cells BETWEEN the rays are covered too —
  // no grey stripes between pings.
  const auto surface = flatSurface(40, 40, -10.0f);
  const auto drape =
    drape_pass(surface, {makePing(9.0, 18.0), makePing(10.0, 18.0)});
  ASSERT_TRUE(drape.ok());
  // A mid-gap cell (x = 9.5) well inside the swath (6 m across-track).
  const int cx = static_cast<int>(std::lround(9.5 / 0.5));
  const int cy = static_cast<int>(std::lround(12.0 / 0.5));
  EXPECT_TRUE(
    std::isfinite(drape.amplitude[static_cast<std::size_t>(cy) * 40 + cx]));
}

TEST(SidescanDrape, UnusablePingsAreCountedSkipped)
{
  const auto surface = flatSurface(20, 20, -10.0f);
  auto no_altitude = makePing(5.0, 5.0, 0.0);
  auto off_surface = makePing(500.0, 500.0);
  const auto drape = drape_pass(surface, {no_altitude, off_surface});
  EXPECT_EQ(drape.pings_used, 0u);
  EXPECT_EQ(drape.pings_skipped, 2u);
}

}  // namespace
