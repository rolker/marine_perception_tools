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

TEST(SidescanDrape, ExtendedTerrainReachesTheSwathAndFillsSmoothly)
{
  // A ping whose starboard swath (30 m of samples) runs far past the 20x20
  // (10 m) grid: the extended terrain must cover the reach, fill with the
  // plane depth, and mark every filled node as interpolated (NaN
  // uncertainty) while measured nodes keep theirs.
  const auto surface = flatSurface(20, 20, -10.0f);
  auto ping = makePing(5.0, 9.0);
  std::string note;
  const auto ext = marine_perception_tools::extend_surface_for_drape(
    surface, {ping}, 100000000ULL, note);
  ASSERT_TRUE(ext.ok());
  EXPECT_TRUE(note.empty());
  // Swath endpoint ~ (5, 9 - 30): well below the old grid's y=0 edge.
  EXPECT_LT(ext.origin_y, -20.0);
  std::size_t measured = 0;
  std::size_t filled = 0;
  for (std::size_t i = 0; i < ext.depth.size(); ++i) {
    ASSERT_TRUE(std::isfinite(ext.depth[i]));
    EXPECT_NEAR(ext.depth[i], -10.0f, 0.01f);   // membrane of a flat plane
    if (std::isfinite(ext.uncertainty[i])) {
      ++measured;
    } else {
      ++filled;
    }
  }
  EXPECT_EQ(measured, surface.depth.size());
  EXPECT_GT(filled, 0u);
  // Draping on the extended terrain paints beyond the old bathymetry.
  const auto drape = drape_pass(ext, {ping});
  bool painted_beyond = false;
  for (int y = 0; y < ext.ny; ++y) {
    const double wy = ext.origin_y + y * ext.cell_m;
    if (wy >= 0.0) {
      continue;   // inside the old grid's y range
    }
    for (int x = 0; x < ext.nx; ++x) {
      if (std::isfinite(
          drape.amplitude[static_cast<std::size_t>(y) * ext.nx + x]))
      {
        painted_beyond = true;
      }
    }
  }
  EXPECT_TRUE(painted_beyond);
}

TEST(SidescanDrape, StraightPassBeatsTurningPassInComposite)
{
  // Two three-ping sequences covering the same mid-swath cell: one running
  // straight (amplitudes = sample index), one mid-turn (constant 999).
  // The straightness score must let the straight pass's pixel through.
  const auto surface = flatSurface(40, 40, -10.0f);
  std::vector<WindowPing> pings;
  for (int i = -1; i <= 1; ++i) {
    pings.push_back(makePing(10.0 + 0.5 * i, 18.0));   // straight, yaw 0
  }
  for (int i = -1; i <= 1; ++i) {
    auto p = makePing(10.0 + 0.5 * i, 18.0);           // same track...
    p.geometry.yaw = 0.4 * i;                          // ...but turning hard
    for (auto & a : p.amplitudes) {
      a = 999.0f;
    }
    pings.push_back(p);
  }
  const auto drape = drape_pass(surface, pings);
  const int cx = 20;
  const int cy = static_cast<int>(std::lround(12.0 / 0.5));
  const float a = drape.amplitude[static_cast<std::size_t>(cy) * 40 + cx];
  ASSERT_TRUE(std::isfinite(a));
  EXPECT_NE(a, 999.0f);   // the turning pass lost the cell
}

TEST(SidescanDrape, RangeScoreModeFlipsConflicts)
{
  // Two pings covering the same cell: for the NEAR ping the cell sits at
  // ~24% of its swath, for the FAR ping at ~71%. Nearest mode keeps the
  // near ping's pixel; mid-range mode prefers the far ping's mid-swath one.
  const auto surface = flatSurface(40, 80, -10.0f);
  auto near_ping = makePing(10.0, 18.0);
  auto far_ping = makePing(10.0, 33.0);
  for (auto & a : far_ping.amplitudes) {
    a = 999.0f;
  }
  const int cx = 20;
  const int cy = static_cast<int>(std::lround(12.0 / 0.5));
  const std::size_t ci = static_cast<std::size_t>(cy) * 40 + cx;

  const auto nearest = drape_pass(
    surface, {near_ping, far_ping},
    marine_perception_tools::RangeScoreMode::Nearest);
  ASSERT_TRUE(std::isfinite(nearest.amplitude[ci]));
  EXPECT_NE(nearest.amplitude[ci], 999.0f);

  const auto mid = drape_pass(
    surface, {near_ping, far_ping},
    marine_perception_tools::RangeScoreMode::MidRange);
  ASSERT_TRUE(std::isfinite(mid.amplitude[ci]));
  EXPECT_EQ(mid.amplitude[ci], 999.0f);
}

TEST(SidescanDrape, AltitudelessPingDoesNotGrowTheTerrain)
{
  // A ping drape_pass() will always skip (no altitude) must not expand the
  // terrain: the extension budget is shared, so growing for a ping that can
  // never be painted takes nodes from the pings that can be.
  const auto surface = flatSurface(20, 20, -10.0f);
  auto no_altitude = makePing(5.0, 9.0, 0.0);
  std::string note;
  const auto ext = marine_perception_tools::extend_surface_for_drape(
    surface, {no_altitude}, 100000000ULL, note);
  ASSERT_TRUE(ext.ok());
  EXPECT_EQ(ext.nx, surface.nx);
  EXPECT_EQ(ext.ny, surface.ny);
  EXPECT_DOUBLE_EQ(ext.origin_x, surface.origin_x);
  EXPECT_DOUBLE_EQ(ext.origin_y, surface.origin_y);
  // Sanity: the same ping WITH an altitude does grow it, so the guard above
  // is what stopped the growth, not a geometry that never reached out.
  auto with_altitude = makePing(5.0, 9.0);
  const auto grown = marine_perception_tools::extend_surface_for_drape(
    surface, {with_altitude}, 100000000ULL, note);
  EXPECT_LT(grown.origin_y, surface.origin_y);
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
