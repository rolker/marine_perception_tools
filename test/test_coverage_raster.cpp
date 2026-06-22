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

#include <vector>

#include "coverage_raster.hpp"
#include "sidescan_geometry.hpp"

using marine_perception_tools::CoverageRaster;
using marine_perception_tools::PingGeometry;
using marine_perception_tools::ping_covered_fraction;
using marine_perception_tools::paint_ping;
using marine_perception_tools::range_quality;

// 10x10 m raster at 1 m cells, origin (0,0): cols/rows 0..9.
CoverageRaster makeRaster() {return CoverageRaster(0.0, 0.0, 1.0, 10, 10);}

TEST(CoverageRaster, CellIndexAndBounds)
{
  auto r = makeRaster();
  int c = -1;
  int row = -1;
  EXPECT_TRUE(r.cellIndex(0.5, 0.5, c, row));
  EXPECT_EQ(c, 0);
  EXPECT_EQ(row, 0);
  EXPECT_TRUE(r.cellIndex(9.9, 4.2, c, row));
  EXPECT_EQ(c, 9);
  EXPECT_EQ(row, 4);
  EXPECT_FALSE(r.cellIndex(-0.1, 5.0, c, row));   // west of origin
  EXPECT_FALSE(r.cellIndex(10.0, 5.0, c, row));   // east edge is exclusive
}

TEST(CoverageRaster, PaintMarksCoverageOnce)
{
  auto r = makeRaster();
  EXPECT_FALSE(r.covered(3.5, 3.5));
  EXPECT_TRUE(r.paint(3.5, 3.5, 0.8f, 1.0f));
  EXPECT_TRUE(r.covered(3.5, 3.5));
  EXPECT_EQ(r.coveredCellCount(), 1u);
  EXPECT_FLOAT_EQ(r.amplitudeAt(3, 3), 0.8f);
  // Repainting the same cell does not double-count coverage.
  r.paint(3.5, 3.5, 0.2f, 2.0f);
  EXPECT_EQ(r.coveredCellCount(), 1u);
}

TEST(CoverageRaster, QualityWinsKeepsBetterLook)
{
  auto r = makeRaster();
  r.paint(5.0, 5.0, 0.3f, 1.0f);            // first look, quality 1.0
  EXPECT_FALSE(r.paint(5.0, 5.0, 0.9f, 0.5f));  // worse quality -> rejected
  EXPECT_FLOAT_EQ(r.amplitudeAt(5, 5), 0.3f);
  EXPECT_TRUE(r.paint(5.0, 5.0, 0.9f, 1.5f));   // better quality -> overwrites
  EXPECT_FLOAT_EQ(r.amplitudeAt(5, 5), 0.9f);
  EXPECT_TRUE(r.paint(5.0, 5.0, 0.7f, 1.5f));   // equal quality -> still wins (>=)
  EXPECT_FLOAT_EQ(r.amplitudeAt(5, 5), 0.7f);
}

TEST(CoverageRaster, OutOfBoundsPaintIgnored)
{
  auto r = makeRaster();
  EXPECT_FALSE(r.paint(-5.0, 5.0, 1.0f, 1.0f));
  EXPECT_EQ(r.coveredCellCount(), 0u);
  EXPECT_FLOAT_EQ(r.amplitudeAt(0, 0), -1.0f);  // uncovered reads -1
}

TEST(CoverageRaster, RangeQualityFallsWithRange)
{
  EXPECT_GT(range_quality(2.0), range_quality(20.0));
  EXPECT_GT(range_quality(0.0), 0.0f);
}

TEST(CoverageRaster, PaintPingAndCoveredFraction)
{
  // Port channel, heading east (yaw 0) at sensor (1,1): samples throw to +y.
  // metres_per_sample 1.0, unknown altitude (flat), so sample i lands at y=1+i.
  PingGeometry g;
  g.sensor_x = 1.0;
  g.sensor_y = 1.0;
  g.yaw = 0.0;
  g.metres_per_sample = 1.0;
  g.altitude = 0.0;
  g.lateral_sign = +1;
  std::vector<float> amps(6, 0.5f);  // slants 0..5 -> y = 1..6; sample 0 has range 0

  auto r = makeRaster();
  // Fresh raster: nothing covered yet.
  EXPECT_NEAR(ping_covered_fraction(r, g, amps), 0.0, 1e-9);
  const std::size_t written = paint_ping(r, g, amps);
  EXPECT_GT(written, 0u);
  // Re-running the same ping now finds every sample already covered.
  EXPECT_NEAR(ping_covered_fraction(r, g, amps), 1.0, 1e-9);
}
