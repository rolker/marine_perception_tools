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

#include "sidescan_geometry.hpp"

using marine_perception_tools::SidescanChannel;
using marine_perception_tools::channel_lateral_sign;
using marine_perception_tools::estimate_altitude_from_nadir;
using marine_perception_tools::ground_range;
using marine_perception_tools::PingGeometry;
using marine_perception_tools::project_sample;
using marine_perception_tools::slant_metres_per_sample;
using marine_perception_tools::slant_range_at;

constexpr double kEps = 1e-9;

TEST(SidescanGeometry, LateralSign)
{
  EXPECT_EQ(channel_lateral_sign(SidescanChannel::Port), +1);
  EXPECT_EQ(channel_lateral_sign(SidescanChannel::Starboard), -1);
  EXPECT_EQ(channel_lateral_sign(SidescanChannel::Down), 0);
}

TEST(SidescanGeometry, SlantMetresPerSample)
{
  // c / (2 fs): 1500 m/s at 75 kHz -> 0.01 m/sample.
  EXPECT_NEAR(slant_metres_per_sample(1500.0, 75000.0), 0.01, kEps);
  EXPECT_EQ(slant_metres_per_sample(1500.0, 0.0), 0.0);     // guard
  EXPECT_EQ(slant_metres_per_sample(1500.0, -5.0), 0.0);
}

TEST(SidescanGeometry, SlantRangeHonorsSample0Gate)
{
  EXPECT_NEAR(slant_range_at(0, 0, 0.1), 0.0, kEps);
  EXPECT_NEAR(slant_range_at(50, 0, 0.1), 5.0, kEps);
  EXPECT_NEAR(slant_range_at(50, 10, 0.1), 6.0, kEps);  // gate offset adds 10 samples
}

TEST(SidescanGeometry, GroundRangeFlatBottom)
{
  EXPECT_NEAR(ground_range(5.0, 3.0), 4.0, kEps);            // 3-4-5
  EXPECT_NEAR(ground_range(3.0, 3.0), 0.0, kEps);            // slant == altitude
  EXPECT_NEAR(ground_range(2.0, 3.0), 0.0, kEps);            // inside the nadir gap
  EXPECT_NEAR(ground_range(5.0, 0.0), 5.0, kEps);            // unknown altitude -> flat
  EXPECT_NEAR(ground_range(5.0, -1.0), 5.0, kEps);
}

TEST(SidescanGeometry, ProjectPortStarboardHeadingEast)
{
  // Heading east (yaw 0): "left of heading" is +y (north). Port throws to +y,
  // starboard to -y. Unknown altitude so ground range == slant range.
  PingGeometry g;
  g.sensor_x = 10.0;
  g.sensor_y = 20.0;
  g.yaw = 0.0;
  g.metres_per_sample = 0.1;
  g.altitude = 0.0;

  g.lateral_sign = +1;  // port
  auto port = project_sample(g, 50);  // slant 5 m
  EXPECT_TRUE(port.valid);
  EXPECT_NEAR(port.ground_range, 5.0, kEps);
  EXPECT_NEAR(port.x, 10.0, kEps);
  EXPECT_NEAR(port.y, 25.0, kEps);

  g.lateral_sign = -1;  // starboard
  auto stbd = project_sample(g, 50);
  EXPECT_NEAR(stbd.x, 10.0, kEps);
  EXPECT_NEAR(stbd.y, 15.0, kEps);
}

TEST(SidescanGeometry, ProjectHeadingNorthThrowsAcrossX)
{
  // Heading north (yaw pi/2): "left of heading" is -x (west). Port -> -x.
  PingGeometry g;
  g.sensor_x = 0.0;
  g.sensor_y = 0.0;
  g.yaw = M_PI / 2.0;
  g.metres_per_sample = 0.1;
  g.altitude = 0.0;
  g.lateral_sign = +1;
  auto p = project_sample(g, 30);  // slant 3 m
  EXPECT_NEAR(p.x, -3.0, 1e-6);
  EXPECT_NEAR(p.y, 0.0, 1e-6);
}

TEST(SidescanGeometry, ProjectAppliesSlantToGroundAndNadirGap)
{
  PingGeometry g;
  g.yaw = 0.0;
  g.metres_per_sample = 0.1;
  g.altitude = 3.0;     // 3 m above bottom
  g.lateral_sign = +1;

  auto inside = project_sample(g, 20);   // slant 2 m < altitude -> invalid
  EXPECT_FALSE(inside.valid);

  auto outside = project_sample(g, 50);  // slant 5 m -> ground 4 m
  EXPECT_TRUE(outside.valid);
  EXPECT_NEAR(outside.ground_range, 4.0, kEps);
  EXPECT_NEAR(outside.y, 4.0, kEps);
}

TEST(SidescanGeometry, EstimateAltitudeFromNadirFirstReturn)
{
  // Flat water column, a strong bottom return at sample 40.
  std::vector<float> amps(200, 0.05f);
  amps[40] = 1.0f;
  amps[41] = 0.9f;
  const double alt = estimate_altitude_from_nadir(amps, 0, 0.1, 0.5);
  EXPECT_NEAR(alt, 4.0, kEps);  // 40 samples * 0.1 m
}

TEST(SidescanGeometry, EstimateAltitudeNanWhenNoBottom)
{
  std::vector<float> amps(100, 0.1f);   // never crosses 0.5 * peak... peak is 0.1
  // peak 0.1, threshold 0.05 -> the very first gated sample (index 1) crosses,
  // so a flat ping returns an altitude, not NaN. Use an all-zero ping for NaN.
  std::vector<float> zero(100, 0.0f);
  EXPECT_TRUE(std::isnan(estimate_altitude_from_nadir(zero, 0, 0.1)));
  EXPECT_TRUE(std::isnan(estimate_altitude_from_nadir({}, 0, 0.1)));
  EXPECT_TRUE(std::isnan(estimate_altitude_from_nadir(amps, 0, 0.0)));  // bad scale
}
