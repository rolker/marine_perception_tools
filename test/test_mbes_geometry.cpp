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

#include "marine_acoustic_msgs/msg/sonar_detections.hpp"
#include "mbes_geometry.hpp"

using marine_perception_tools::MbesSounding;
using marine_perception_tools::project_beam;
using marine_perception_tools::project_detections;

namespace
{
constexpr double kSoundSpeed = 1500.0;

// Two-way travel time (s) that yields a given one-way range (m) at kSoundSpeed.
double twtt_for_range(double range_m)
{
  return 2.0 * range_m / kSoundSpeed;
}
}  // namespace

// Nadir beam (tx=0, rx=0): the sounding sits straight below at z=range.
TEST(MbesGeometry, NadirBeam)
{
  const MbesSounding s = project_beam(twtt_for_range(10.0), 0.0, 0.0, kSoundSpeed);
  EXPECT_NEAR(s.x, 0.0, 1e-9);
  EXPECT_NEAR(s.y, 0.0, 1e-9);
  EXPECT_NEAR(s.z, 10.0, 1e-9);
}

// Across-track (rx) beam throws the sounding to +y and shortens z by cos(rx).
TEST(MbesGeometry, AcrossTrackBeamMatchesFormula)
{
  const double rx = 0.5;  // rad
  const MbesSounding s = project_beam(twtt_for_range(10.0), 0.0, rx, kSoundSpeed);
  EXPECT_NEAR(s.x, 0.0, 1e-9);
  EXPECT_NEAR(s.y, 10.0 * std::sin(rx), 1e-9);
  EXPECT_NEAR(s.z, 10.0 * std::cos(rx), 1e-9);
}

// Along-track (tx) tilt throws the sounding to -x (cube convention) and scales z.
TEST(MbesGeometry, AlongTrackTiltMatchesFormula)
{
  const double tx = 0.3;
  const double rx = 0.2;
  const MbesSounding s = project_beam(twtt_for_range(10.0), tx, rx, kSoundSpeed);
  EXPECT_NEAR(s.x, 10.0 * -std::sin(tx), 1e-9);
  EXPECT_NEAR(s.y, 10.0 * std::sin(rx), 1e-9);
  EXPECT_NEAR(s.z, 10.0 * std::cos(tx) * std::cos(rx), 1e-9);
}

// project_detections skips non-positive travel times, copies intensities, and
// agrees beam-for-beam with project_beam.
TEST(MbesGeometry, ProjectDetectionsSkipsAndCopies)
{
  marine_acoustic_msgs::msg::SonarDetections d;
  d.ping_info.sound_speed = static_cast<float>(kSoundSpeed);
  d.two_way_travel_times = {
    static_cast<float>(twtt_for_range(5.0)),
    0.0f,                                       // no detection -> skipped
    static_cast<float>(twtt_for_range(20.0))};
  d.tx_angles = {0.0f, 0.1f, 0.2f};
  d.rx_angles = {0.3f, 0.4f, -0.5f};
  d.intensities = {-12.0f, -3.0f, -25.0f};

  const std::vector<MbesSounding> out = project_detections(d);
  ASSERT_EQ(out.size(), 2u) << "the zero-twtt beam must be skipped";

  const MbesSounding b0 = project_beam(twtt_for_range(5.0), 0.0, 0.3, kSoundSpeed);
  EXPECT_NEAR(out[0].x, b0.x, 1e-6);
  EXPECT_NEAR(out[0].y, b0.y, 1e-6);
  EXPECT_NEAR(out[0].z, b0.z, 1e-6);
  EXPECT_FLOAT_EQ(out[0].intensity, -12.0f);

  // Second surviving sounding is the third beam (index 2).
  const MbesSounding b2 = project_beam(twtt_for_range(20.0), 0.2, -0.5, kSoundSpeed);
  EXPECT_NEAR(out[1].x, b2.x, 1e-6);
  EXPECT_NEAR(out[1].z, b2.z, 1e-6);
  EXPECT_FLOAT_EQ(out[1].intensity, -25.0f);
}

// A zero/!positive sound speed yields no soundings (unusable scale).
TEST(MbesGeometry, ZeroSoundSpeedYieldsNothing)
{
  marine_acoustic_msgs::msg::SonarDetections d;
  d.ping_info.sound_speed = 0.0f;
  d.two_way_travel_times = {static_cast<float>(twtt_for_range(10.0))};
  d.tx_angles = {0.0f};
  d.rx_angles = {0.0f};
  EXPECT_TRUE(project_detections(d).empty());
}
