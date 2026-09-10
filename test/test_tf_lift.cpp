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

// lift_sounding_to_world(): the one rigid sensor->world lift both bag paths
// use (#42). The regression it guards is not the arithmetic — that was always
// right — but the FIELDS: the two call sites used to rebuild the world
// sounding member by member and silently dropped `beam_angle` / `slant_range`,
// leaving every bag-loaded sounding with a NaN angle. The angle-aware
// uncertainty (#49) drops such a sounding outright, so a recurrence is data
// loss, not a degraded weight.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "tf_lift.hpp"

using marine_perception_tools::MbesSounding;
using marine_perception_tools::lift_sounding_to_world;

namespace
{

// A sounding with every field distinguishable, so a dropped one is visible.
MbesSounding sampleSounding()
{
  MbesSounding s;
  s.x = 1.0;
  s.y = 2.0;
  s.z = 3.0;
  s.intensity = -21.5f;
  s.beam_angle = 0.7853981634f;   // 45 deg to starboard
  s.slant_range = 17.25f;
  return s;
}

}  // namespace

// The identity transform must be a pure copy — nothing moves, nothing is lost.
TEST(TfLift, IdentityTransformCarriesEveryField)
{
  const auto s = sampleSounding();
  const auto w = lift_sounding_to_world(s, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

  EXPECT_DOUBLE_EQ(s.x, w.x);
  EXPECT_DOUBLE_EQ(s.y, w.y);
  EXPECT_DOUBLE_EQ(s.z, w.z);
  EXPECT_FLOAT_EQ(s.intensity, w.intensity);
  EXPECT_FLOAT_EQ(s.beam_angle, w.beam_angle);
  EXPECT_FLOAT_EQ(s.slant_range, w.slant_range);
}

// THE REGRESSION (#42). A rigid transform moves the position and nothing else:
// the beam angle and slant range are measured in the sensor frame and are the
// same numbers in world. A lift that rotates and translates must leave them —
// and the intensity, which is not geometry at all — exactly as they were.
TEST(TfLift, BeamGeometryAndIntensitySurviveARotatedTranslatedLift)
{
  const auto s = sampleSounding();
  // 90 deg about +z, then a translation well away from the origin.
  const double h = std::sqrt(0.5);
  const auto w = lift_sounding_to_world(s, 100.0, -50.0, 7.0, 0.0, 0.0, h, h);

  EXPECT_FLOAT_EQ(s.beam_angle, w.beam_angle);
  EXPECT_FLOAT_EQ(s.slant_range, w.slant_range);
  EXPECT_FLOAT_EQ(s.intensity, w.intensity);
  EXPECT_TRUE(std::isfinite(w.beam_angle));
  EXPECT_TRUE(std::isfinite(w.slant_range));
}

// The position itself: 90 deg about +z takes (x, y) -> (-y, x), then translate.
TEST(TfLift, PositionIsRotatedThenTranslated)
{
  const auto s = sampleSounding();
  const double h = std::sqrt(0.5);
  const auto w = lift_sounding_to_world(s, 100.0, -50.0, 7.0, 0.0, 0.0, h, h);

  EXPECT_NEAR(100.0 - 2.0, w.x, 1e-9);
  EXPECT_NEAR(-50.0 + 1.0, w.y, 1e-9);
  EXPECT_NEAR(7.0 + 3.0, w.z, 1e-9);
}

// A sounding whose geometry was never measured stays unmeasured — the lift
// must not fabricate a finite angle for it.
TEST(TfLift, UnknownGeometryStaysUnknown)
{
  MbesSounding s;
  s.x = 1.0;
  s.y = 2.0;
  s.z = 3.0;
  ASSERT_TRUE(std::isnan(s.beam_angle));
  ASSERT_TRUE(std::isnan(s.slant_range));

  const auto w = lift_sounding_to_world(s, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 1.0);

  EXPECT_TRUE(std::isnan(w.beam_angle));
  EXPECT_TRUE(std::isnan(w.slant_range));
}
