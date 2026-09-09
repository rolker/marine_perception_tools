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

// The CUBE lab's angle-aware per-sounding uncertainty (#49): the outer-beam
// behaviour the operator is after, the nadir reduction that pins the model to
// its range term, and the refusal to answer for geometry that is not there.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "sounding_uncertainty.hpp"

namespace
{

using marine_perception_tools::SoundingUncertainty;
using marine_perception_tools::beam_angle_sigma_rad;
using marine_perception_tools::kRangeErrorFloorM;
using marine_perception_tools::range_sigma_m;
using marine_perception_tools::sounding_uncertainty;

double vertical_std(double angle_rad, double range_m)
{
  SoundingUncertainty u;
  EXPECT_TRUE(sounding_uncertainty(angle_rad, range_m, &u));
  return std::sqrt(u.vertical_variance);
}

// The defect this closes: vertical uncertainty must GROW with beam angle for
// beams over the SAME DEPTH — one node seen by two passes, which is the
// comparison CUBE actually makes when it weights soundings. Before #49 it
// depended on depth alone, so a swath-edge beam was trusted exactly as much as
// a nadir one.
TEST(SoundingUncertainty, VerticalGrowsWithAngleOverTheSameDepth)
{
  const double depth = 15.0;
  const auto at = [depth](double deg) {
      const double t = deg * M_PI / 180.0;
      return vertical_std(t, depth / std::cos(t));   // same seabed, steered out
    };
  double previous = at(0.0);
  for (int deg = 5; deg <= 70; deg += 5) {
    const double next = at(deg);
    EXPECT_GT(next, previous) << "at " << deg << " deg";
    previous = next;
  }
  // The growth is worth having, not a rounding artefact: in CUBE's
  // inverse-variance weighting a 60 deg beam counts about half a nadir one.
  EXPECT_GT(at(60.0), 1.3 * at(0.0));
  EXPECT_GT(at(70.0), 1.8 * at(0.0));
}

// The floor must not reintroduce the inversion. In water shallow enough for
// the 0.05 m to bind, uncertainty may go flat with angle but must never FALL —
// a swath edge that came out more certain than nadir would demote exactly the
// wrong beams.
TEST(SoundingUncertainty, ShallowWaterNeverInvertsTheOrdering)
{
  for (const double depth : {2.0, 5.0, 8.0, 10.0}) {
    double previous = vertical_std(0.0, depth);
    for (int deg = 5; deg <= 70; deg += 5) {
      const double t = deg * M_PI / 180.0;
      const double next = vertical_std(t, depth / std::cos(t));
      EXPECT_GE(next, previous - 1e-12) << depth << " m at " << deg << " deg";
      previous = next;
    }
  }
}

// What the model does NOT claim, pinned so the absence is recorded rather than
// assumed: at a fixed SLANT RANGE sigma_z falls with angle, because an outer
// beam at the same range is over shallower water and its range error projects
// mostly sideways. Geometry, not a defect — but it is why the test above
// compares at fixed depth.
TEST(SoundingUncertainty, AtFixedSlantRangeVerticalFallsWithAngle)
{
  const double range = 40.0;   // deep enough that the floor never binds
  EXPECT_LT(vertical_std(60.0 * M_PI / 180.0, range), vertical_std(0.0, range));
}

// At nadir the angular term is multiplied by sin(0): the model must reduce to
// the range term alone, exactly — and that value is cube::ErrorModel's own
// range_error, max(percent * R, floor).
TEST(SoundingUncertainty, NadirReducesToTheRangeTerm)
{
  for (const double range : {0.0, 5.0, 30.0, 200.0}) {
    SoundingUncertainty u;
    ASSERT_TRUE(sounding_uncertainty(0.0, range, &u));
    const double expected = std::fmax(range_sigma_m(range), kRangeErrorFloorM);
    EXPECT_DOUBLE_EQ(std::sqrt(u.vertical_variance), expected) << "range " << range;
    // ... and all of the angular term has gone the other way, into horizontal,
    // where it rides on top of the inherited positioning stand-in.
    const double angular = range * beam_angle_sigma_rad();
    const double pos = marine_perception_tools::kPositionStdFixedM +
      marine_perception_tools::kPositionStdPercentOfDepth * range;
    EXPECT_DOUBLE_EQ(u.horizontal_variance, angular * angular + pos * pos)
      << "range " << range;
  }
  // The floor keeps a short-range beam from claiming zero error.
  SoundingUncertainty u;
  ASSERT_TRUE(sounding_uncertainty(0.0, 0.0, &u));
  EXPECT_DOUBLE_EQ(std::sqrt(u.vertical_variance), kRangeErrorFloorM);
}

// The angular term scales with range, and the closed form is pinned so the
// formula itself is under test, not merely its monotonicity.
TEST(SoundingUncertainty, AngularTermScalesWithRange)
{
  const double angle = 55.0 * M_PI / 180.0;
  SoundingUncertainty near_beam;
  SoundingUncertainty far_beam;
  ASSERT_TRUE(sounding_uncertainty(angle, 10.0, &near_beam));
  ASSERT_TRUE(sounding_uncertainty(angle, 100.0, &far_beam));
  EXPECT_GT(far_beam.vertical_variance, near_beam.vertical_variance);
  const double expected =
    std::pow(range_sigma_m(100.0) * std::cos(angle), 2.0) +
    std::pow(100.0 * beam_angle_sigma_rad() * std::sin(angle), 2.0);
  EXPECT_NEAR(far_beam.vertical_variance, expected, 1e-12);
}

// Sign symmetry: a port beam and a starboard beam at the same angle from nadir
// carry the same uncertainty.
TEST(SoundingUncertainty, SymmetricAboutNadir)
{
  const double angle = 40.0 * M_PI / 180.0;
  SoundingUncertainty port;
  SoundingUncertainty starboard;
  ASSERT_TRUE(sounding_uncertainty(-angle, 25.0, &port));
  ASSERT_TRUE(sounding_uncertainty(angle, 25.0, &starboard));
  EXPECT_DOUBLE_EQ(port.vertical_variance, starboard.vertical_variance);
  EXPECT_DOUBLE_EQ(port.horizontal_variance, starboard.horizontal_variance);
}

// Degenerate geometry yields NO answer — the caller drops the sounding rather
// than handing the estimator a fabricated confidence.
TEST(SoundingUncertainty, DegenerateGeometryYieldsNothing)
{
  const double nan_v = std::numeric_limits<double>::quiet_NaN();
  const double inf_v = std::numeric_limits<double>::infinity();
  SoundingUncertainty u;
  EXPECT_FALSE(sounding_uncertainty(nan_v, 20.0, &u));
  EXPECT_FALSE(sounding_uncertainty(0.3, nan_v, &u));
  EXPECT_FALSE(sounding_uncertainty(inf_v, 20.0, &u));
  EXPECT_FALSE(sounding_uncertainty(0.3, inf_v, &u));
  EXPECT_FALSE(sounding_uncertainty(0.3, -1.0, &u));   // range behind the head
  EXPECT_FALSE(sounding_uncertainty(0.3, 20.0, nullptr));
  // A rejected call leaves the output untouched.
  SoundingUncertainty untouched;
  untouched.vertical_variance = 7.0;
  EXPECT_FALSE(sounding_uncertainty(nan_v, nan_v, &untouched));
  EXPECT_DOUBLE_EQ(untouched.vertical_variance, 7.0);
}

// The constants are the library's device defaults, not numbers invented here.
TEST(SoundingUncertainty, ConstantsMatchTheDeviceDefaults)
{
  EXPECT_DOUBLE_EQ(marine_perception_tools::kRangeErrorPercent, 0.005);
  EXPECT_DOUBLE_EQ(marine_perception_tools::kRangeErrorFloorM, 0.05);
  EXPECT_DOUBLE_EQ(marine_perception_tools::kAcrossTrackBeamwidthDeg, 2.0);
  // The beamwidth-to-sigma convention is ErrorModel's: beamwidth (rad) / 12.
  EXPECT_DOUBLE_EQ(marine_perception_tools::kBeamwidthToSigmaDivisor, 12.0);
  EXPECT_NEAR(beam_angle_sigma_rad(), (2.0 * M_PI / 180.0) / 12.0, 1e-15);
}

// The horizontal budget keeps the positioning stand-in the depth-only
// placeholder carried (see kPositionStdFixedM): beam geometry alone would put
// sigma_y in the centimetres, which claims a positioning accuracy nothing here
// measured and — because CUBE spends sigma_y on the influence radius — would
// quietly pin every radius to the cell size and make the operator's
// uncertainty-budget control inert.
TEST(SoundingUncertainty, HorizontalKeepsThePositioningStandIn)
{
  SoundingUncertainty u;
  ASSERT_TRUE(sounding_uncertainty(0.0, 10.0, &u));
  // 0.2 m + 1% of depth is the incumbent floor on horizontal, not centimetres.
  EXPECT_GT(std::sqrt(u.horizontal_variance), 0.29);
  // ... and the beam-geometry term still rides on top of it: an outer beam is
  // less well placed than a nadir one over the same depth.
  SoundingUncertainty outer;
  ASSERT_TRUE(sounding_uncertainty(60.0 * M_PI / 180.0, 20.0, &outer));
  EXPECT_GT(outer.horizontal_variance, u.horizontal_variance);
}

}  // namespace
