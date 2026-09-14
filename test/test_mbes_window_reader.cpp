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
#include <string>

#include "mbes_window_reader.hpp"
#include "tf_lift.hpp"

namespace
{

using marine_perception_tools::FrameReprojection;
using marine_perception_tools::apply_reprojection;
using marine_perception_tools::lookup_at_or_latest;
using marine_perception_tools::make_reprojection;
using marine_perception_tools::rotate_by_quat;

geometry_msgs::msg::TransformStamped makeTransform(
  double tx, double ty, double tz,
  double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0)
{
  geometry_msgs::msg::TransformStamped t;
  t.transform.translation.x = tx;
  t.transform.translation.y = ty;
  t.transform.translation.z = tz;
  t.transform.rotation.x = qx;
  t.transform.rotation.y = qy;
  t.transform.rotation.z = qz;
  t.transform.rotation.w = qw;
  return t;
}

// --- rotate_by_quat -------------------------------------------------------

TEST(TfLift, RotateByIdentityQuatIsNoop)
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  rotate_by_quat(0.0, 0.0, 0.0, 1.0, 1.5, -2.5, 3.5, x, y, z);
  EXPECT_DOUBLE_EQ(x, 1.5);
  EXPECT_DOUBLE_EQ(y, -2.5);
  EXPECT_DOUBLE_EQ(z, 3.5);
}

TEST(TfLift, RotateNinetyDegreesAboutZ)
{
  // q = 90° about +z maps +x to +y.
  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  rotate_by_quat(0.0, 0.0, s, c, 1.0, 0.0, 0.0, x, y, z);
  EXPECT_NEAR(x, 0.0, 1e-12);
  EXPECT_NEAR(y, 1.0, 1e-12);
  EXPECT_NEAR(z, 0.0, 1e-12);
}

// --- lookup_at_or_latest --------------------------------------------------

TEST(TfLift, LookupFallsBackToLatestOnExtrapolation)
{
  tf2::BufferCore buffer;
  geometry_msgs::msg::TransformStamped tr = makeTransform(10.0, 0.0, 0.0);
  tr.header.frame_id = "world";
  tr.child_frame_id = "sensor";
  tr.header.stamp.sec = 100;
  ASSERT_TRUE(buffer.setTransform(tr, "test", false));

  // A stamp far beyond the single sample would extrapolate; the helper must
  // fall back to the latest available transform instead of failing.
  geometry_msgs::msg::TransformStamped out;
  const auto late = tf2::TimePoint(std::chrono::seconds(500));
  EXPECT_TRUE(lookup_at_or_latest(buffer, "world", "sensor", late, out));
  EXPECT_DOUBLE_EQ(out.transform.translation.x, 10.0);
}

TEST(TfLift, LookupFailsCleanlyOnUnknownFrame)
{
  const tf2::BufferCore buffer;
  geometry_msgs::msg::TransformStamped out;
  EXPECT_FALSE(
    lookup_at_or_latest(
      buffer, "world", "nowhere", tf2::TimePoint(std::chrono::seconds(1)), out));
}

// --- make_reprojection / apply_reprojection --------------------------------

TEST(FrameReprojectionTest, DefaultIsIdentity)
{
  const FrameReprojection r;
  double x = 3.0;
  double y = -4.0;
  double z = 5.0;
  apply_reprojection(r, x, y, z);
  EXPECT_DOUBLE_EQ(x, 3.0);
  EXPECT_DOUBLE_EQ(y, -4.0);
  EXPECT_DOUBLE_EQ(z, 5.0);
}

TEST(FrameReprojectionTest, EqualAnchorsComposeToIdentity)
{
  // Same earth<-world on both sides: src IS ref, points must not move.
  const auto anchor = makeTransform(1000.0, -2000.0, 300.0, 0.1, 0.2, 0.3,
      std::sqrt(1.0 - 0.01 - 0.04 - 0.09));
  const auto r = make_reprojection(anchor, anchor);
  // The numerically-identity composition must be DETECTED, not just harmless:
  // apply_reprojection() then skips the per-point rotate+translate entirely.
  EXPECT_TRUE(r.identity);
  double x = 12.0;
  double y = 34.0;
  double z = -5.0;
  apply_reprojection(r, x, y, z);
  EXPECT_NEAR(x, 12.0, 1e-9);
  EXPECT_NEAR(y, 34.0, 1e-9);
  EXPECT_NEAR(z, -5.0, 1e-9);
}

TEST(FrameReprojectionTest, RealOffsetIsNotCollapsedToIdentity)
{
  // A 5 mm inter-bag offset is real survey signal — it must survive the
  // identity detection.
  const auto earth_from_ref = makeTransform(0.0, 0.0, 0.0);
  const auto earth_from_src = makeTransform(0.005, 0.0, 0.0);
  const auto r = make_reprojection(earth_from_ref, earth_from_src);
  EXPECT_FALSE(r.identity);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  apply_reprojection(r, x, y, z);
  EXPECT_NEAR(x, 0.005, 1e-12);
}

TEST(FrameReprojectionTest, TranslatedSrcFrameMapsIntoRef)
{
  // Both frames axis-aligned with earth; src's origin sits 100 m east and
  // 50 m north of ref's. A point at src (1, 2, 3) is at ref (101, 52, 3).
  const auto earth_from_ref = makeTransform(0.0, 0.0, 0.0);
  const auto earth_from_src = makeTransform(100.0, 50.0, 0.0);
  const auto r = make_reprojection(earth_from_ref, earth_from_src);
  double x = 1.0;
  double y = 2.0;
  double z = 3.0;
  apply_reprojection(r, x, y, z);
  EXPECT_NEAR(x, 101.0, 1e-9);
  EXPECT_NEAR(y, 52.0, 1e-9);
  EXPECT_NEAR(z, 3.0, 1e-9);
}

TEST(FrameReprojectionTest, RotatedRefFrameCountersTheRotation)
{
  // ref is rotated 90° about +z relative to earth, src is axis-aligned. The
  // src point (1, 0, 0) sits at earth (1, 0, 0); expressed in ref that is
  // rotated the other way: (0, -1, 0).
  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  const auto earth_from_ref = makeTransform(0.0, 0.0, 0.0, 0.0, 0.0, s, c);
  const auto earth_from_src = makeTransform(0.0, 0.0, 0.0);
  const auto r = make_reprojection(earth_from_ref, earth_from_src);
  double x = 1.0;
  double y = 0.0;
  double z = 0.0;
  apply_reprojection(r, x, y, z);
  EXPECT_NEAR(x, 0.0, 1e-9);
  EXPECT_NEAR(y, -1.0, 1e-9);
  EXPECT_NEAR(z, 0.0, 1e-9);
}

// --- read_mbes_window error path -------------------------------------------

TEST(ReadMbesWindow, MissingBagThrows)
{
  EXPECT_THROW(
    marine_perception_tools::read_mbes_window("/nonexistent-dir/nope", 0, 1),
    std::exception);
}

}  // namespace

namespace
{

using marine_perception_tools::MbesWindowOptions;
using marine_perception_tools::MbesWindowResult;
using marine_perception_tools::PingProjection;
using marine_perception_tools::accumulate_ping;

// --- the projector's frames and its run totals (#55) ------------------------

// These three defaults were read out of a real BizzyBoat M3 recording, not
// taken from cube::ProjectorParams. The library's own default base_link_frame
// is the unprefixed "base_link", and these bags carry a
// `bizzy/base_link -> base_link -> base_link_frd` alias chain, so an
// unprefixed lookup would resolve without throwing and take the boat's
// attitude — and every sounding's uncertainty — from the wrong frame. A
// silent wrong answer is what this test exists to keep out.
TEST(MbesWindowOptionsFrames, DefaultToTheFramesVerifiedAgainstARealBag)
{
  const MbesWindowOptions options;
  EXPECT_EQ(options.base_link_frame, "bizzy/base_link");
  EXPECT_EQ(options.level_frame, "bizzy/base_link_north_up");
  EXPECT_EQ(options.tide_frame, "bizzy/map_tide");
  // All three namespaced, like the world frame that was already here.
  EXPECT_EQ(options.world_frame, "bizzy/map");
}

// One ping's projection with distinguishable counts in every field, so a
// field that is not accumulated shows up as a wrong total rather than as a
// number that happens to match its neighbour.
PingProjection samplePing(std::size_t kept, std::size_t filtered, std::size_t invalid)
{
  PingProjection p;
  p.soundings.resize(kept);
  p.diagnostics.total = kept + filtered + invalid;
  p.diagnostics.filtered_range = filtered;
  p.diagnostics.missing_attitude = 1;
  p.diagnostics.missing_heave = 1;
  p.diagnostics.default_beamwidth_beams = p.diagnostics.total;
  p.diagnostics.missing_rx_angle_beams = filtered;
  p.invalid_beams = invalid;
  return p;
}

// Every field of the run totals accumulates, across pings — the counts the
// lab's load note is built from.
TEST(MbesWindowDiagnostics, AccumulateEveryFieldAcrossPings)
{
  MbesWindowResult result;
  result.diagnostics.reports_georeferencing = false;
  accumulate_ping(result, samplePing(10, 2, 1));
  accumulate_ping(result, samplePing(6, 3, 2));

  EXPECT_EQ(result.diagnostics.pings, 2u);
  EXPECT_EQ(result.diagnostics.soundings, 16u);
  EXPECT_EQ(result.diagnostics.beams, 24u);
  EXPECT_EQ(result.diagnostics.filtered_range, 5u);
  EXPECT_EQ(result.diagnostics.missing_attitude, 2u);
  EXPECT_EQ(result.diagnostics.missing_heave, 2u);
  EXPECT_EQ(result.diagnostics.default_beamwidth_beams, 24u);
  EXPECT_EQ(result.diagnostics.missing_rx_angle_beams, 5u);
  EXPECT_EQ(result.invalid_beams, 3);
  EXPECT_EQ(result.invalid_pings, 0);
  // The arithmetic the note asks a reader to check.
  EXPECT_EQ(
    result.diagnostics.beams,
    result.diagnostics.soundings + result.diagnostics.filtered_range +
    static_cast<std::size_t>(result.invalid_beams));
}

// A ping refused for a non-positive sound speed contributes no beams and no
// soundings — it never reached the error model — but is counted, so the note
// can say that data went missing before the projection rather than during it.
TEST(MbesWindowDiagnostics, AnInvalidPingIsCountedAndContributesNoBeams)
{
  MbesWindowResult result;
  PingProjection bad;
  bad.invalid_pings = 1;
  accumulate_ping(result, bad);
  accumulate_ping(result, samplePing(4, 0, 0));

  EXPECT_EQ(result.invalid_pings, 1);
  EXPECT_EQ(result.diagnostics.pings, 2u);
  EXPECT_EQ(result.diagnostics.beams, 4u);
  EXPECT_EQ(result.diagnostics.soundings, 4u);
}

// This path does not georeference per sounding: its earth-anchor reprojection
// (has_geo / earth_from_world) relates one bag's world frame to another's,
// which is a different thing. Left true, report_projection_summary would print
// "0 georeferenced into the grid, 0 dropped (no earth TF)" and then warn about
// a localization chain that was never in question.
TEST(MbesWindowDiagnostics, DoNotClaimPerSoundingGeoreferencing)
{
  MbesWindowResult result;
  EXPECT_TRUE(cube::ProjectionRunTotals{}.reports_georeferencing)
    << "the library default is what makes this an explicit choice";
  result.diagnostics.reports_georeferencing = false;
  EXPECT_FALSE(result.diagnostics.reports_georeferencing);
}

}  // namespace
