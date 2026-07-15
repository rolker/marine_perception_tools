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
  double x = 12.0;
  double y = 34.0;
  double z = -5.0;
  apply_reprojection(r, x, y, z);
  EXPECT_NEAR(x, 12.0, 1e-9);
  EXPECT_NEAR(y, 34.0, 1e-9);
  EXPECT_NEAR(z, -5.0, 1e-9);
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
