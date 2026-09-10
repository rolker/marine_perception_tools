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

// Position -> time over the nav track (#46), and the rule that makes the hit
// radius mean the same thing at every zoom.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "nav_track_hit.hpp"

namespace
{

using marine_perception_tools::kFixHitRadiusPx;
using marine_perception_tools::kHitMetresPerDegLat;
using marine_perception_tools::nearestTrackFix;
using marine_survey_index::NavPoint;

constexpr std::int64_t kS = 1000000000LL;
constexpr double kLat = 43.0;

// Degrees of latitude for a given number of metres north.
double dLat(double metres)
{
  return metres / kHitMetresPerDegLat;
}

// Degrees of longitude for a given number of metres east, at kLat.
double dLon(double metres)
{
  return metres / (kHitMetresPerDegLat * std::cos(kLat * M_PI / 180.0));
}

// Two bags over the same water: bag 1 runs north from (43.0, -71.0) in 10 m
// steps, bag 2 is the same line 30 m east, an hour later — the revisited-area
// case the feature exists for.
std::vector<NavPoint> makeTrack()
{
  std::vector<NavPoint> track;
  const double east_30m = dLon(30.0);
  for (int i = 0; i < 5; ++i) {
    track.push_back({1, (3600 + i) * kS, kLat + dLat(10.0 * i), -71.0});
  }
  for (int i = 0; i < 5; ++i) {
    track.push_back({2, (7200 + i) * kS, kLat + dLat(10.0 * i), -71.0 + east_30m});
  }
  return track;
}

TEST(NavTrackHit, EmptyTrackHasNoHit)
{
  EXPECT_FALSE(nearestTrackFix({}, kLat, -71.0, 1.0).has_value());
}

TEST(NavTrackHit, FindsTheFixUnderTheCursorAndItsTime)
{
  // 1 m per pixel; sitting 2 m north of the third fix of bag 1.
  const auto hit = nearestTrackFix(makeTrack(), kLat + dLat(22.0), -71.0, 1.0);
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->index, 2u);
  EXPECT_EQ(hit->t_ns, 3602 * kS);
  EXPECT_EQ(hit->bag_id, 1);
  EXPECT_NEAR(hit->distance_px, 2.0, 1e-6);
}

// The whole reason the radius is in pixels: the SAME cursor offset in ground
// metres is a hit when the view is zoomed in and a miss when it is zoomed out.
TEST(NavTrackHit, TheRadiusIsScreenPixelsNotGroundMetres)
{
  const auto track = makeTrack();
  const double lon = -71.0 - dLon(5.0);   // 5 m west of the first fix
  // 1 m/px: 5 px away, inside the radius.
  const auto near_view = nearestTrackFix(track, kLat, lon, 1.0);
  ASSERT_TRUE(near_view.has_value());
  EXPECT_NEAR(near_view->distance_px, 5.0, 1e-6);
  // 0.1 m/px (zoomed in): the same ground offset is 50 px — out of reach.
  EXPECT_FALSE(nearestTrackFix(track, kLat, lon, 0.1).has_value());
  // 10 m/px (zoomed out): half a pixel, and still the same fix.
  const auto far_view = nearestTrackFix(track, kLat, lon, 10.0);
  ASSERT_TRUE(far_view.has_value());
  EXPECT_EQ(far_view->index, near_view->index);
  EXPECT_NEAR(far_view->distance_px, 0.5, 1e-6);
}

TEST(NavTrackHit, BeyondTheRadiusThereIsNoHit)
{
  const auto track = makeTrack();
  // 200 m west of the line at 1 m/px is nowhere near it.
  const double west = -71.0 - dLon(200.0);
  EXPECT_FALSE(nearestTrackFix(track, kLat, west, 1.0).has_value());
}

// Inclusive at the boundary, matching the click-versus-drag slop: the pixel
// exactly at the limit is the one the operator aimed at.
TEST(NavTrackHit, TheBoundaryIsInclusive)
{
  const auto track = makeTrack();
  const double at = -71.0 - dLon(kFixHitRadiusPx);   // radius px at 1 m/px
  EXPECT_TRUE(nearestTrackFix(track, kLat, at, 1.0).has_value());
  const double past = -71.0 - dLon(kFixHitRadiusPx + 0.001);
  EXPECT_FALSE(nearestTrackFix(track, kLat, past, 1.0).has_value());
}

// The nearest fix wins even when several are in range — this is what keeps
// the marker on the pass the cursor is actually over in a revisited area.
TEST(NavTrackHit, NearestWinsAmongSeveralInRange)
{
  const auto track = makeTrack();
  // Between bag 1's line and bag 2's, but 10 m from the second: at 5 m/px
  // both lines are within the radius and the eastern one is nearer.
  const auto hit = nearestTrackFix(track, kLat, -71.0 + dLon(20.0), 5.0);
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->bag_id, 2) << "picked the farther pass";
  EXPECT_EQ(hit->index, 5u);
}

// A cursor exactly between two fixes must resolve the same way on every
// frame, or the marker flickers as the hand holds still. Earlier fix wins.
TEST(NavTrackHit, TiesGoToTheEarlierFix)
{
  const std::vector<NavPoint> track = {
    {1, 100 * kS, kLat, -71.0 - dLon(3.0)},
    {1, 200 * kS, kLat, -71.0 + dLon(3.0)},
  };
  const auto hit = nearestTrackFix(track, kLat, -71.0, 1.0);
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->index, 0u);
  EXPECT_EQ(hit->t_ns, 100 * kS);
}

// An unusable view scale means the pixel radius has no meaning, so there is
// no hit rather than a hit computed from a garbage number. The canvas reports
// 1/px_per_m, which is zero or infinite in exactly the states (no layout, no
// geo origin) where a hover is meaningless anyway.
TEST(NavTrackHit, AnUnusableViewScaleHasNoHit)
{
  const auto track = makeTrack();
  EXPECT_FALSE(nearestTrackFix(track, kLat, -71.0, 0.0).has_value());
  EXPECT_FALSE(nearestTrackFix(track, kLat, -71.0, -1.0).has_value());
  EXPECT_FALSE(
    nearestTrackFix(track, kLat, -71.0, std::numeric_limits<double>::infinity())
    .has_value());
  EXPECT_FALSE(
    nearestTrackFix(track, std::numeric_limits<double>::quiet_NaN(), -71.0, 1.0)
    .has_value());
}

// A zero radius is degenerate but well defined: only a fix exactly under the
// cursor hits. Nothing in the app passes one; the rule must not be undefined.
TEST(NavTrackHit, AZeroRadiusOnlyHitsAnExactCoincidence)
{
  const auto track = makeTrack();
  EXPECT_FALSE(nearestTrackFix(track, kLat, -71.0 - dLon(0.5), 1.0, 0.0).has_value());
  const auto exact = nearestTrackFix(track, kLat, -71.0, 1.0, 0.0);
  ASSERT_TRUE(exact.has_value());
  EXPECT_EQ(exact->index, 0u);
}

}  // namespace

// A non-finite fix must be skipped, not selected (#42 review, independently
// flagged by Copilot). Its pixel distance is NaN, which the radius gate does
// NOT reject, and once it were `best` no real candidate could beat it — so one
// bad row would swallow the whole track and return itself with a NaN distance.
TEST(NavTrackHit, ANonFiniteRowIsSkippedAndDoesNotSwallowTheTrack)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<marine_survey_index::NavPoint> track;

  marine_survey_index::NavPoint bad;
  bad.t_ns = 1;
  bad.latitude = nan;
  bad.longitude = nan;
  track.push_back(bad);   // first, so it would become `best` before any good row

  marine_survey_index::NavPoint good;
  good.t_ns = 2;
  good.latitude = 43.0;
  good.longitude = -71.0;
  track.push_back(good);

  const auto hit = nearestTrackFix(track, 43.0, -71.0, 1.0);

  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(1u, hit->index) << "the non-finite row was returned";
  EXPECT_EQ(2, hit->t_ns);
  EXPECT_TRUE(std::isfinite(hit->distance_px));
}

// A track of nothing BUT non-finite rows is a miss, not a bogus hit.
TEST(NavTrackHit, AnAllNonFiniteTrackReturnsNoHit)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<marine_survey_index::NavPoint> track(3);
  for (auto & p : track) {
    p.latitude = nan;
    p.longitude = nan;
  }

  EXPECT_FALSE(nearestTrackFix(track, 43.0, -71.0, 1.0).has_value());
}
