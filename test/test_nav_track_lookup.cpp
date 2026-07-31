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

#include "nav_track_lookup.hpp"

namespace
{

using marine_perception_tools::fixAtTime;
using marine_survey_index::NavPoint;

constexpr std::int64_t kS = 1000000000LL;

// Two bags: bag 1 heads due east 10:00-10:10, bag 2 due north 11:00-11:10.
// The 50-minute hole between them is a gap, not data.
std::vector<NavPoint> makeTrack()
{
  return {
    {1, 36000 * kS, 43.00, -71.00},
    {1, 36300 * kS, 43.00, -70.99},
    {1, 36600 * kS, 43.00, -70.98},
    {2, 39600 * kS, 43.10, -71.10},
    {2, 40200 * kS, 43.12, -71.10},
  };
}

TEST(NavTrackLookup, InterpolatesWithinABag)
{
  const auto track = makeTrack();
  const auto fix = fixAtTime(track, 36150 * kS);   // halfway through leg 1
  ASSERT_TRUE(fix.has_value());
  EXPECT_EQ(fix->bag_id, 1);
  EXPECT_NEAR(fix->lat, 43.00, 1e-9);
  EXPECT_NEAR(fix->lon, -70.995, 1e-9);
  // Due east: atan2(east, north) = +pi/2.
  EXPECT_NEAR(fix->heading_rad, M_PI / 2.0, 1e-6);
}

TEST(NavTrackLookup, NorthboundHeading)
{
  const auto fix = fixAtTime(makeTrack(), 39900 * kS);
  ASSERT_TRUE(fix.has_value());
  EXPECT_EQ(fix->bag_id, 2);
  EXPECT_NEAR(fix->lat, 43.11, 1e-9);
  EXPECT_NEAR(fix->heading_rad, 0.0, 1e-6);   // due north
}

TEST(NavTrackLookup, GapBetweenBagsHasNoFix)
{
  EXPECT_FALSE(fixAtTime(makeTrack(), 38000 * kS).has_value());
}

TEST(NavTrackLookup, OutsideTheCampaignHasNoFix)
{
  EXPECT_FALSE(fixAtTime(makeTrack(), 1000 * kS).has_value());
  EXPECT_FALSE(fixAtTime(makeTrack(), 50000 * kS).has_value());
}

TEST(NavTrackLookup, ExactEndpointsResolve)
{
  const auto track = makeTrack();
  const auto first = fixAtTime(track, 36000 * kS);
  ASSERT_TRUE(first.has_value());
  EXPECT_NEAR(first->lon, -71.00, 1e-9);
  const auto last = fixAtTime(track, 40200 * kS);
  ASSERT_TRUE(last.has_value());
  EXPECT_EQ(last->bag_id, 2);
  EXPECT_NEAR(last->lat, 43.12, 1e-9);
}

TEST(NavTrackLookup, EmptyTrackHasNoFix)
{
  EXPECT_FALSE(fixAtTime({}, 36000 * kS).has_value());
}

}  // namespace
