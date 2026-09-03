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

#include <cstdint>

#include "sidescan_bag_session.hpp"

namespace
{

using marine_perception_tools::SessionIndex;
using marine_perception_tools::SidescanPing;
using marine_perception_tools::MbesPing;
using marine_perception_tools::distance_interval;

constexpr int64_t kSec = 1000000000LL;

// A synthetic stamp-sorted index: one ping per second, one metre per second,
// starting at t=100 s / d=0 m. The time→distance mapping under test.
SessionIndex makeIndex(int ping_count, bool posed = true)
{
  SessionIndex index;
  for (int i = 0; i < ping_count; ++i) {
    SidescanPing ping;
    ping.stamp_ns = (100 + i) * kSec;
    ping.cumulative_distance_m = static_cast<double>(i);
    ping.has_pose = posed;
    index.pings.push_back(ping);
  }
  index.total_distance_m = static_cast<double>(ping_count - 1);
  return index;
}

TEST(DistanceInterval, WindowInsideTrackMapsToDistances)
{
  const auto index = makeIndex(60);
  const auto interval = distance_interval(index, 110 * kSec, 120 * kSec);
  ASSERT_TRUE(interval.has_value());
  EXPECT_DOUBLE_EQ(interval->first, 10.0);
  EXPECT_DOUBLE_EQ(interval->second, 20.0);
}

TEST(DistanceInterval, WindowOverhangingEndsClampsToTrack)
{
  const auto index = makeIndex(60);
  // Starts before the first ping and ends after the last: covers everything.
  const auto interval = distance_interval(index, 1 * kSec, 1000 * kSec);
  ASSERT_TRUE(interval.has_value());
  EXPECT_DOUBLE_EQ(interval->first, 0.0);
  EXPECT_DOUBLE_EQ(interval->second, 59.0);
}

TEST(DistanceInterval, SwappedBoundsNormalize)
{
  const auto index = makeIndex(60);
  const auto interval = distance_interval(index, 120 * kSec, 110 * kSec);
  ASSERT_TRUE(interval.has_value());
  EXPECT_DOUBLE_EQ(interval->first, 10.0);
  EXPECT_DOUBLE_EQ(interval->second, 20.0);
}

TEST(DistanceInterval, WindowOutsideTrackYieldsNullopt)
{
  const auto index = makeIndex(60);
  EXPECT_FALSE(distance_interval(index, 1 * kSec, 50 * kSec).has_value());
  EXPECT_FALSE(distance_interval(index, 1000 * kSec, 2000 * kSec).has_value());
}

TEST(DistanceInterval, UnposedPingsDoNotParticipate)
{
  // Same window as the in-range case, but nothing has a pose: no cue.
  const auto index = makeIndex(60, false);
  EXPECT_FALSE(distance_interval(index, 110 * kSec, 120 * kSec).has_value());
}

TEST(DistanceInterval, MbesOnlyBagStillCues)
{
  // Down-look-only recordings carry no sidescan pings; the MBES pings share
  // the scrub axis and must drive the interval alone.
  SessionIndex index;
  for (int i = 0; i < 30; ++i) {
    MbesPing ping;
    ping.stamp_ns = (200 + i) * kSec;
    ping.cumulative_distance_m = static_cast<double>(2 * i);
    ping.tf_ok = true;
    ping.has_pose = true;
    index.mbes_pings.push_back(ping);
  }
  const auto interval = distance_interval(index, 205 * kSec, 210 * kSec);
  ASSERT_TRUE(interval.has_value());
  EXPECT_DOUBLE_EQ(interval->first, 10.0);
  EXPECT_DOUBLE_EQ(interval->second, 20.0);
}

TEST(DistanceInterval, SingleInstantWindowPicksOnePing)
{
  const auto index = makeIndex(60);
  const auto interval = distance_interval(index, 130 * kSec, 130 * kSec);
  ASSERT_TRUE(interval.has_value());
  EXPECT_DOUBLE_EQ(interval->first, 30.0);
  EXPECT_DOUBLE_EQ(interval->second, 30.0);
}

// Dense synthetic track for the hover-rate lookup: pings every 0.1 m along
// x, so decimation to the 1 m stride is observable.
SessionIndex makeDenseIndex(int ping_count, bool posed = true)
{
  SessionIndex index;
  for (int i = 0; i < ping_count; ++i) {
    SidescanPing ping;
    ping.stamp_ns = (100 + i) * kSec / 10;
    ping.cumulative_distance_m = 0.1 * static_cast<double>(i);
    ping.geometry.sensor_x = ping.cumulative_distance_m;
    ping.geometry.sensor_y = 0.0;
    ping.has_pose = posed;
    index.pings.push_back(ping);
  }
  return index;
}

TEST(TrackLookup, DecimatesToTheStrideAndKeepsTheTrackEnd)
{
  auto index = makeDenseIndex(101);   // 10 m of track at 0.1 m spacing
  marine_perception_tools::buildTrackLookup(index);
  ASSERT_FALSE(index.track_lookup.empty());
  // Samples are >= 1 m apart: ~11 of the 101 pings survive.
  EXPECT_LE(index.track_lookup.size(), 12u);
  for (std::size_t i = 1; i < index.track_lookup.size(); ++i) {
    EXPECT_GE(
      index.track_lookup[i].cum_dist_m - index.track_lookup[i - 1].cum_dist_m,
      marine_perception_tools::kTrackLookupStrideM - 1e-9);
  }
  EXPECT_DOUBLE_EQ(index.track_lookup.front().cum_dist_m, 0.0);
  EXPECT_DOUBLE_EQ(index.track_lookup.back().cum_dist_m, 10.0);
  EXPECT_DOUBLE_EQ(index.track_lookup.back().x, 10.0);
}

TEST(TrackLookup, ExactEndSurvivesEvenMidStride)
{
  // 10.55 m of track: the last posed ping (10.5) is only half a stride past
  // the last regular sample (10.0) but must still be present.
  auto index = makeDenseIndex(106);
  marine_perception_tools::buildTrackLookup(index);
  ASSERT_FALSE(index.track_lookup.empty());
  EXPECT_DOUBLE_EQ(index.track_lookup.back().cum_dist_m, 10.5);
}

TEST(TrackLookup, UnposedPingsYieldNoLookup)
{
  auto index = makeDenseIndex(50, false);
  marine_perception_tools::buildTrackLookup(index);
  EXPECT_TRUE(index.track_lookup.empty());
}

TEST(TrackLookup, RebuildClearsThePreviousLookup)
{
  auto index = makeDenseIndex(101);
  marine_perception_tools::buildTrackLookup(index);
  const auto first_size = index.track_lookup.size();
  marine_perception_tools::buildTrackLookup(index);   // idempotent, no doubling
  EXPECT_EQ(index.track_lookup.size(), first_size);
}

}  // namespace
