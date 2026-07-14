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

}  // namespace
