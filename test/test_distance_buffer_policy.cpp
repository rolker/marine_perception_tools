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

#include "distance_buffer_policy.hpp"

using marine_perception_tools::distance_window;
using marine_perception_tools::stationary_keep_from;

constexpr double kEps = 1e-9;

TEST(DistanceBufferPolicy, WindowTrailsHeadByLength)
{
  auto w = distance_window(250.0, 100.0, 676.0);
  EXPECT_NEAR(w.hi, 250.0, kEps);
  EXPECT_NEAR(w.lo, 150.0, kEps);
}

TEST(DistanceBufferPolicy, WindowClampsLowAtZero)
{
  auto w = distance_window(40.0, 100.0, 676.0);  // head < window length
  EXPECT_NEAR(w.hi, 40.0, kEps);
  EXPECT_NEAR(w.lo, 0.0, kEps);
}

TEST(DistanceBufferPolicy, HeadClampedToTotal)
{
  auto w = distance_window(900.0, 100.0, 676.0);  // past the end
  EXPECT_NEAR(w.hi, 676.0, kEps);
  EXPECT_NEAR(w.lo, 576.0, kEps);
}

TEST(DistanceBufferPolicy, NonPositiveWindowCollapses)
{
  auto w = distance_window(250.0, 0.0, 676.0);
  EXPECT_NEAR(w.lo, 250.0, kEps);
  EXPECT_NEAR(w.hi, 250.0, kEps);
}

TEST(DistanceBufferPolicy, NegativeHeadAndTotalGuarded)
{
  auto w = distance_window(-5.0, 100.0, -10.0);
  EXPECT_NEAR(w.hi, 0.0, kEps);
  EXPECT_NEAR(w.lo, 0.0, kEps);
}

TEST(DistanceBufferPolicy, StationaryCapKeepsMostRecent)
{
  EXPECT_EQ(stationary_keep_from(10, 4), 6);   // drop the 6 oldest, keep last 4
  EXPECT_EQ(stationary_keep_from(3, 4), 0);    // fewer than cap -> keep all
  EXPECT_EQ(stationary_keep_from(10, 0), 0);   // cap disabled
  EXPECT_EQ(stationary_keep_from(10, -1), 0);  // cap disabled
  EXPECT_EQ(stationary_keep_from(4, 4), 0);    // exactly at cap -> keep all
}
