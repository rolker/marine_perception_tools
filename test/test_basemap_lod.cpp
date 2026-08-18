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

#include <limits>
#include <vector>

#include "basemap_lod.hpp"

namespace
{

using marine_perception_tools::selectLodLevel;

// Behavioural invariants of the camp ADR-0013 selection, without pinning
// gggs cell sizes: sentinel, single-level ladders, coarser-side preference,
// and monotonicity in metres-per-pixel.

TEST(SelectLodLevel, EmptyLadderIsSentinel)
{
  EXPECT_EQ(selectLodLevel(1.0, {}), -1);
}

TEST(SelectLodLevel, SingleLevelLadderAlwaysWins)
{
  for (const double mpp : {1e-6, 0.1, 1.0, 100.0, 1e6}) {
    EXPECT_EQ(selectLodLevel(mpp, {10}), 10);
  }
}

TEST(SelectLodLevel, HugePixelsPickTheCoarsestOfAFineLadder)
{
  // At 1e6 m/px the ideal is far coarser than any real store level: every
  // available level is finer than ideal, so the coarsest available wins.
  EXPECT_EQ(selectLodLevel(1e6, {12, 13}), 12);
  EXPECT_EQ(selectLodLevel(1e6, {0, 6, 10, 13}), 0);
}

TEST(SelectLodLevel, TinyPixelsPickTheFinestAvailable)
{
  // At 1e-6 m/px the ideal clamps to the finest GGGS level; the finest
  // coarser-or-equal available level is the ladder's finest.
  EXPECT_EQ(selectLodLevel(1e-6, {0, 5}), 5);
  EXPECT_EQ(selectLodLevel(1e-6, {0, 6, 10, 13}), 13);
}

TEST(SelectLodLevel, SelectionIsMonotonicInResolution)
{
  // Finer screen resolution (smaller m/px) never selects a coarser level.
  const std::vector<int> ladder = {0, 3, 6, 8, 10, 13};
  int previous = selectLodLevel(1e6, ladder);
  for (double mpp = 1e5; mpp >= 1e-4; mpp /= 10.0) {
    const int level = selectLodLevel(mpp, ladder);
    EXPECT_GE(level, previous) << "mpp=" << mpp;
    previous = level;
  }
}

TEST(SelectLodLevel, DegenerateInputsFallToTheGuard)
{
  // Non-positive and NaN metres-per-pixel behave like the finest request
  // rather than invoking UB.
  EXPECT_EQ(selectLodLevel(0.0, {0, 5}), 5);
  EXPECT_EQ(selectLodLevel(-1.0, {0, 5}), 5);
  EXPECT_EQ(
    selectLodLevel(std::numeric_limits<double>::quiet_NaN(), {0, 5}), 5);
}

}  // namespace
