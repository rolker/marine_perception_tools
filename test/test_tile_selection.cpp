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

#include <set>
#include <vector>

#include "tile_selection.hpp"

namespace
{

using marine_perception_tools::SelectableRect;
using marine_perception_tools::hitRect;
using marine_perception_tools::rectsInBox;
using marine_perception_tools::toggleSelection;

// A 2x2 grid of unit tiles: [0]=(0,0)-(1,1), [1]=(1,0)-(2,1),
// [2]=(0,1)-(1,2), [3]=(1,1)-(2,2).
std::vector<SelectableRect> grid()
{
  return {
    {0.0, 0.0, 1.0, 1.0},
    {1.0, 0.0, 2.0, 1.0},
    {0.0, 1.0, 1.0, 2.0},
    {1.0, 1.0, 2.0, 2.0}};
}

TEST(HitRect, FindsContainingTileAndMissesOutside)
{
  const auto rects = grid();
  EXPECT_EQ(hitRect(rects, 0.5, 0.5), 0);
  EXPECT_EQ(hitRect(rects, 1.5, 0.5), 1);
  EXPECT_EQ(hitRect(rects, 0.5, 1.5), 2);
  EXPECT_EQ(hitRect(rects, 2.5, 0.5), -1);
  EXPECT_EQ(hitRect(rects, -0.1, 0.5), -1);
}

TEST(HitRect, EdgesAreInclusiveAndFirstMatchWins)
{
  const auto rects = grid();
  // (1, 0.5) lies on the shared edge of tiles 0 and 1: first match wins.
  EXPECT_EQ(hitRect(rects, 1.0, 0.5), 0);
  EXPECT_EQ(hitRect(rects, 0.0, 0.0), 0);   // outer corner
}

TEST(RectsInBox, IntersectionNotContainment)
{
  const auto rects = grid();
  // A band covering the middle of the grid clips all four tiles.
  const auto hits = rectsInBox(rects, 0.5, 0.5, 1.5, 1.5);
  EXPECT_EQ(hits.size(), 4u);
}

TEST(RectsInBox, CornerOrderDoesNotMatter)
{
  const auto rects = grid();
  const auto a = rectsInBox(rects, 0.2, 0.2, 0.8, 0.8);
  const auto b = rectsInBox(rects, 0.8, 0.8, 0.2, 0.2);
  ASSERT_EQ(a.size(), 1u);
  EXPECT_EQ(a, b);
  EXPECT_EQ(a[0], 0u);
}

TEST(RectsInBox, TouchingEdgeCounts)
{
  const auto rects = grid();
  // Band exactly up to x=1: tiles starting at x=1 still count (a band dragged
  // to a boundary takes the tile under it).
  const auto hits = rectsInBox(rects, 0.5, 0.5, 1.0, 0.6);
  ASSERT_EQ(hits.size(), 2u);
  EXPECT_EQ(hits[0], 0u);
  EXPECT_EQ(hits[1], 1u);
}

TEST(RectsInBox, MissReturnsEmpty)
{
  EXPECT_TRUE(rectsInBox(grid(), 5.0, 5.0, 6.0, 6.0).empty());
}

TEST(ToggleSelection, TogglesMembership)
{
  std::set<std::size_t> sel;
  EXPECT_TRUE(toggleSelection(sel, 2));
  EXPECT_EQ(sel.count(2), 1u);
  EXPECT_FALSE(toggleSelection(sel, 2));
  EXPECT_TRUE(sel.empty());
}

}  // namespace
