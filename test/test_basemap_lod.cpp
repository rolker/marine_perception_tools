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

#include <QCoreApplication>
#include <QImage>

#include <limits>
#include <map>
#include <memory>
#include <vector>

#include "basemap_lod.hpp"

namespace
{

using marine_perception_tools::BasemapLod;
using marine_perception_tools::BasemapTileKey;
using marine_perception_tools::OverviewTile;
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

// --- Composite order (#43) -------------------------------------------------
//
// uma-ADR-0013 D5's ascending guarantee: a consumer composites every level at
// or below its selection with FINER OVER COARSER, always. The selection
// decides what to LOAD, never what to overlay. renderTiles() returns
// back-to-front paint order, so the level numbers it yields must be
// non-decreasing (lower numbers are coarser — level 0 is the apex) regardless
// of which level is selected.

// A synthetic resident tile carrying its level in a 1x1 image, so the paint
// order can be read back from renderTiles()'s output.
OverviewTile syntheticTile(int level, double south, double west)
{
  OverviewTile tile;
  tile.image = QImage(1, 1, QImage::Format_ARGB32);
  tile.image.setPixel(0, 0, qRgba(level, 0, 0, 255));
  tile.south = south;
  tile.west = west;
  tile.north = south + 1.0;
  tile.east = west + 1.0;
  return tile;
}

int levelOf(const OverviewTile & tile)
{
  return qRed(tile.image.pixel(0, 0));
}

std::vector<int> paintOrder(const std::vector<OverviewTile> & tiles)
{
  std::vector<int> levels;
  levels.reserve(tiles.size());
  for (const auto & tile : tiles) {
    levels.push_back(levelOf(tile));
  }
  return levels;
}

// A two-level synthetic layer: a coarse level 6 and a fine level 10, two
// tiles each.
std::map<int, std::map<BasemapTileKey, OverviewTile>> twoLevelLayer()
{
  std::map<int, std::map<BasemapTileKey, OverviewTile>> resident;
  resident[6][{0, 0}] = syntheticTile(6, 43.0, -71.0);
  resident[6][{0, 1}] = syntheticTile(6, 43.0, -70.0);
  resident[10][{0, 0}] = syntheticTile(10, 43.0, -71.0);
  resident[10][{0, 1}] = syntheticTile(10, 43.0, -70.0);
  return resident;
}

// BasemapLod is a QObject holding a QFutureWatcher; give the process an
// application object, as the other Qt-touching tests here do.
class CompositeOrder : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (QCoreApplication::instance() == nullptr) {
      static int argc = 1;
      static char arg0[] = "test_basemap_lod";
      static char * argv[] = {arg0, nullptr};
      app_ = std::make_unique<QCoreApplication>(argc, argv);
    }
  }

  std::unique_ptr<QCoreApplication> app_;
};

TEST_F(CompositeOrder, ZoomedInSelectionIsPaintedLastBecauseItIsFinest)
{
  // Zoomed in: the selection IS the finest resident level, so "selected last"
  // and "finest last" agree — the case the old rule got right.
  BasemapLod lod;
  lod.setResidentForTest(twoLevelLayer(), 10);
  const auto levels = paintOrder(lod.renderTiles());
  ASSERT_EQ(levels.size(), 4u);
  EXPECT_EQ(levels, (std::vector<int>{6, 6, 10, 10}));
}

TEST_F(CompositeOrder, ZoomedOutCoarseSelectionNeverPaintsOverFinerTiles)
{
  // The #43 regression. Zooming out, selectLodLevel() picks the coarser level
  // while the finer tiles are still resident. Painting the selection last put
  // level 6 on top of level 10 and hid detail the operator could see a moment
  // earlier; the finer level must still be painted last.
  BasemapLod lod;
  lod.setResidentForTest(twoLevelLayer(), 6);
  const auto levels = paintOrder(lod.renderTiles());
  ASSERT_EQ(levels.size(), 4u);
  EXPECT_EQ(levels, (std::vector<int>{6, 6, 10, 10}));
}

TEST_F(CompositeOrder, PaintOrderIsIndependentOfTheSelection)
{
  // The ordering rule is "finest last, full stop": over a three-level ladder
  // every selection — including none (-1) and a level that is not resident —
  // yields the same coarse-to-fine sequence.
  std::map<int, std::map<BasemapTileKey, OverviewTile>> resident;
  resident[3][{0, 0}] = syntheticTile(3, 43.0, -71.0);
  resident[8][{0, 0}] = syntheticTile(8, 43.0, -71.0);
  resident[12][{0, 0}] = syntheticTile(12, 43.0, -71.0);
  const std::vector<int> expected{3, 8, 12};
  for (const int selected : {-1, 3, 8, 12, 5}) {
    BasemapLod lod;
    lod.setResidentForTest(resident, selected);
    EXPECT_EQ(paintOrder(lod.renderTiles()), expected)
      << "selected_level=" << selected;
  }
}

TEST_F(CompositeOrder, EveryResidentTileIsPaintedExactlyOnce)
{
  // Nothing is dropped and nothing is doubled by the ordering — a coarse
  // selection used to be skipped in the backdrop pass and re-emitted after it.
  BasemapLod lod;
  lod.setResidentForTest(twoLevelLayer(), 6);
  EXPECT_EQ(lod.renderTiles().size(), 4u);
  lod.setResidentForTest({}, 6);
  EXPECT_TRUE(lod.renderTiles().empty());
}

}  // namespace
