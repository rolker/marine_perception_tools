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

#include <sqlite3.h>

#include <algorithm>
#include <cstdio>
#include <string>

#include "marine_autonomy/gggs.h"
#include "marine_survey_index/schema.hpp"
#include "survey_index_bridge.hpp"

namespace
{

constexpr double kLat = 43.02;
constexpr double kLon = -71.36;

// A file-backed fixture DB (the bridge opens by path, so :memory: can't be
// shared with it), populated through the same schema the indexer writes —
// one bag with a pass on the L14 tile containing (kLat, kLon).
class BridgeFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path_ = std::string(::testing::TempDir()) + "/bridge_fixture.db";
    std::remove(path_.c_str());
    sqlite3 * db = marine_survey_index::openIndexDb(path_);
    exec(db, "INSERT INTO bags (id, path, size_bytes, mtime_ns, indexed_at_ns)"
      " VALUES (1, '/data/bag_a', 100, 200, 300);");
    const auto tile = gggs::Level(14).gridIndex(kLat, kLon);
    const std::string pass =
      "INSERT INTO passes (bag_id, level, tile_row, tile_col, sensor_type,"
      " topic, t_start_ns, t_end_ns, ping_count) VALUES (1, 14, " +
      std::to_string(tile.row()) + ", " + std::to_string(tile.column()) +
      ", 'mbes-bathy', '/detections', 1000, 2000, 42);";
    exec(db, pass.c_str());
    // A second pass on a tile ~1 km north (same bag): far enough that the
    // stage-2 point-query tests keep their single-tile semantics, and the
    // tile-selection queries must find exactly the selected tiles' passes.
    const auto north_tile = gggs::Level(14).gridIndex(kLat + 0.01, kLon);
    const std::string pass2 =
      "INSERT INTO passes (bag_id, level, tile_row, tile_col, sensor_type,"
      " topic, t_start_ns, t_end_ns, ping_count) VALUES (1, 14, " +
      std::to_string(north_tile.row()) + ", " + std::to_string(north_tile.column()) +
      ", 'sidescan-port', '/ss_port', 5000, 6000, 7);";
    exec(db, pass2.c_str());
    // Nav track (schema v2, #265) — inserted out of time order to prove the
    // accessor's ordering.
    exec(db, ("INSERT INTO nav_track (bag_id, t_ns, latitude, longitude) VALUES"
      " (1, 1500, " + std::to_string(kLat) + ", " + std::to_string(kLon) + "),"
      " (1, 1200, " + std::to_string(kLat + 1e-4) + ", " + std::to_string(kLon) + ");")
      .c_str());
    sqlite3_close(db);
  }

  void TearDown() override
  {
    std::remove(path_.c_str());
  }

  static void exec(sqlite3 * db, const char * sql)
  {
    char * err = nullptr;
    const int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
    const std::string msg = err ? err : "unknown sqlite error";
    sqlite3_free(err);
    ASSERT_EQ(rc, SQLITE_OK) << msg;
  }

  std::string path_;
};

TEST_F(BridgeFixture, ClickOnIndexedSpotFindsThePass)
{
  const marine_perception_tools::SurveyIndexBridge bridge(path_);
  const auto rows = bridge.queryPoint(kLat, kLon);
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0].bag_path, "/data/bag_a");
  EXPECT_EQ(rows[0].sensor_type, "mbes-bathy");
  EXPECT_EQ(rows[0].t_start_ns, 1000);
  EXPECT_EQ(rows[0].t_end_ns, 2000);
  EXPECT_EQ(rows[0].ping_count, 42);
}

TEST_F(BridgeFixture, ClickFarAwayFindsNothing)
{
  const marine_perception_tools::SurveyIndexBridge bridge(path_);
  // Portsmouth Harbor is many tiles from the Massabesic fixture pass.
  EXPECT_TRUE(bridge.queryPoint(43.07, -70.71).empty());
}

TEST_F(BridgeFixture, RadiusReachesIntoNeighbouringTile)
{
  // Query from the far side of the adjacent tile with a radius that spans
  // back across the boundary: an L14 tile is ~54 m, so 120 m reaches it.
  const marine_perception_tools::SurveyIndexBridge bridge(path_);
  const auto tile = gggs::Level(14).gridIndex(kLat, kLon);
  const double lat_outside = tile.northLatitude() + 1e-5;
  EXPECT_FALSE(bridge.queryPoint(lat_outside, kLon, 120.0).empty());
  EXPECT_TRUE(bridge.queryPoint(lat_outside, kLon, 0.1).empty());
}

TEST_F(BridgeFixture, ExtentMatchesTheIndexedTileBounds)
{
  // The bridge derives tile bounds from the public gggs::levels specs (the
  // GridIndex row/col constructor is private); pin those formulas against a
  // real GridIndex built from coordinates so they can't drift.
  const marine_perception_tools::SurveyIndexBridge bridge(path_);
  const auto box = bridge.extent();
  ASSERT_TRUE(box.has_value());
  const auto home = gggs::Level(14).gridIndex(kLat, kLon);
  const auto far_north = gggs::Level(14).gridIndex(kLat + 0.01, kLon);
  EXPECT_DOUBLE_EQ(box->south, home.southLatitude());
  EXPECT_DOUBLE_EQ(box->north, far_north.northLatitude());
  EXPECT_DOUBLE_EQ(box->west, std::min(home.westLongitude(), far_north.westLongitude()));
  EXPECT_DOUBLE_EQ(box->east, std::max(home.eastLongitude(), far_north.eastLongitude()));
}

TEST_F(BridgeFixture, IndexedTileBoundsMatchGridIndex)
{
  // The selectable-tile bounds come from the same public gggs::levels
  // formulas as extent(); pin them against real GridIndex accessors.
  const marine_perception_tools::SurveyIndexBridge bridge(path_);
  const auto tiles = bridge.indexedTiles();
  ASSERT_EQ(tiles.size(), 2u);
  const auto home = gggs::Level(14).gridIndex(kLat, kLon);
  const auto & first = tiles[0];   // ordered by (level, row, col): home is south
  EXPECT_EQ(first.level, 14);
  EXPECT_EQ(first.row, home.row());
  EXPECT_EQ(first.col, home.column());
  EXPECT_DOUBLE_EQ(first.south, home.southLatitude());
  EXPECT_DOUBLE_EQ(first.north, home.northLatitude());
  EXPECT_DOUBLE_EQ(first.west, home.westLongitude());
  EXPECT_DOUBLE_EQ(first.east, home.eastLongitude());
}

TEST_F(BridgeFixture, OutOfContractTileRowsAreSkipped)
{
  // A corrupt (but readable) index can hold negative or oversized row/col or
  // an unknown level; those rows must be skipped like the level guard
  // already does — a raw uint32 cast would wrap a negative row to ~4e9 (or
  // an over-uint32 row to a small one) and fabricate a selectable tile with
  // nonsense bounds.
  sqlite3 * db = marine_survey_index::openIndexDb(path_);
  exec(db, "INSERT INTO passes (bag_id, level, tile_row, tile_col, sensor_type,"
    " topic, t_start_ns, t_end_ns, ping_count)"
    " VALUES (1, 14, -3, 100, 'mbes-bathy', '/detections', 1, 2, 1),"
    " (1, 14, 100, -3, 'mbes-bathy', '/detections', 1, 2, 1),"
    " (1, 14, 5000000000, 100, 'mbes-bathy', '/detections', 1, 2, 1),"
    " (1, 14, 100, 5000000000, 'mbes-bathy', '/detections', 1, 2, 1),"
    " (1, 99, 100, 100, 'mbes-bathy', '/detections', 1, 2, 1);");
  sqlite3_close(db);
  const marine_perception_tools::SurveyIndexBridge bridge(path_);
  EXPECT_EQ(bridge.indexedTiles().size(), 2u);   // only the two fixture tiles
}

TEST_F(BridgeFixture, QueryTilesReturnsExactlyTheSelectedTilesPasses)
{
  const marine_perception_tools::SurveyIndexBridge bridge(path_);
  const auto tiles = bridge.indexedTiles();
  ASSERT_EQ(tiles.size(), 2u);

  // Selecting only the home (southern) tile finds only its mbes pass.
  const auto home_rows = bridge.queryTiles({tiles[0]});
  ASSERT_EQ(home_rows.size(), 1u);
  EXPECT_EQ(home_rows[0].sensor_type, "mbes-bathy");

  // Selecting both tiles finds both passes.
  EXPECT_EQ(bridge.queryTiles(tiles).size(), 2u);

  // An empty selection finds nothing.
  EXPECT_TRUE(bridge.queryTiles({}).empty());
}

TEST_F(BridgeFixture, NavTrackIsTimeOrdered)
{
  const marine_perception_tools::SurveyIndexBridge bridge(path_);
  const auto track = bridge.navTrack();
  ASSERT_EQ(track.size(), 2u);
  EXPECT_EQ(track[0].t_ns, 1200);
  EXPECT_EQ(track[1].t_ns, 1500);
  EXPECT_EQ(track[0].bag_id, track[1].bag_id);
}

TEST(SurveyIndexBridge, NavTrackOfEmptyIndexIsEmpty)
{
  const std::string path = std::string(::testing::TempDir()) + "/empty_track.db";
  std::remove(path.c_str());
  sqlite3_close(marine_survey_index::openIndexDb(path));
  const marine_perception_tools::SurveyIndexBridge bridge(path);
  EXPECT_TRUE(bridge.navTrack().empty());
  EXPECT_TRUE(bridge.indexedTiles().empty());
  std::remove(path.c_str());
}

TEST(SurveyIndexBridge, ExtentOfEmptyIndexIsNullopt)
{
  const std::string path = std::string(::testing::TempDir()) + "/empty_index.db";
  std::remove(path.c_str());
  sqlite3_close(marine_survey_index::openIndexDb(path));
  const marine_perception_tools::SurveyIndexBridge bridge(path);
  EXPECT_FALSE(bridge.extent().has_value());
  std::remove(path.c_str());
}

TEST(SurveyIndexBridge, MissingDbThrows)
{
  // openIndexDb creates a fresh DB on a writable path, so point at a path
  // whose parent directory does not exist.
  EXPECT_THROW(
    marine_perception_tools::SurveyIndexBridge("/nonexistent-dir/idx.db"),
    std::runtime_error);
}

}  // namespace
