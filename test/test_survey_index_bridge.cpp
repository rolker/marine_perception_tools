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
    path_ = std::string(::testing::TempDir()) + "bridge_fixture.db";
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
    sqlite3_close(db);
  }

  void TearDown() override
  {
    std::remove(path_.c_str());
  }

  static void exec(sqlite3 * db, const char * sql)
  {
    char * err = nullptr;
    ASSERT_EQ(sqlite3_exec(db, sql, nullptr, nullptr, &err), SQLITE_OK)
      << (err ? err : "unknown sqlite error");
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

TEST(SurveyIndexBridge, MissingDbThrows)
{
  // openIndexDb creates a fresh DB on a writable path, so point at a path
  // whose parent directory does not exist.
  EXPECT_THROW(
    marine_perception_tools::SurveyIndexBridge("/nonexistent-dir/idx.db"),
    std::runtime_error);
}

}  // namespace
