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

#include "survey_index_bridge.hpp"

#include <sqlite3.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "marine_autonomy/gggs.h"
#include "marine_survey_index/footprint.hpp"
#include "marine_survey_index/schema.hpp"

namespace marine_perception_tools
{

SurveyIndexBridge::SurveyIndexBridge(const std::string & db_path)
: db_(marine_survey_index::openIndexDb(db_path))
{
}

SurveyIndexBridge::~SurveyIndexBridge()
{
  if (db_ != nullptr) {
    sqlite3_close(db_);
  }
}

std::vector<marine_survey_index::PassRow> SurveyIndexBridge::queryPoint(
  double lat, double lon, double radius_m) const
{
  // The same point→box expansion the query CLI uses: metres → degrees about
  // the click latitude (1° lat ≈ 111.32 km; longitude shrinks by cos(lat)).
  constexpr double kMetersPerDegLat = 111320.0;
  const double dlat = radius_m / kMetersPerDegLat;
  const double cos_lat = std::max(0.01, std::cos(lat * M_PI / 180.0));
  const double dlon = dlat / cos_lat;

  // Translate the box into tile keys at every level the index holds, so a DB
  // indexed at a non-default level (or mixed levels) still answers.
  std::vector<gggs::GridIndex> tiles;
  for (const auto level_n : marine_survey_index::distinctLevels(db_, "")) {
    const auto level_tiles = marine_survey_index::tilesForBoundingBox(
      lat - dlat, lon - dlon, lat + dlat, lon + dlon, gggs::Level(level_n));
    tiles.insert(tiles.end(), level_tiles.begin(), level_tiles.end());
  }
  return marine_survey_index::queryPasses(db_, tiles, "");
}

std::vector<IndexedTile> SurveyIndexBridge::indexedTiles() const
{
  // gggs::GridIndex knows its own bounds but its (level, row, col) constructor
  // is private, so mirror its accessors using the public gggs::levels specs.
  // The formulas must stay in lockstep with GridIndex::southLatitude() etc. —
  // the bridge test pins them against a GridIndex built from coordinates.
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(
      db_,
      "SELECT DISTINCT level, tile_row, tile_col FROM passes"
      " ORDER BY level, tile_row, tile_col;",
      -1, &stmt, nullptr) != SQLITE_OK)
  {
    throw std::runtime_error(
      std::string("survey index tile scan failed: ") + sqlite3_errmsg(db_));
  }
  std::vector<IndexedTile> tiles;
  int rc;
  while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
    const auto level = sqlite3_column_int64(stmt, 0);
    const auto row_i = sqlite3_column_int64(stmt, 1);
    const auto col_i = sqlite3_column_int64(stmt, 2);
    constexpr sqlite3_int64 kMaxRowCol = std::numeric_limits<std::uint32_t>::max();
    if (level < 0 || static_cast<std::size_t>(level) >= gggs::levels.size() ||
      row_i < 0 || col_i < 0 || row_i > kMaxRowCol || col_i > kMaxRowCol)
    {
      continue;   // out-of-contract row; the schema check already vouched
    }
    const auto row = static_cast<std::uint32_t>(row_i);
    const auto col = static_cast<std::uint32_t>(col_i);
    const auto & spec = gggs::levels[level];
    IndexedTile tile;
    tile.level = static_cast<std::uint8_t>(level);
    tile.row = row;
    tile.col = col;
    // Bounds arithmetic in double: a uint32 `row + 1` wraps at UINT32_MAX,
    // which would alias a corrupt row's tile onto tile 0's real location.
    const double row_d = static_cast<double>(row);
    const double col_d = static_cast<double>(col);
    tile.south = std::clamp(-96.0 + row_d * spec.grid_angular_span, -90.0, 90.0);
    tile.north = std::clamp(-96.0 + (row_d + 1.0) * spec.grid_angular_span, -90.0, 90.0);
    const double lon_span = spec.gridLongitudinalSpan(row);
    tile.west = -180.0 + col_d * lon_span;
    tile.east = -180.0 + (col_d + 1.0) * lon_span;
    tiles.push_back(tile);
  }
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) {
    throw std::runtime_error(
      std::string("survey index tile scan failed: ") + sqlite3_errmsg(db_));
  }
  return tiles;
}

std::optional<GeoExtent> SurveyIndexBridge::extent() const
{
  std::optional<GeoExtent> box;
  for (const auto & tile : indexedTiles()) {
    if (!box) {
      box = GeoExtent{tile.south, tile.west, tile.north, tile.east};
    } else {
      box->south = std::min(box->south, tile.south);
      box->west = std::min(box->west, tile.west);
      box->north = std::max(box->north, tile.north);
      box->east = std::max(box->east, tile.east);
    }
  }
  return box;
}

std::vector<marine_survey_index::PassRow> SurveyIndexBridge::queryTiles(
  const std::vector<IndexedTile> & tiles) const
{
  // The selection's tiles are already index keys. GridIndex's (level, row,
  // col) constructor is private, so rebuild each key from its centre — the
  // centre of a tile's own bounds always maps back to that tile.
  std::vector<gggs::GridIndex> keys;
  keys.reserve(tiles.size());
  for (const auto & tile : tiles) {
    keys.push_back(gggs::Level(tile.level).gridIndex(
        (tile.south + tile.north) / 2.0, (tile.west + tile.east) / 2.0));
  }
  return marine_survey_index::queryPasses(db_, keys, "");
}

std::vector<marine_survey_index::NavPoint> SurveyIndexBridge::navTrack() const
{
  // Nav points are posed pings' ground origins, and every posed ping put its
  // footprint tile in the index — so the pass-tile extent always covers the
  // track. An empty index (or a pre-#265 one regenerated without pings) has
  // no extent and no track.
  const auto box = extent();
  if (!box) {
    return {};
  }
  return marine_survey_index::queryNavTrackInBox(
    db_, box->south, box->west, box->north, box->east);
}

std::vector<std::pair<std::int64_t, std::string>> SurveyIndexBridge::bagPaths() const
{
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(
      db_, "SELECT id, path FROM bags ORDER BY id;", -1, &stmt, nullptr) != SQLITE_OK)
  {
    throw std::runtime_error(
      std::string("survey index bag scan failed: ") + sqlite3_errmsg(db_));
  }
  std::vector<std::pair<std::int64_t, std::string>> bags;
  int rc;
  while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
    const auto * path = sqlite3_column_text(stmt, 1);
    bags.emplace_back(
      static_cast<std::int64_t>(sqlite3_column_int64(stmt, 0)),
      path ? reinterpret_cast<const char *>(path) : "");
  }
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) {
    throw std::runtime_error(
      std::string("survey index bag scan failed: ") + sqlite3_errmsg(db_));
  }
  return bags;
}

}  // namespace marine_perception_tools
