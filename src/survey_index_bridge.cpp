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

std::optional<GeoExtent> SurveyIndexBridge::extent() const
{
  // gggs::GridIndex knows its own bounds but its (level, row, col) constructor
  // is private, so mirror its accessors using the public gggs::levels specs.
  // The formulas must stay in lockstep with GridIndex::southLatitude() etc. —
  // the bridge test pins them against a GridIndex built from coordinates.
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(
      db_, "SELECT DISTINCT level, tile_row, tile_col FROM passes;",
      -1, &stmt, nullptr) != SQLITE_OK)
  {
    throw std::runtime_error(
      std::string("survey index extent query failed: ") + sqlite3_errmsg(db_));
  }
  std::optional<GeoExtent> box;
  int rc;
  while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
    const auto level = sqlite3_column_int64(stmt, 0);
    const auto row = static_cast<std::uint32_t>(sqlite3_column_int64(stmt, 1));
    const auto col = static_cast<std::uint32_t>(sqlite3_column_int64(stmt, 2));
    if (level < 0 || static_cast<std::size_t>(level) >= gggs::levels.size()) {
      continue;   // out-of-contract row; the schema check already vouched for v1
    }
    const auto & spec = gggs::levels[level];
    const double south = std::clamp(-96.0 + row * spec.grid_angular_span, -90.0, 90.0);
    const double north = std::clamp(-96.0 + (row + 1) * spec.grid_angular_span, -90.0, 90.0);
    const double lon_span = spec.gridLongitudinalSpan(row);
    const double west = -180.0 + col * lon_span;
    const double east = -180.0 + (col + 1) * lon_span;
    if (!box) {
      box = GeoExtent{south, west, north, east};
    } else {
      box->south = std::min(box->south, south);
      box->west = std::min(box->west, west);
      box->north = std::max(box->north, north);
      box->east = std::max(box->east, east);
    }
  }
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) {
    throw std::runtime_error(
      std::string("survey index extent scan failed: ") + sqlite3_errmsg(db_));
  }
  return box;
}

}  // namespace marine_perception_tools
