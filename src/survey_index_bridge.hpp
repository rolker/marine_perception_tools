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

#ifndef SURVEY_INDEX_BRIDGE_HPP_
#define SURVEY_INDEX_BRIDGE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "marine_survey_index/query.hpp"

struct sqlite3;

namespace marine_perception_tools
{

// A geographic bounding box in degrees (south/west inclusive corners of the
// covered GGGS tiles).
struct GeoExtent
{
  double south = 0.0;
  double west = 0.0;
  double north = 0.0;
  double east = 0.0;
};

// One distinct indexed tile: its GGGS key plus its geographic bounds (derived
// from the public gggs::levels specs — the GridIndex row/col constructor is
// private; the formulas are pinned against GridIndex accessors in the test).
struct IndexedTile
{
  std::uint8_t level = 0;
  std::uint32_t row = 0;
  std::uint32_t col = 0;
  double south = 0.0;
  double west = 0.0;
  double north = 0.0;
  double east = 0.0;
};

// Headless bridge from a map click to the survey index: owns the
// `survey_index.db` connection and answers "which passes saw this point?"
// by reusing marine_survey_index's query library (tilesForBoundingBox +
// queryPasses — the schema contract in unh_marine_autonomy
// docs/survey_index_schema.md), not by re-implementing the SQL. Qt-free,
// unit-testable against a file DB populated through the same schema.
class SurveyIndexBridge
{
public:
  // Opens the index; throws std::runtime_error on a missing/incompatible DB
  // (the schema-version check's regenerate hint propagates).
  explicit SurveyIndexBridge(const std::string & db_path);
  ~SurveyIndexBridge();

  SurveyIndexBridge(const SurveyIndexBridge &) = delete;
  SurveyIndexBridge & operator=(const SurveyIndexBridge &) = delete;

  // Passes whose indexed footprint tiles cover the radius_m neighbourhood of
  // (lat, lon), across every tile level the index holds, ordered by bag and
  // time. Empty when the spot was never ensonified.
  std::vector<marine_survey_index::PassRow> queryPoint(
    double lat, double lon, double radius_m = 25.0) const;

  // Passes whose indexed footprint tiles intersect the geographic box,
  // across every tile level the index holds, optionally filtered by sensor
  // type (the queryPasses filter, e.g. "mbes-bathy"; "" = all). The box
  // CUBE lab's pass gather (#27).
  std::vector<marine_survey_index::PassRow> queryBox(
    double south, double west, double north, double east,
    const std::string & sensor_filter = "") const;

  // Union of the geographic bounds of every indexed pass tile — what the
  // overview fits its view to when no store tiles are available, so the map
  // click can still be aimed. nullopt when the index holds no passes.
  std::optional<GeoExtent> extent() const;

  // Every distinct indexed pass tile with its bounds — the selectable tile
  // grid the explorer map draws (#24). Ordered by (level, row, col).
  std::vector<IndexedTile> indexedTiles() const;

  // Passes covering exactly the given tiles (the map's selection — the tiles
  // ARE index keys, no bounding-box detour), ordered by bag and time.
  std::vector<marine_survey_index::PassRow> queryTiles(
    const std::vector<IndexedTile> & tiles) const;

  // The whole survey's decimated nav track (schema v2, #265), ordered by bag
  // then time — segment into per-bag polylines at bag_id changes. Empty when
  // the index predates the track table's population or holds no posed pings.
  std::vector<marine_survey_index::NavPoint> navTrack() const;

  // bag_id -> bag path for every indexed bag (the schema's bags table) — the
  // time bar resolves a nav-track fix's bag into an openable path with this.
  std::vector<std::pair<std::int64_t, std::string>> bagPaths() const;

private:
  sqlite3 * db_ = nullptr;
};

}  // namespace marine_perception_tools

#endif  // SURVEY_INDEX_BRIDGE_HPP_
