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

}  // namespace marine_perception_tools
