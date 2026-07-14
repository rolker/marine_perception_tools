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

#include <string>
#include <vector>

#include "marine_survey_index/query.hpp"

struct sqlite3;

namespace marine_perception_tools
{

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

private:
  sqlite3 * db_ = nullptr;
};

}  // namespace marine_perception_tools

#endif  // SURVEY_INDEX_BRIDGE_HPP_
