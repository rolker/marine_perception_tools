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

#ifndef PASS_COALESCE_HPP_
#define PASS_COALESCE_HPP_

// Pure pass-segment coalescing (extracted from the stage-2 overview window,
// #24): queryPasses/queryTiles return one PassRow per (pass, tile), so a
// transit crossing several selected tiles comes back as several per-tile
// segments — merge them into one row per physical pass for the timeline and
// the cloud loader.

#include <algorithm>
#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "marine_survey_index/query.hpp"

namespace marine_perception_tools
{

// One physical pass over the selection.
struct CoalescedPass
{
  std::string bag_path;
  std::string sensor_type;
  std::string topic;
  std::int64_t t_start_ns = 0;
  std::int64_t t_end_ns = 0;
  std::int64_t ping_count = 0;
};

// Merge the per-tile segments so consumers see one row per physical pass.
// Segments sharing (bag, sensor, topic) whose time windows overlap or sit
// within kSegmentGapNs are one transit across adjacent tiles (their windows
// are back-to-back); a genuine revisit of the same spot is separated by far
// more, so it stays a distinct entry. Ping counts of merged segments are
// summed. Result is ordered by bag then start, matching the queryPasses
// contract downstream consumers rely on.
inline std::vector<CoalescedPass> coalescePasses(
  const std::vector<marine_survey_index::PassRow> & rows)
{
  // 5 s comfortably spans the inter-tile ping gap within one transit without
  // bridging two separate visits (survey revisits are minutes apart).
  constexpr std::int64_t kSegmentGapNs = 5LL * 1000000000LL;

  // rows arrive ordered by bag then t_start; bucket by (bag, sensor, topic) —
  // filtering that order per bucket keeps each bucket sorted by t_start, so a
  // running interval-merge against the last segment is correct.
  std::map<std::tuple<std::string, std::string, std::string>,
    std::vector<CoalescedPass>> buckets;
  for (const auto & row : rows) {
    auto & merged = buckets[{row.bag_path, row.sensor_type, row.topic}];
    if (!merged.empty() && row.t_start_ns <= merged.back().t_end_ns + kSegmentGapNs) {
      merged.back().t_end_ns = std::max(merged.back().t_end_ns, row.t_end_ns);
      merged.back().ping_count += row.ping_count;
    } else {
      merged.push_back(CoalescedPass{
          row.bag_path, row.sensor_type, row.topic,
          row.t_start_ns, row.t_end_ns, row.ping_count});
    }
  }

  std::vector<CoalescedPass> passes;
  for (auto & [key, merged] : buckets) {
    passes.insert(passes.end(), merged.begin(), merged.end());
  }
  std::sort(passes.begin(), passes.end(),
    [](const CoalescedPass & a, const CoalescedPass & b) {
      if (a.bag_path != b.bag_path) {
        return a.bag_path < b.bag_path;
      }
      return a.t_start_ns < b.t_start_ns;
    });
  return passes;
}

}  // namespace marine_perception_tools

#endif  // PASS_COALESCE_HPP_
