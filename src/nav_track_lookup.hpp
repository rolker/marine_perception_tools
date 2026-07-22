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

#ifndef NAV_TRACK_LOOKUP_HPP_
#define NAV_TRACK_LOOKUP_HPP_

// Time -> boat position over the campaign nav track (#24 time-bar arrow):
// given the survey index's decimated nav_track rows (ordered by bag then
// time), interpolate the position and course at an instant. Pure and cheap
// (binary search + lerp), so the arrow can follow a live tape drag.
// Interpolation stays within one bag's track — the stride is distance-based
// (10 m), so points can be minutes apart while station-keeping, but the
// interpolated position is still within a stride of truth. Between bags
// there is no data: nullopt, the arrow hides.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "marine_survey_index/query.hpp"

namespace marine_perception_tools
{

struct TrackFix
{
  double lat = 0.0;
  double lon = 0.0;
  double heading_rad = 0.0;   // course over ground, CW from north
  std::int64_t bag_id = 0;
};

// `track` is the bridge's navTrack() result: ordered by bag, then time.
inline std::optional<TrackFix> fixAtTime(
  const std::vector<marine_survey_index::NavPoint> & track, std::int64_t t_ns)
{
  auto seg_begin = track.begin();
  while (seg_begin != track.end()) {
    auto seg_end = seg_begin;
    while (seg_end != track.end() && seg_end->bag_id == seg_begin->bag_id) {
      ++seg_end;
    }
    if (t_ns >= seg_begin->t_ns && t_ns <= (seg_end - 1)->t_ns) {
      // In this bag: bracket by time (the segment is time-ordered).
      const auto after = std::lower_bound(
        seg_begin, seg_end, t_ns,
        [](const marine_survey_index::NavPoint & p, std::int64_t t) {
          return p.t_ns < t;
        });
      const auto b = (after == seg_end) ? after - 1 : after;
      const auto a = (b == seg_begin) ? b : b - 1;
      TrackFix fix;
      fix.bag_id = seg_begin->bag_id;
      const double span = static_cast<double>(b->t_ns - a->t_ns);
      const double u = (span > 0.0) ?
        std::clamp(static_cast<double>(t_ns - a->t_ns) / span, 0.0, 1.0) : 0.0;
      fix.lat = a->latitude + u * (b->latitude - a->latitude);
      fix.lon = a->longitude + u * (b->longitude - a->longitude);
      // Course from the bracketing pair (falling back to the neighbouring
      // pair at the segment edges). atan2(east, north) = CW from north.
      auto ca = a;
      auto cb = b;
      if (ca == cb) {
        if (cb + 1 != seg_end) {
          ++cb;
        } else if (ca != seg_begin) {
          --ca;
        }
      }
      const double dlat = cb->latitude - ca->latitude;
      const double dlon = (cb->longitude - ca->longitude) *
        std::cos(fix.lat * M_PI / 180.0);
      fix.heading_rad = (dlat == 0.0 && dlon == 0.0) ? 0.0 : std::atan2(dlon, dlat);
      return fix;
    }
    seg_begin = seg_end;
  }
  return std::nullopt;
}

}  // namespace marine_perception_tools

#endif  // NAV_TRACK_LOOKUP_HPP_
