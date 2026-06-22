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

#ifndef DISTANCE_BUFFER_POLICY_HPP_
#define DISTANCE_BUFFER_POLICY_HPP_

#include <algorithm>

// Pure rolling-distance scrub policy — the distance-indexed analogue of
// buffer_policy.hpp (which is time-indexed). No Qt, no ROS: just arithmetic, so
// the breakable window logic is unit-tested in isolation. The sidescan viewer
// scrubs along *distance travelled* (not time), so a stationary boat does not
// pile pings onto one spot and a fast boat does not compress them.

namespace marine_perception_tools
{

// The along-track span (metres) currently painted: [lo, hi], with hi at the scrub
// head and lo trailing by the window length. Clamped to [0, total].
struct DistanceWindow
{
  double lo = 0.0;
  double hi = 0.0;
};

// Window for a scrub head at along-track distance `head`, showing the trailing
// `window_len` metres. `head` is clamped to [0, total]; `lo` never goes below 0.
// A non-positive window_len collapses to an empty [head, head] span.
inline DistanceWindow distance_window(double head, double window_len, double total)
{
  const double tot = std::max(0.0, total);
  const double h = std::clamp(head, 0.0, tot);
  const double lo = h - std::max(0.0, window_len);
  return {std::max(0.0, lo), h};
}

// Cap for "how many pings to keep when the boat is stationary". When the window
// holds more than `max_pings` pings (e.g. the boat sat still and the same ground
// was ensonified repeatedly), keep only the most recent `max_pings` — return the
// index of the first ping to keep within a window-ordered list of `count` pings.
// `max_pings <= 0` means "no cap" (keep all). The newest pings are at the end of
// the window list (ascending distance), so we drop from the front.
inline int stationary_keep_from(int count, int max_pings)
{
  if (max_pings <= 0 || count <= max_pings) {return 0;}
  return count - max_pings;
}

}  // namespace marine_perception_tools

#endif  // DISTANCE_BUFFER_POLICY_HPP_
