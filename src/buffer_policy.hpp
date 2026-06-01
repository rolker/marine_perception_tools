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

#ifndef BUFFER_POLICY_HPP_
#define BUFFER_POLICY_HPP_

#include <algorithm>

namespace marine_perception_tools
{

// Pure window-buffering policy for the windowed File->Open scrub (Milestone D).
// No Qt, no ROS — just time arithmetic, so the breakable logic is unit-testable
// in isolation. All times are seconds from bag start.
//
// Two spans are in play for a view time `t`:
//   * the REPLAY span `[t - integration, t]` — exactly what the engine
//     accumulates, so the costmap at `t` is a pure function of (t, params, bag)
//     and never depends on how the cache was reached (R1);
//   * the I/O CACHE span `[cache_lo, cache_hi]` — frames already read into
//     memory, kept as one contiguous span and trimmed to `max_span` so nearby
//     scrubs avoid re-reading the bag (retention; R3). The cache is an
//     optimization only — it never widens what the engine replays.
//
// The GUARANTEED window `[t - integration - margin, t + margin]` (clamped to the
// bag) is what a reload materializes: it covers the replay span plus `margin` of
// reload-free scrub slack on each side. `max_span = (integration + 2*margin) +
// retention`, so trimming the cache can never cut inside the guaranteed window
// (the retention floor is the guaranteed-window length, R9c).

enum class BufferAction
{
  NoReload,    // replay span already cached — re-slice/seek, no disk read
  Extend,      // grow the contiguous cache one side to cover the guaranteed window
  FarReload,   // guaranteed window doesn't overlap (or straddles both ends of) the
               // cache — drop it and read the guaranteed window fresh
  FreshLoad,   // no cache yet (first File->Open) — read the guaranteed window
};

struct BufferParams
{
  double integration_s = 90.0;   // = integration_halflives * decay_half_life_s
  double margin_s = 10.0;        // reload-free scrub slack each side of replay
  double retention_s = 120.0;    // extra cached span kept beyond the guaranteed window
  double bag_lo = 0.0;           // effective scrubbable lower bound (honors --start-s)
  double bag_hi = 0.0;           // effective upper bound (honors --end-s / duration)
};

struct BufferState
{
  bool has_cache = false;
  double cache_lo = 0.0;
  double cache_hi = 0.0;
};

struct BufferPlan
{
  BufferAction action = BufferAction::FreshLoad;
  double replay_lo = 0.0;   // engine accumulates exactly [replay_lo, replay_hi]
  double replay_hi = 0.0;
  double cache_lo = 0.0;    // resulting I/O cache extent after this operation
  double cache_hi = 0.0;
  bool read = false;        // true if [read_lo, read_hi] must be read from disk
  double read_lo = 0.0;     // span to read this operation (valid only if `read`)
  double read_hi = 0.0;

  // The actual warm-up the engine gets. Equals integration_s except near the bag
  // start, where the replay span clamps short — the caller surfaces this (R5/R9a).
  double warmup_s() const {return replay_hi - replay_lo;}
};

// Decide what to do to render view time `t` given the current cache. `t` is
// assumed already clamped to [bag_lo, bag_hi] by the caller (the scrubber range);
// the spans below clamp defensively regardless.
inline BufferPlan plan_buffer(double t, const BufferParams & p, const BufferState & s)
{
  const double B0 = p.bag_lo;
  const double B1 = std::max(p.bag_lo, p.bag_hi);
  t = std::clamp(t, B0, B1);

  const double integration = std::max(0.0, p.integration_s);
  const double margin = std::max(0.0, p.margin_s);
  const double retention = std::max(0.0, p.retention_s);

  BufferPlan plan;
  plan.replay_lo = std::clamp(t - integration, B0, B1);
  plan.replay_hi = t;  // already clamped to [B0,B1]

  const double g_lo = std::clamp(t - integration - margin, B0, B1);
  const double g_hi = std::clamp(t + margin, B0, B1);
  // Trimming never cuts inside the guaranteed window, so the retention floor is
  // the guaranteed-window length (R9c). max_span is expressed off the clamped
  // guaranteed length so it stays consistent near the bag edges.
  const double max_span = (g_hi - g_lo) + retention;

  // No cache yet → fresh read of the guaranteed window.
  if (!s.has_cache) {
    plan.action = BufferAction::FreshLoad;
    plan.cache_lo = g_lo;
    plan.cache_hi = g_hi;
    plan.read = true;
    plan.read_lo = g_lo;
    plan.read_hi = g_hi;
    return plan;
  }

  const double C0 = s.cache_lo;
  const double C1 = s.cache_hi;

  // Required data (the replay span) already cached → no disk read. The cache may
  // still need trimming if `integration` shrank since it was built, so apply the
  // trim below by falling through with merged == current cache.
  const bool replay_cached = (plan.replay_lo >= C0 - 1e-9) && (plan.replay_hi <= C1 + 1e-9);

  // Far jump: the guaranteed window does not overlap the cache at all → the old
  // frames can't contribute warm-up and the engine needs one contiguous span, so
  // drop the cache and read the guaranteed window fresh (Roland, 2026-05-31).
  if (!replay_cached && (g_hi < C0 || g_lo > C1)) {
    plan.action = BufferAction::FarReload;
    plan.cache_lo = g_lo;
    plan.cache_hi = g_hi;
    plan.read = true;
    plan.read_lo = g_lo;
    plan.read_hi = g_hi;
    return plan;
  }

  // Otherwise the new requirement overlaps the cache. Grow to cover the
  // guaranteed window. If growth is needed on BOTH sides (only happens when
  // `integration` jumped up via the half-life knob, leaving a too-small cache
  // around t), the cached middle isn't worth stitching around two reads — drop
  // and reload the guaranteed window fresh.
  const double merged_lo = std::min(C0, g_lo);
  const double merged_hi = std::max(C1, g_hi);
  const bool grow_low = merged_lo < C0 - 1e-9;
  const bool grow_high = merged_hi > C1 + 1e-9;

  if (grow_low && grow_high) {
    plan.action = BufferAction::FarReload;
    plan.cache_lo = g_lo;
    plan.cache_hi = g_hi;
    plan.read = true;
    plan.read_lo = g_lo;
    plan.read_hi = g_hi;
    return plan;
  }

  // NoReload (replay already cached, just trim) or one-sided Extend.
  double new_lo = merged_lo;
  double new_hi = merged_hi;

  // Retention trim: shrink to max_span, removing from the end farthest from `t`
  // first, but never cutting inside the guaranteed window [g_lo, g_hi].
  double excess = (new_hi - new_lo) - max_span;
  while (excess > 1e-9) {
    const double room_lo = g_lo - new_lo;   // trimmable below the guaranteed window
    const double room_hi = new_hi - g_hi;   // trimmable above it
    const double dist_lo = t - new_lo;
    const double dist_hi = new_hi - t;
    bool trimmed = false;
    if (dist_lo >= dist_hi && room_lo > 1e-9) {
      const double cut = std::min(excess, room_lo);
      new_lo += cut; excess -= cut; trimmed = true;
    } else if (room_hi > 1e-9) {
      const double cut = std::min(excess, room_hi);
      new_hi -= cut; excess -= cut; trimmed = true;
    } else if (room_lo > 1e-9) {
      const double cut = std::min(excess, room_lo);
      new_lo += cut; excess -= cut; trimmed = true;
    }
    if (!trimmed) {break;}  // nothing left to trim outside the guaranteed window
  }

  plan.cache_lo = new_lo;
  plan.cache_hi = new_hi;

  if (replay_cached) {
    plan.action = BufferAction::NoReload;
    plan.read = false;
    return plan;
  }

  // One-sided extend: read only the newly-exposed slice beyond the old cache.
  plan.action = BufferAction::Extend;
  plan.read = true;
  if (grow_low) {
    plan.read_lo = new_lo;
    plan.read_hi = C0;
  } else {  // grow_high
    plan.read_lo = C1;
    plan.read_hi = new_hi;
  }
  return plan;
}

}  // namespace marine_perception_tools

#endif  // BUFFER_POLICY_HPP_
