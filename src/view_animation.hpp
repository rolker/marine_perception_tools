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

#ifndef VIEW_ANIMATION_HPP_
#define VIEW_ANIMATION_HPP_

// The eased glide the middle-click recentre travels along (#42).
//
// A view that jumps costs the operator the re-orientation it was meant to
// save: nothing on screen says whether the map moved a metre or a kilometre,
// so the eye has to re-find the survey. A short glide carries that
// continuity, and the ease-in/ease-out is what makes it read as the map
// moving rather than a slide starting and stopping abruptly.
//
// Kept Qt-free and header-only so the two properties the animation actually
// depends on are testable without a widget: the endpoints must be EXACT (a
// glide that settles 1e-16 off its target leaves the canvas one repaint short
// of a cache-exact view, and a recentre that misses the clicked point is the
// bug the gesture exists to avoid), and the curve must be monotonic (any
// dip would show as the map backing up mid-flight).

#include <algorithm>

namespace marine_perception_tools
{

/// How long a middle-click recentre takes, milliseconds. Long enough for the
/// eye to follow the map across, short enough that it never feels like a wait
/// before the next click.
constexpr int kRecenterDurationMs = 750;

/// Normalised progress through an animation, clamped to [0, 1].
/// A non-positive duration means "no animation": progress is immediately 1,
/// which is the instant-completion path headless runs and tests use.
inline double animationProgress(double elapsed_ms, double duration_ms)
{
  if (duration_ms <= 0.0) {
    return 1.0;
  }
  return std::clamp(elapsed_ms / duration_ms, 0.0, 1.0);
}

/// Cubic ease-in-out: accelerates out of the start, coasts, decelerates into
/// the end. Exact at both endpoints (0 -> 0, 1 -> 1), monotonically
/// increasing in between, and symmetric about the midpoint, where it passes
/// through exactly 0.5.
inline double easeInOutCubic(double t)
{
  t = std::clamp(t, 0.0, 1.0);
  if (t < 0.5) {
    return 4.0 * t * t * t;
  }
  const double u = 2.0 - 2.0 * t;   // 0 at t == 1, so the endpoint is exact
  return 1.0 - 0.5 * u * u * u;
}

/// Interpolate `from` -> `to` along the eased curve at raw progress `t`.
/// The endpoints are returned unmodified rather than computed: `from + (to -
/// from) * 1.0` is not exactly `to` in floating point, and the canvas
/// compares its settled centre against the cached one for equality.
inline double easedInterpolate(double from, double to, double t)
{
  if (t <= 0.0) {
    return from;
  }
  if (t >= 1.0) {
    return to;
  }
  return from + (to - from) * easeInOutCubic(t);
}

}  // namespace marine_perception_tools

#endif  // VIEW_ANIMATION_HPP_
