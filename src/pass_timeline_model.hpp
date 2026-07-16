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

#ifndef PASS_TIMELINE_MODEL_HPP_
#define PASS_TIMELINE_MODEL_HPP_

// Pure time-axis model for the pass timeline pane (#24): maps the selection's
// pass intervals onto a gap-compressed x axis in [0, 1]. A three-week campaign
// whose selected tiles were visited on three days must not render as three
// slivers in an ocean of empty axis — the idle time between covered spans
// compresses to a fixed visual break. Qt-free and unit-testable.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace marine_perception_tools
{

// One pass interval to lay out. `row` is the display lane (one per sensor
// type); it plays no part in the axis math but drives hit-testing.
struct TimelinePass
{
  std::int64_t t0 = 0;
  std::int64_t t1 = 0;
  int row = 0;
};

// A covered span of survey time and the x range [0, 1] it occupies.
struct TimelineSpan
{
  std::int64_t t0 = 0;
  std::int64_t t1 = 0;
  double x0 = 0.0;
  double x1 = 0.0;
};

struct TimelineLayout
{
  std::vector<TimelineSpan> spans;                 // time-ordered, disjoint
  std::vector<std::pair<double, double>> pass_x;   // per input pass {x0, x1}

  // Fraction of the axis at time t: linear inside a span, pinned to the
  // nearest span edge in a compressed gap or outside the covered range.
  double xOf(std::int64_t t) const
  {
    if (spans.empty()) {
      return 0.0;
    }
    if (t <= spans.front().t0) {
      return spans.front().x0;
    }
    for (const auto & span : spans) {
      if (t <= span.t1) {
        if (t < span.t0) {
          return span.x0;   // inside the compressed gap before this span
        }
        const double dur = static_cast<double>(span.t1 - span.t0);
        if (dur <= 0.0) {
          return span.x0;
        }
        return span.x0 + (span.x1 - span.x0) *
               (static_cast<double>(t - span.t0) / dur);
      }
    }
    return spans.back().x1;
  }
};

// Lays the passes out on the compressed axis. Intervals padded by pad_ns are
// union-merged into covered spans; each inter-span gap renders as a fixed
// gap_frac of the axis (shrunk if many gaps would overflow it), and the spans
// share the remaining width proportionally to their durations.
inline TimelineLayout layoutTimeline(
  const std::vector<TimelinePass> & passes,
  std::int64_t pad_ns = 0, double gap_frac = 0.02)
{
  TimelineLayout layout;
  if (passes.empty()) {
    return layout;
  }

  // Union-merge the padded intervals into disjoint covered spans.
  std::vector<std::pair<std::int64_t, std::int64_t>> padded;
  padded.reserve(passes.size());
  for (const auto & pass : passes) {
    const std::int64_t lo = std::min(pass.t0, pass.t1) - pad_ns;
    const std::int64_t hi = std::max(pass.t0, pass.t1) + pad_ns;
    padded.emplace_back(lo, hi);
  }
  std::sort(padded.begin(), padded.end());
  for (const auto & [lo, hi] : padded) {
    if (!layout.spans.empty() && lo <= layout.spans.back().t1) {
      layout.spans.back().t1 = std::max(layout.spans.back().t1, hi);
    } else {
      layout.spans.push_back(TimelineSpan{lo, hi, 0.0, 0.0});
    }
  }

  // Distribute the axis: gaps get a fixed fraction each (capped so at least
  // half the axis stays with the data), spans share the rest by duration.
  const std::size_t n_gaps = layout.spans.size() - 1;
  double per_gap = gap_frac;
  if (n_gaps > 0 && per_gap * static_cast<double>(n_gaps) > 0.5) {
    per_gap = 0.5 / static_cast<double>(n_gaps);
  }
  const double usable = 1.0 - per_gap * static_cast<double>(n_gaps);
  double total_dur = 0.0;
  for (const auto & span : layout.spans) {
    total_dur += static_cast<double>(span.t1 - span.t0);
  }
  double x = 0.0;
  for (auto & span : layout.spans) {
    const double w = (total_dur > 0.0) ?
      usable * static_cast<double>(span.t1 - span.t0) / total_dur :
      usable / static_cast<double>(layout.spans.size());
    span.x0 = x;
    span.x1 = x + w;
    x = span.x1 + per_gap;
  }
  layout.spans.back().x1 = std::min(layout.spans.back().x1, 1.0);

  layout.pass_x.reserve(passes.size());
  for (const auto & pass : passes) {
    layout.pass_x.emplace_back(
      layout.xOf(std::min(pass.t0, pass.t1)),
      layout.xOf(std::max(pass.t0, pass.t1)));
  }
  return layout;
}

// The pass under (x, row), or -1. When bars overlap in a lane, the narrowest
// match wins — the small bar would otherwise be un-clickable under a big one.
inline int hitPass(
  const std::vector<TimelinePass> & passes, const TimelineLayout & layout,
  double x, int row)
{
  int best = -1;
  double best_width = 2.0;
  for (std::size_t i = 0; i < passes.size() && i < layout.pass_x.size(); ++i) {
    if (passes[i].row != row) {
      continue;
    }
    const auto & [x0, x1] = layout.pass_x[i];
    if (x < x0 || x > x1) {
      continue;
    }
    const double width = x1 - x0;
    if (width < best_width) {
      best_width = width;
      best = static_cast<int>(i);
    }
  }
  return best;
}

}  // namespace marine_perception_tools

#endif  // PASS_TIMELINE_MODEL_HPP_
