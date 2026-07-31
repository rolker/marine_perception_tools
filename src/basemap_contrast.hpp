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

#ifndef BASEMAP_CONTRAST_HPP_
#define BASEMAP_CONTRAST_HPP_

// Robust contrast range for the explorer's store-tile basemap (#24). The
// stores carry residual outlier cells (pre-outlier-gate CUBE junk reaching
// kilometres of impossible depth), so a min/max range collapses the real
// data into a sliver of the colour scale — the "all black basemap" bug.
// Percentile clipping over a value sample keeps the scale on the data.
// Pure (no Qt/GDAL), unit-testable.

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

namespace marine_perception_tools
{

// The [lo_pct, hi_pct] percentile range of `values` (need not be sorted;
// reordered in place). Returns {0, 1} when empty; a degenerate (equal)
// percentile pair widens to the full min/max so a near-constant layer still
// renders, and if that is degenerate too the span is left to the caller's
// "span <= 0 -> treat as 1" convention.
inline std::pair<double, double> robust_range(
  std::vector<double> & values, double lo_pct = 0.02, double hi_pct = 0.98)
{
  if (values.empty()) {
    return {0.0, 1.0};
  }
  const auto rank = [&](double pct) {
      const auto n = static_cast<double>(values.size() - 1);
      return static_cast<std::size_t>(std::clamp(pct, 0.0, 1.0) * n + 0.5);
    };
  const std::size_t lo_i = rank(lo_pct);
  const std::size_t hi_i = rank(hi_pct);
  std::nth_element(values.begin(), values.begin() + lo_i, values.end());
  const double lo = values[lo_i];
  std::nth_element(values.begin() + lo_i, values.begin() + hi_i, values.end());
  const double hi = values[hi_i];
  if (hi > lo) {
    return {lo, hi};
  }
  const auto [mn, mx] = std::minmax_element(values.begin(), values.end());
  return {*mn, *mx};
}

}  // namespace marine_perception_tools

#endif  // BASEMAP_CONTRAST_HPP_
