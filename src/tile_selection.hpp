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

#ifndef TILE_SELECTION_HPP_
#define TILE_SELECTION_HPP_

// Pure tile-selection math for the explorer map (#24): hit-testing a point
// against the selectable index-tile rects, rubber-band intersection, and the
// toggle rule. Qt-free and frame-agnostic (any consistent 2D coordinates), so
// the interaction correctness is unit-tested without a widget.

#include <cstddef>
#include <set>
#include <vector>

namespace marine_perception_tools
{

// An axis-aligned selectable rect with x0 <= x1 and y0 <= y1.
struct SelectableRect
{
  double x0 = 0.0;
  double y0 = 0.0;
  double x1 = 0.0;
  double y1 = 0.0;
};

// Index of the first rect containing (x, y) (edges inclusive), or -1. Index
// tiles at one level never overlap, so "first" is not ambiguous in practice;
// with mixed levels the finer tiles should be ordered first by the caller.
inline int hitRect(const std::vector<SelectableRect> & rects, double x, double y)
{
  for (std::size_t i = 0; i < rects.size(); ++i) {
    const auto & r = rects[i];
    if (x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

// Indices of every rect intersecting the (any-corner-order) box — the
// rubber-band rule. Touching edges count as intersecting: a band dragged to a
// tile boundary should still take the tile under it.
inline std::vector<std::size_t> rectsInBox(
  const std::vector<SelectableRect> & rects,
  double bx0, double by0, double bx1, double by1)
{
  if (bx0 > bx1) {const double t = bx0; bx0 = bx1; bx1 = t;}
  if (by0 > by1) {const double t = by0; by0 = by1; by1 = t;}
  std::vector<std::size_t> hits;
  for (std::size_t i = 0; i < rects.size(); ++i) {
    const auto & r = rects[i];
    if (r.x1 >= bx0 && r.x0 <= bx1 && r.y1 >= by0 && r.y0 <= by1) {
      hits.push_back(i);
    }
  }
  return hits;
}

// Ctrl-click rule: toggle membership. Returns true if the index is selected
// after the toggle.
inline bool toggleSelection(std::set<std::size_t> & selection, std::size_t index)
{
  const auto it = selection.find(index);
  if (it != selection.end()) {
    selection.erase(it);
    return false;
  }
  selection.insert(index);
  return true;
}

}  // namespace marine_perception_tools

#endif  // TILE_SELECTION_HPP_
