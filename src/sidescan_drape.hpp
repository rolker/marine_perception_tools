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

#ifndef SIDESCAN_DRAPE_HPP_
#define SIDESCAN_DRAPE_HPP_

// Surface-aware sidescan drape (#29, uma#258 stage 5): project one pass's
// sidescan samples onto the box-CUBE relief by slant range against the ACTUAL
// terrain instead of flat-at-nadir-depth. Per ping, a first-return march
// walks the across-track profile of the surface outward from nadir tracking
// the line-of-sight grazing angle: visible cells look up the recorded sample
// at their true slant range; terrain-occluded cells are marked SHADOW
// (distinct from unseen — and the grazing rays are exactly what the
// shadow-bathymetry sibling #30 consumes). Nearest sample per cell, no
// averaging (single pass = the unit of interpretation; blending kills
// shadows); where two pings of the pass cover a cell, the smaller slant
// range (better across-track resolution) wins. Qt-free and bag-free —
// unit-testable with synthetic terrain and pings.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "cube_lab.hpp"           // CubeSurface (the grid being draped)
#include "sidescan_bag_session.hpp"   // WindowPing (a pass's pings + geometry)

namespace marine_perception_tools
{

// Per-cell drape aligned with the CubeSurface it was built from (row-major
// y * nx + x, same origin/cell size). amplitude NaN = the pass never
// ensonified the cell; shadow = terrain blocked the ray (a real acoustic
// shadow, worth displaying dark rather than transparent).
struct SidescanDrape
{
  int nx = 0;
  int ny = 0;
  std::vector<float> amplitude;
  std::vector<std::uint8_t> shadow;
  std::vector<float> painted_slant;   // slant range that painted each cell
  std::size_t pings_used = 0;
  std::size_t pings_skipped = 0;      // no altitude / no side / off-surface
  bool ok() const {return nx > 0 && ny > 0;}
};

// Drape one pass (its pings in the SAME world frame as `surface`) onto the
// surface. Pings without altitude or a lateral side, or whose nadir has no
// estimated surface within a few cells (sensor z is anchored as nadir
// surface z + altitude), are counted skipped.
SidescanDrape drape_pass(
  const CubeSurface & surface, const std::vector<WindowPing> & pings);

}  // namespace marine_perception_tools

#endif  // SIDESCAN_DRAPE_HPP_
