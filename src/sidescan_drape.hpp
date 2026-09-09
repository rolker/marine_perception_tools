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

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
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
  // Quality score of the sample that painted each cell — higher wins a
  // conflict. score = straightness x range-closeness: pings on a straight
  // course beat pings mid-turn, and nearer (better-resolved) samples beat
  // far ones, so composites of several passes let the better pixels
  // through (Roland's design, 2026-08-18). Still never averaged.
  std::vector<float> painted_score;
  std::size_t pings_used = 0;
  std::size_t pings_skipped = 0;      // no altitude / no side / off-surface
  bool ok() const {return nx > 0 && ny > 0;}
};

// Drape pings (in the SAME world frame as `surface`) onto the surface. One
// pass or a concatenation of several (the composite mode): the per-cell
// quality score decides conflicts either way. Pings without altitude or a
// lateral side, or whose nadir has no estimated surface within a few cells
// (sensor z is anchored as nadir surface z + altitude), are counted skipped.
// Straightness is derived from each ping's yaw rate against its neighbours
// in the vector; a pass boundary's position jump makes the rate negligible,
// so concatenated passes need no explicit boundaries.
// The range half of the pixel-quality score. It is one factor of a product
// (straightness x range), so it decides a conflict outright only within a
// single ping, where the straightness factor is constant; across pings a
// farther sample on a straighter ping can win. The range factor is also
// clamped at 0.05, so far-range differences compress rather than vanish:
//  - Nearest: closer samples outrank far ones (best across-track
//    resolution, but composites favour near-nadir imagery with its
//    distortion);
//  - MidRange: the score peaks mid-swath (4u(1-u), u = slant/max_slant),
//    penalising BOTH the nadir region and the far edge — the classic
//    mosaicking preference.
enum class RangeScoreMode { Nearest, MidRange };

// `cancel` (optional) is polled per ping — the unit the march is built from,
// and the one a composite of several passes has tens of thousands of (#44).
// A cancelled march returns a drape that is not ok(): nothing to paint.
SidescanDrape drape_pass(
  const CubeSurface & surface, const std::vector<WindowPing> & pings,
  RangeScoreMode range_mode = RangeScoreMode::Nearest,
  const std::shared_ptr<std::atomic<bool>> & cancel = {});

// Drape terrain (#29 follow-up): the sidescan reaches past the MBES, so the
// surface is extended to the pass's swath before marching — the grid grows
// (whole cells, same origin lattice) to cover each ping's across-track
// reach, holes fill and edges extrapolate with a smooth membrane (nearest-
// measured seed + Laplacian relaxation, measured nodes pinned). Every node
// of the result is finite; `uncertainty` stays NaN on interpolated nodes so
// callers can tell measured from inferred terrain. Growth is capped by
// `max_nodes` (clamped proportionally, noted) — the operator's grid guard.
// `cancel` (optional) is polled per ping while the swath bounds are gathered,
// per BFS wave while the membrane seeds, and per relaxation sweep (#44) — the
// three loops that scale with the grid. A cancelled extension returns a
// surface that is not ok() and notes "cancelled", so no caller mistakes a
// half-relaxed membrane for terrain.
CubeSurface extend_surface_for_drape(
  const CubeSurface & surface, const std::vector<WindowPing> & pings,
  std::uint64_t max_nodes, std::string & note,
  const std::shared_ptr<std::atomic<bool>> & cancel = {});

}  // namespace marine_perception_tools

#endif  // SIDESCAN_DRAPE_HPP_
