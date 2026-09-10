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

#ifndef CUBE_LAB_HPP_
#define CUBE_LAB_HPP_

// In-app CUBE over a box of soundings (#27, uma#258 stage 4): the callable
// cube_bathymetry library (cube::Grid + cube::Parameters) run at operator
// resolution over soundings the explorer already loaded, producing a
// heightmap surface with per-cell uncertainty and CUBE-settled backscatter
// intensity (ADR-0007: intensity rides the hypothesis queue bound to its
// depth). Qt-free and bag-free — unit-testable with synthetic soundings.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "marine_colormap/colormap.hpp"
#include "mbes_geometry.hpp"

namespace marine_perception_tools
{

// A CUBE-estimated heightmap in the soundings' (reference world) frame.
// Node (col x, row y) sits at (origin_x + x*cell_m, origin_y + y*cell_m);
// arrays are row-major (y * nx + x), NaN = no estimate at that node.
struct CubeSurface
{
  double origin_x = 0.0;
  double origin_y = 0.0;
  double cell_m = 0.0;
  int nx = 0;
  int ny = 0;
  std::vector<float> depth;
  std::vector<float> uncertainty;
  std::vector<float> intensity;
  std::size_t soundings_in = 0;
  std::string note;           // human-readable failure reason when !ok()
  // Encodes the invariant every consumer relies on, not just non-emptiness:
  // mesh building, drape and extend all divide by `cell_m` and index all three
  // arrays at `y * nx + x`. A surface that passed the old check with a zero
  // cell size or a short array would have reached them.
  bool ok() const
  {
    if (nx <= 0 || ny <= 0 || !(cell_m > 0.0)) {
      return false;
    }
    const std::size_t n =
      static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny);
    return depth.size() == n && uncertainty.size() == n &&
           intensity.size() == n;
  }
};


// The operator-tunable subset of cube::Parameters (#27, the cube#98 tuning
// harness intent): concrete values, seeded from the library's real defaults
// by default_cube_tuning() so they can never silently drift from upstream.
// distance_exponent and the derived scales stay library-managed (they are
// recomputed from the cell size and interlock with each other).
struct CubeTuning
{
  // Scale on depth for how far out a sounding is accepted (unitless;
  // hydrography ~0.05, larger for sparse/flat geological mapping).
  float capture_distance_scale = 0.05f;
  // The vertical-uncertainty budget, set directly rather than only by picking
  // a named IHO order (#45). Together they are
  //   max variance allowed(depth) = (iho_fixed^2 + (iho_percent*depth)^2)
  //                                 / CONF_95PC^2,
  // whose ratio against a sounding's own vertical error scales the RADIUS over
  // which that sounding spreads its influence on insertion — a looser budget
  // gives every sounding a larger radius, so the surface fills in and smooths;
  // a tighter one shrinks it, so the surface is crisper and holier. It is a
  // knob on how far the estimator may extrapolate, NOT a pass/fail gate, which
  // is why non-hydrographic work wants the numbers and not the standards
  // label (S-44 orders span 0.15 m/0.75% at exclusive to 1.0 m/2.3% at
  // order 2; go looser still for sparse or flat geological mapping).
  //
  // Held here in the operator's units — metres and a fraction of depth, the
  // numbers S-44 is written in. cube::Parameters stores their SQUARES, so
  // run_cube squares on the way in and default_cube_tuning() takes the root on
  // the way out.
  float iho_fixed = 0.5f;      // fixed part of the budget, m (95% confidence)
  float iho_percent = 0.013f;  // depth-proportional part, fraction of depth
  std::uint32_t median_length = 11;      // median pre-filter queue length
  float quotient_limit = 30.0f;          // outlier quotient upper limit
  float discount = 1.0f;                 // evolution noise discount factor
  float estimate_offset = 4.0f;          // intervention offset threshold
  float bayes_factor_threshold = 0.135f;   // intervention Bayes factor
  std::uint32_t runlength_threshold = 5;   // intervention run length
  int extractor = 1;   // cube::CubeExtractor: 0 prior, 1 lhood, 2 posterior
  // Angular-response correction (cube#81 ARA): path to a per-sonar curve
  // CSV ({abs_angle_deg, db_relative_to_nadir} + optional tier-2 TL header,
  // e.g. ~/data/logs/analysis/m3_angular_response_curve.csv). Empty = off
  // (raw intensities). Loaded per run; a bad file surfaces in the note.
  std::string ara_curve_path;
  // Grid-size guard, operator-owned (not a hidden policy): the largest node
  // count a run may allocate. Baseline cost is ~20 B/node (node-pointer +
  // output arrays) before per-populated-node CUBE state, so the 100M default
  // is ~2 GB. Editable in the params dialog; the run confirmation offers a
  // one-shot override when a box would exceed it.
  std::uint64_t max_nodes = 100000000;
};

// The library's own defaults, read from a real cube::Parameters instance.
CubeTuning default_cube_tuning();

// The named IHO S-44 orders, kept as PRESETS that seed a CubeTuning's
// iho_fixed / iho_percent (#45) — the run itself uses whatever those two
// numbers currently are, never the label. Names are the cube::Parameters
// vocabulary with ONE deliberate departure: "order1a/1b", because S-44's
// order 1a and 1b carry the SAME vertical-uncertainty budget (0.5 m, 1.3%)
// and differ only in the seafloor-search requirement, which CUBE does not
// model — two dropdown entries where one is a silent no-op is misleading.
std::vector<std::string> iho_preset_names();

// The {iho_fixed, iho_percent} a preset seeds, or nullopt if `name` is not a
// preset (including "custom").
std::optional<std::pair<float, float>> iho_preset_limits(const std::string & name);

// The inverse: the preset whose pair these values are, else "custom" — how an
// edited threshold takes the selection off the named orders.
std::string iho_order_for_limits(float iho_fixed, float iho_percent);

// The selection name for values that match no preset.
inline constexpr const char * kCustomIhoOrder = "custom";

// Self-calibrated angular-response curve (#27, operator request 2026-08-18):
// derive the residual empirically from a box's OWN beams instead of trusting
// the campaign curve + physical TL model — kills whatever gain behaviour the
// sonar actually has, by construction. Beams are TL-corrected
// (40log10(R) + 2*alpha*R) then binned by |rx angle| (2-degree bins, sparse
// bins dropped); the residual is each bin's mean relative to the most-nadir
// bin. Writes the standard 4-column curve CSV with a tier-2 header (so
// run_cube consumes it like any other curve). Returns "" on success, else
// the reason (e.g. too few angled beams to calibrate).
std::string derive_box_curve(
  const std::vector<MbesSounding> & soundings, double absorption_db_per_m,
  const std::string & csv_path);

// Run CUBE over `soundings` (already gathered and box-clipped, all in one
// world frame; z up, seabed negative — the cube depth convention) on a grid
// of `cell_m` cells under the given tuning overrides. The grid covers the
// soundings' bounding box plus one cell of margin.
//
// The uncertainty budget comes from `tuning.iho_fixed` / `tuning.iho_percent`
// and nothing else (#45): there is no order-name argument to fall out of step
// with the two numbers the operator edited.
//
// Per-sounding errors: the full cube_bathymetry ErrorModel needs the raw
// detections + vessel/device config, which the explorer's cloud path does
// not retain — so this uses a documented PLACEHOLDER, angle-aware since #49:
// each beam's own angle and slant range propagated through
// sounding_uncertainty.hpp (stored as variances per the Sounding contract),
// seeded from cube::Device's defaults, until detections are carried through
// (follow-up on #27). It replaced a depth-only formula that gave a swath-edge
// beam the same confidence as a nadir one; it does NOT model refraction,
// which is systematic (#28). A sounding carrying no beam geometry is skipped
// rather than given a fabricated error, and the note says how many were.
//
// `cancel` (optional) is polled per sounding while the soundings are spread
// over the node grid, and per node while the estimates are extracted (#44) —
// the two loops a box-sized run spends its minutes in. cube::Node exposes no
// finer unit than one insert / one extraction, and neither is itself long, so
// these are the useful granularity. A cancelled run returns a surface that is
// not ok(), noted "cancelled": never a partial grid dressed as an estimate.
CubeSurface run_cube(
  const std::vector<MbesSounding> & soundings, double cell_m,
  const CubeTuning & tuning = CubeTuning{},
  const std::shared_ptr<std::atomic<bool>> & cancel = {});

// A renderable triangulation of a CubeSurface: positions in the surface's
// world frame (xyz triples), per-vertex colours (rgb triples in [0,1]),
// triangle indices. Only cells whose four corners all carry estimates
// triangulate — gaps stay gaps.
struct CubeSurfaceMesh
{
  std::vector<float> positions;
  std::vector<float> colors;
  std::vector<std::uint32_t> indices;
  float scalar_lo = 0.0f;   // the colour ramp's data range (for readouts)
  float scalar_hi = 0.0f;
};

enum class CubeShade { Depth, Uncertainty, Intensity };

// How the surface triangulates and takes its colour (#27/#29):
//  - CrispSmooth (the lab default): every node is one CONSTANT-colour quad
//    (no texture blending — each CUBE cell one crisp texel) whose corners
//    take the mean height of the adjacent nodes, so the relief is a
//    watertight smooth membrane. Crisp texture, smooth geometry.
//  - CrispStepped: constant-colour quads at each node's own depth — the
//    fully literal view (stepped plateaus, one step per cell).
//  - Blended: one vertex per node, colours Gouraud-interpolate across
//    triangles — the conventional smooth-shaded relief.
enum class CubeMeshStyle { CrispSmooth, CrispStepped, Blended };

// Triangulate + colour a surface with the given palette LUT. The colour
// ramp auto-scales to the finite range of the chosen scalar.
// `range` (optional) fixes the colour ramp to [lo, hi] instead of the
// scalar's finite extent — the CUBE row's manual range control (#27).
CubeSurfaceMesh build_cube_mesh(
  const CubeSurface & surface, CubeShade shade,
  const std::vector<marine_colormap::Rgba8> & lut,
  CubeMeshStyle style = CubeMeshStyle::CrispSmooth,
  const std::optional<std::pair<float, float>> & range = std::nullopt);

// Mesh from explicit per-node colours (`node_rgb`: rgb triples in [0,1],
// ny*nx*3, row-major like the surface arrays) — the sidescan drape's path
// (#29): the caller owns each node's colour, this only triangulates.
// Unestimated nodes are holes regardless of their colour entries.
CubeSurfaceMesh build_cube_mesh_colored(
  const CubeSurface & surface, const std::vector<float> & node_rgb,
  CubeMeshStyle style);

}  // namespace marine_perception_tools

#endif  // CUBE_LAB_HPP_
