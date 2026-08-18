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

#include <cstddef>
#include <cstdint>
#include <string>
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
  bool ok() const {return nx > 0 && ny > 0 && !depth.empty();}
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
  std::uint32_t median_length = 11;      // median pre-filter queue length
  float quotient_limit = 30.0f;          // outlier quotient upper limit
  float discount = 1.0f;                 // evolution noise discount factor
  float estimate_offset = 4.0f;          // intervention offset threshold
  float bayes_factor_threshold = 0.135f;   // intervention Bayes factor
  std::uint32_t runlength_threshold = 5;   // intervention run length
  int extractor = 1;   // cube::CubeExtractor: 0 prior, 1 lhood, 2 posterior
  // Grid-size guard, operator-owned (not a hidden policy): the largest node
  // count a run may allocate. Baseline cost is ~20 B/node (node-pointer +
  // output arrays) before per-populated-node CUBE state, so the 100M default
  // is ~2 GB. Editable in the params dialog; the run confirmation offers a
  // one-shot override when a box would exceed it.
  std::uint64_t max_nodes = 100000000;
};

// The library's own defaults, read from a real cube::Parameters instance.
CubeTuning default_cube_tuning();

// Run CUBE over `soundings` (already gathered and box-clipped, all in one
// world frame; z up, seabed negative — the cube depth convention) on a grid
// of `cell_m` cells under the given IHO order ("order1a" etc., the
// cube::Parameters vocabulary) and tuning overrides. The grid covers the
// soundings' bounding box plus one cell of margin.
//
// Per-sounding errors: the full cube_bathymetry ErrorModel needs the raw
// detections + vessel/device config, which the explorer's cloud path does
// not retain — so this uses a documented depth-dependent PLACEHOLDER
// (vertical std 0.1 m + 0.7% of depth, horizontal std 0.2 m + 1% of depth,
// stored as variances per the Sounding contract) until detections are
// carried through (follow-up on #27).
CubeSurface run_cube(
  const std::vector<MbesSounding> & soundings, double cell_m,
  const std::string & iho_order = "order1a",
  const CubeTuning & tuning = CubeTuning{});

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

// Triangulate + colour a surface with the given palette LUT. The colour
// ramp auto-scales to the finite range of the chosen scalar.
CubeSurfaceMesh build_cube_mesh(
  const CubeSurface & surface, CubeShade shade,
  const std::vector<marine_colormap::Rgba8> & lut);

}  // namespace marine_perception_tools

#endif  // CUBE_LAB_HPP_
