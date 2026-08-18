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

#include "cube_lab.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cube_bathymetry/node.h"
#include "cube_bathymetry/parameters.h"
#include "cube_bathymetry/sizes.h"
#include "cube_bathymetry/sounding.h"

namespace marine_perception_tools
{

CubeTuning default_cube_tuning()
{
  const cube::Parameters params(cube::CellSizes(1.0f));
  CubeTuning t;
  t.capture_distance_scale = params.capture_distance_scale;
  t.median_length = params.median_length;
  t.quotient_limit = params.quotient_limit;
  t.discount = params.discount;
  t.estimate_offset = params.estimate_offset;
  t.bayes_factor_threshold = params.bayes_factor_threshold;
  t.runlength_threshold = params.runlength_threshold;
  t.extractor = static_cast<int>(params.extractor);
  return t;
}

CubeSurface run_cube(
  const std::vector<MbesSounding> & soundings, double cell_m,
  const std::string & iho_order, const CubeTuning & tuning)
{
  CubeSurface out;
  out.cell_m = cell_m;
  out.soundings_in = soundings.size();
  if (soundings.empty()) {
    out.note = "no soundings in the box";
    return out;
  }
  if (!(cell_m > 0.0)) {
    out.note = "cell size must be positive";
    return out;
  }

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = min_x;
  double max_y = max_x;
  for (const auto & s : soundings) {
    min_x = std::min(min_x, s.x);
    max_x = std::max(max_x, s.x);
    min_y = std::min(min_y, s.y);
    max_y = std::max(max_y, s.y);
  }
  // One cell of margin so edge soundings still influence a bordering node.
  out.origin_x = min_x - cell_m;
  out.origin_y = min_y - cell_m;
  const double span_x = (max_x - min_x) + 2.0 * cell_m;
  const double span_y = (max_y - min_y) + 2.0 * cell_m;
  out.nx = static_cast<int>(std::ceil(span_x / cell_m)) + 1;
  out.ny = static_cast<int>(std::ceil(span_y / cell_m)) + 1;
  const std::size_t n_nodes =
    static_cast<std::size_t>(out.nx) * static_cast<std::size_t>(out.ny);
  if (n_nodes > kMaxCubeNodes) {
    out.note = "grid would be " + std::to_string(out.nx) + "x" +
      std::to_string(out.ny) + " nodes — shrink the box or coarsen the cells";
    out.nx = 0;
    out.ny = 0;
    return out;
  }

  const cube::CellSizes sizes(static_cast<float>(cell_m));   // square cells
  cube::Parameters params(sizes, iho_order);
  // Operator tuning (#27): the exposed subset only — the derived scales
  // (distance_scale etc.) stay as the ctor computed them from the cell size.
  params.capture_distance_scale = tuning.capture_distance_scale;
  params.median_length = std::max<std::uint32_t>(1, tuning.median_length);
  params.quotient_limit = tuning.quotient_limit;
  params.discount = tuning.discount;
  params.estimate_offset = tuning.estimate_offset;
  params.bayes_factor_threshold = tuning.bayes_factor_threshold;
  params.runlength_threshold = tuning.runlength_threshold;
  params.extractor = static_cast<cube::CubeExtractor>(
    std::clamp(tuning.extractor, 0, static_cast<int>(cube::CUBE_POSTERIOR)));

  // Drive cube::Node directly instead of cube::Grid: Grid::values() is the
  // depth-only legacy extraction, while Node::extractNodeRecord carries the
  // CUBE-settled backscatter (ADR-0007) this lab drapes with. The sounding
  // spread mirrors Grid::insert's effect square exactly (influence radius,
  // node-centre distance test).
  std::vector<std::unique_ptr<cube::Node>> nodes(n_nodes);
  for (const auto & s : soundings) {
    // World z is up (seabed negative) — the cube depth convention directly.
    cube::Sounding cs(static_cast<float>(s.z));
    // Placeholder depth-dependent errors (see header): stds squared into the
    // variances the Sounding contract carries.
    const double d = std::abs(s.z);
    const double v_std = 0.1 + 0.007 * d;
    const double h_std = 0.2 + 0.01 * d;
    cs.vertical_error = static_cast<float>(v_std * v_std);
    cs.horizontal_error = static_cast<float>(h_std * h_std);
    // CUBE-settled backscatter (ADR-0007): the intensity rides the
    // hypothesis queue bound to its depth and comes back per node.
    cs.intensity = s.intensity;

    const double radius = params.influenceRadius(cs);
    int min_x = static_cast<int>(std::floor(((s.x - radius) - out.origin_x) / cell_m));
    int max_x = static_cast<int>(std::ceil(((s.x + radius) - out.origin_x) / cell_m));
    int min_y = static_cast<int>(std::floor(((s.y - radius) - out.origin_y) / cell_m));
    int max_y = static_cast<int>(std::ceil(((s.y + radius) - out.origin_y) / cell_m));
    min_x = std::max(0, min_x);
    max_x = std::min(out.nx, max_x);
    min_y = std::max(0, min_y);
    max_y = std::min(out.ny, max_y);
    const double radius_squared = radius * radius;
    for (int y = min_y; y < max_y; ++y) {
      for (int x = min_x; x < max_x; ++x) {
        const double node_x = out.origin_x + x * cell_m;
        const double node_y = out.origin_y + y * cell_m;
        const double dist2 = (node_x - s.x) * (node_x - s.x) +
          (node_y - s.y) * (node_y - s.y);
        if (dist2 < radius_squared) {
          auto & node = nodes[static_cast<std::size_t>(y) * out.nx + x];
          if (!node) {
            node = std::make_unique<cube::Node>();
          }
          node->insert(std::sqrt(dist2), cs, params);
        }
      }
    }
  }

  out.depth.reserve(n_nodes);
  out.uncertainty.reserve(n_nodes);
  out.intensity.reserve(n_nodes);
  std::size_t estimated = 0;
  for (auto & node : nodes) {
    if (!node) {
      out.depth.push_back(std::nanf(""));
      out.uncertainty.push_back(std::nanf(""));
      out.intensity.push_back(std::nanf(""));
      continue;
    }
    node->queueFlush(params);
    const auto rec = node->extractNodeRecord(params);
    out.depth.push_back(rec.depth);
    out.uncertainty.push_back(rec.depth_var);
    out.intensity.push_back(rec.intensity);
    if (std::isfinite(rec.depth)) {
      ++estimated;
    }
  }
  out.note = std::to_string(estimated) + " of " +
    std::to_string(n_nodes) + " nodes estimated";
  return out;
}

CubeSurfaceMesh build_cube_mesh(
  const CubeSurface & surface, CubeShade shade,
  const std::vector<marine_colormap::Rgba8> & lut)
{
  CubeSurfaceMesh mesh;
  if (!surface.ok() || lut.empty()) {
    return mesh;
  }
  const auto & scalar =
    (shade == CubeShade::Depth) ? surface.depth :
    (shade == CubeShade::Uncertainty) ? surface.uncertainty : surface.intensity;

  // Colour-ramp range from the chosen scalar's finite values (nodes lacking
  // that scalar — e.g. no intensity reported — draw at the ramp's bottom).
  float lo = std::numeric_limits<float>::max();
  float hi = std::numeric_limits<float>::lowest();
  for (std::size_t i = 0; i < scalar.size(); ++i) {
    if (std::isfinite(surface.depth[i]) && std::isfinite(scalar[i])) {
      lo = std::min(lo, scalar[i]);
      hi = std::max(hi, scalar[i]);
    }
  }
  if (!(hi >= lo)) {
    lo = 0.0f;
    hi = 1.0f;
  }
  mesh.scalar_lo = lo;
  mesh.scalar_hi = hi;
  const float span = (hi > lo) ? (hi - lo) : 1.0f;

  // Vertices: one per estimated node; grid index -> compact vertex index.
  const std::size_t n_nodes =
    static_cast<std::size_t>(surface.nx) * static_cast<std::size_t>(surface.ny);
  std::vector<std::int32_t> vertex_of(n_nodes, -1);
  for (int y = 0; y < surface.ny; ++y) {
    for (int x = 0; x < surface.nx; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * surface.nx + x;
      if (!std::isfinite(surface.depth[i])) {
        continue;
      }
      vertex_of[i] = static_cast<std::int32_t>(mesh.positions.size() / 3);
      mesh.positions.push_back(
        static_cast<float>(surface.origin_x + x * surface.cell_m));
      mesh.positions.push_back(
        static_cast<float>(surface.origin_y + y * surface.cell_m));
      mesh.positions.push_back(surface.depth[i]);
      const float v = std::isfinite(scalar[i]) ? scalar[i] : lo;
      const float t = std::clamp((v - lo) / span, 0.0f, 1.0f);
      const auto & c = lut[static_cast<std::size_t>(
            t * static_cast<float>(lut.size() - 1) + 0.5f)];
      mesh.colors.push_back(c.r / 255.0f);
      mesh.colors.push_back(c.g / 255.0f);
      mesh.colors.push_back(c.b / 255.0f);
    }
  }

  // Two triangles per fully-estimated cell quad.
  for (int y = 0; y + 1 < surface.ny; ++y) {
    for (int x = 0; x + 1 < surface.nx; ++x) {
      const std::size_t i00 = static_cast<std::size_t>(y) * surface.nx + x;
      const std::size_t i10 = i00 + 1;
      const std::size_t i01 = i00 + static_cast<std::size_t>(surface.nx);
      const std::size_t i11 = i01 + 1;
      if (vertex_of[i00] < 0 || vertex_of[i10] < 0 ||
        vertex_of[i01] < 0 || vertex_of[i11] < 0)
      {
        continue;
      }
      mesh.indices.push_back(static_cast<std::uint32_t>(vertex_of[i00]));
      mesh.indices.push_back(static_cast<std::uint32_t>(vertex_of[i10]));
      mesh.indices.push_back(static_cast<std::uint32_t>(vertex_of[i11]));
      mesh.indices.push_back(static_cast<std::uint32_t>(vertex_of[i00]));
      mesh.indices.push_back(static_cast<std::uint32_t>(vertex_of[i11]));
      mesh.indices.push_back(static_cast<std::uint32_t>(vertex_of[i01]));
    }
  }
  return mesh;
}

}  // namespace marine_perception_tools
