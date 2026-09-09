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
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "basemap_contrast.hpp"
#include "cube_bathymetry/angular_response_curve.h"
#include "cube_bathymetry/node.h"
#include "cube_bathymetry/parameters.h"
#include "cube_bathymetry/sizes.h"
#include "cube_bathymetry/sounding.h"
#include "sounding_uncertainty.hpp"

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
  // The library stores the SQUARED budget (setIHOLimits squares both on the
  // way in); the operator edits the un-squared metres and fraction of depth.
  t.iho_fixed = static_cast<float>(std::sqrt(params.iho_fixed));
  t.iho_percent = static_cast<float>(std::sqrt(params.iho_percent));
  return t;
}

namespace
{
// The preset table, in the order the dropdown offers them. Values are S-44's
// (and the library's) un-squared budget: metres, then fraction of depth.
// order1a and order1b share this pair in cube::Parameters::setIHOLimits, so
// they are ONE entry here (see iho_preset_names in the header).
const std::vector<std::pair<std::string, std::pair<float, float>>> & presets()
{
  static const std::vector<std::pair<std::string, std::pair<float, float>>> kP{
    {"exclusive", {0.15f, 0.0075f}},
    {"special", {0.25f, 0.0075f}},
    {"order1a/1b", {0.5f, 0.013f}},
    {"order2", {1.0f, 0.023f}}};
  return kP;
}
}  // namespace

std::vector<std::string> iho_preset_names()
{
  std::vector<std::string> names;
  names.reserve(presets().size());
  for (const auto & p : presets()) {
    names.push_back(p.first);
  }
  return names;
}

std::optional<std::pair<float, float>> iho_preset_limits(const std::string & name)
{
  for (const auto & p : presets()) {
    if (p.first == name) {
      return p.second;
    }
  }
  // The library's own vocabulary still resolves, so a caller (or a saved
  // session) naming "order1a"/"order1b" gets the budget it asked for rather
  // than silently falling through to custom.
  if (name == "order1a" || name == "order1b") {
    return std::pair<float, float>{0.5f, 0.013f};
  }
  return std::nullopt;
}

std::string iho_order_for_limits(float iho_fixed, float iho_percent)
{
  // Tolerance is a display-precision epsilon, not a physical one: the spins
  // hand back the same values the presets seeded, so this only absorbs the
  // float round trip.
  constexpr float kEps = 1e-6f;
  for (const auto & p : presets()) {
    if (std::fabs(iho_fixed - p.second.first) <= kEps &&
      std::fabs(iho_percent - p.second.second) <= kEps)
    {
      return p.first;
    }
  }
  return kCustomIhoOrder;
}

std::string derive_box_curve(
  const std::vector<MbesSounding> & soundings, double absorption_db_per_m,
  const std::string & csv_path)
{
  constexpr double kBinDeg = 2.0;
  constexpr std::size_t kMinPerBin = 200;   // sparse bins are noise, drop them
  struct Bin
  {
    double sum = 0.0;
    std::size_t n = 0;
  };
  std::vector<Bin> bins(46);   // 0..90 degrees in 2-degree bins
  for (const auto & s : soundings) {
    if (!std::isfinite(s.intensity) || !std::isfinite(s.beam_angle) ||
      !std::isfinite(s.slant_range) || s.slant_range <= 0.0f)
    {
      continue;
    }
    const double angle_deg =
      std::abs(static_cast<double>(s.beam_angle)) * 180.0 / M_PI;
    const auto b = static_cast<std::size_t>(angle_deg / kBinDeg);
    if (b >= bins.size()) {
      continue;
    }
    // TL-removed sample (the tier-2 convention), so the derived residual
    // composes with the same TL add-back run_cube applies.
    const double r = static_cast<double>(s.slant_range);
    bins[b].sum += static_cast<double>(s.intensity) +
      40.0 * std::log10(r) + 2.0 * absorption_db_per_m * r;
    bins[b].n += 1;
  }
  // Reference = the most-nadir populated bin; residual = bin mean - reference.
  std::size_t ref = bins.size();
  for (std::size_t b = 0; b < bins.size(); ++b) {
    if (bins[b].n >= kMinPerBin) {
      ref = b;
      break;
    }
  }
  if (ref >= bins.size()) {
    return "too few beams per angle bin to self-calibrate";
  }
  std::size_t populated = 0;
  for (const auto & b : bins) {
    if (b.n >= kMinPerBin) {
      ++populated;
    }
  }
  if (populated < 3) {
    return "not enough angular spread to self-calibrate (need >= 3 bins)";
  }
  const double nadir_mean = bins[ref].sum / static_cast<double>(bins[ref].n);
  std::ofstream out(csv_path);
  if (!out) {
    return "could not write " + csv_path;
  }
  out << "# Box self-calibrated angular response (survey explorer CUBE lab, "
    "mpt#27)\n";
  out << "# tl_removed: true\n";
  out << "# absorption_db_per_m: " << absorption_db_per_m << "\n";
  out << "abs_angle_deg_center,mean_bs_db,n,db_relative_to_nadir\n";
  for (std::size_t b = 0; b < bins.size(); ++b) {
    if (bins[b].n < kMinPerBin) {
      continue;
    }
    const double mean = bins[b].sum / static_cast<double>(bins[b].n);
    out << (b * kBinDeg + 0.5 * kBinDeg) << "," << mean << "," << bins[b].n
        << "," << (mean - nadir_mean) << "\n";
  }
  return {};
}

CubeSurface run_cube(
  const std::vector<MbesSounding> & soundings, double cell_m,
  const CubeTuning & tuning,
  const std::shared_ptr<std::atomic<bool>> & cancel)
{
  const auto stop = [&cancel]() {
      return cancel && cancel->load(std::memory_order_relaxed);
    };
  // A cancelled run yields no surface at all (#44): half a grid of hypotheses
  // is not a coarser estimate, it is an unfinished one.
  const auto abandon = [&soundings]() {
      CubeSurface c;
      c.soundings_in = soundings.size();
      c.note = "cancelled";
      return c;
    };
  if (stop()) {return abandon();}
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
  // The operator-owned grid guard (tuning.max_nodes, editable in the params
  // dialog — no hidden policy). The UI's run confirmation offers a one-shot
  // override before ever reaching this.
  if (n_nodes > tuning.max_nodes) {
    out.note = "grid would be " + std::to_string(out.nx) + "x" +
      std::to_string(out.ny) + " = " + std::to_string(n_nodes) +
      " nodes, over the max-nodes limit (" + std::to_string(tuning.max_nodes) +
      ", editable in params…)";
    out.nx = 0;
    out.ny = 0;
    return out;
  }

  const cube::CellSizes sizes(static_cast<float>(cell_m));   // square cells
  cube::Parameters params(sizes);
  // Operator tuning (#27): the exposed subset only — the derived scales
  // (distance_scale etc.) stay as the ctor computed them from the cell size.
  params.capture_distance_scale = tuning.capture_distance_scale;
  // The uncertainty budget is the operator's two numbers, squared into the
  // library's storage convention (#45). Overwriting what the ctor's order name
  // seeded is the point: a run can only ever use the current values.
  params.iho_fixed = static_cast<double>(tuning.iho_fixed) * tuning.iho_fixed;
  params.iho_percent =
    static_cast<double>(tuning.iho_percent) * tuning.iho_percent;
  params.iho_order = iho_order_for_limits(tuning.iho_fixed, tuning.iho_percent);
  params.median_length = std::max<std::uint32_t>(1, tuning.median_length);
  params.quotient_limit = tuning.quotient_limit;
  params.discount = tuning.discount;
  params.estimate_offset = tuning.estimate_offset;
  params.bayes_factor_threshold = tuning.bayes_factor_threshold;
  params.runlength_threshold = tuning.runlength_threshold;
  params.extractor = static_cast<cube::CubeExtractor>(
    std::clamp(tuning.extractor, 0, static_cast<int>(cube::CUBE_POSTERIOR)));

  // ARA correction (cube#81): the per-sonar curve makes the CUBE-settled
  // backscatter angle-independent (removes the bright-nadir bias). Applied
  // at record time inside the library (recordBeam -> correctBeamIntensity)
  // using each sounding's rx angle; the tier-2 TL terms come from the CSV
  // header (cube#87). A load failure is reported, not fatal.
  std::string ara_note;
  if (!tuning.ara_curve_path.empty()) {
    try {
      const auto curve =
        cube::loadAngularResponseCurveWithHeader(tuning.ara_curve_path);
      if (curve.points.empty()) {
        ara_note = "; ARA curve empty/unreadable — correction off";
      } else {
        params.angular_response_curve = curve.points;
        params.backscatter_angle_correction =
          cube::BackscatterAngleCorrection::Empirical;
        params.backscatter_tl_removed = curve.tl_removed;
        params.backscatter_absorption_db_per_m = curve.absorption_db_per_m;
        ara_note = "; ARA corrected (" +
          std::to_string(curve.points.size()) + " bins" +
          (curve.tl_removed ? ", tier-2 TL" : "") + ")";
      }
    } catch (const std::exception & e) {
      ara_note = std::string("; ARA curve load failed: ") + e.what();
    }
  }

  // Drive cube::Node directly instead of cube::Grid: Grid::values() is the
  // depth-only legacy extraction, while Node::extractNodeRecord carries the
  // CUBE-settled backscatter (ADR-0007) this lab drapes with. The sounding
  // spread mirrors Grid::insert's effect square exactly (influence radius,
  // node-centre distance test).
  std::vector<std::unique_ptr<cube::Node>> nodes(n_nodes);
  std::size_t dropped_no_geometry = 0;   // beams with no usable angle/range
  for (const auto & s : soundings) {
    if (stop()) {return abandon();}
    // Angle-aware placeholder errors (#49, see sounding_uncertainty.hpp):
    // first-order propagation through this beam's own angle and slant range,
    // so the estimator can prefer a pass's near-nadir coverage over another
    // pass's outer beams. Already variances — the contract cube::Sounding
    // carries — so they are stored as computed, not squared again.
    // A beam whose geometry is missing or non-finite is DROPPED: no sounding
    // is better than one carrying a fabricated confidence.
    SoundingUncertainty u;
    if (!sounding_uncertainty(s.beam_angle, s.slant_range, &u)) {
      ++dropped_no_geometry;
      continue;
    }
    // World z is up (seabed negative) — the cube depth convention directly.
    cube::Sounding cs(static_cast<float>(s.z));
    cs.vertical_error = static_cast<float>(u.vertical_variance);
    cs.horizontal_error = static_cast<float>(u.horizontal_variance);
    // CUBE-settled backscatter (ADR-0007): the intensity rides the
    // hypothesis queue bound to its depth and comes back per node, with
    // its beam angle + slant range so the ARA/TL corrections can act.
    cs.intensity = s.intensity;
    cs.beam_angle = s.beam_angle;
    cs.slant_range = s.slant_range;

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
    if (stop()) {return abandon();}
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
    std::to_string(n_nodes) + " nodes estimated" + ara_note;
  if (dropped_no_geometry > 0) {
    // Never silent: a beam without angle/slant range gets no uncertainty and
    // so is not inserted at all (#49).
    out.note += "; " + std::to_string(dropped_no_geometry) +
      " sounding(s) skipped — no beam geometry";
  }
  return out;
}

CubeSurfaceMesh build_cube_mesh_colored(
  const CubeSurface & surface, const std::vector<float> & node_rgb,
  CubeMeshStyle style)
{
  CubeSurfaceMesh mesh;
  const std::size_t n_nodes =
    static_cast<std::size_t>(surface.nx) * static_cast<std::size_t>(surface.ny);
  if (!surface.ok() || node_rgb.size() < n_nodes * 3) {
    return mesh;
  }

  // Crisp smooth (the default): every node is one constant-colour quad —
  // no texture blending, each CUBE cell a single crisp texel — but the
  // quad's corners take the MEAN height of the (up to four) adjacent
  // estimated nodes, so neighbouring quads share corner heights exactly
  // and the relief is a watertight smooth membrane.
  if (style == CubeMeshStyle::CrispSmooth) {
    const float half = static_cast<float>(0.5 * surface.cell_m);
    // Corner lattice (nx+1) x (ny+1): corner (cx, cy) touches nodes
    // (cx-1..cx, cy-1..cy); its height is their finite-depth mean.
    const int cnx = surface.nx + 1;
    const int cny = surface.ny + 1;
    std::vector<float> corner_z(
      static_cast<std::size_t>(cnx) * static_cast<std::size_t>(cny),
      std::nanf(""));
    for (int cy = 0; cy < cny; ++cy) {
      for (int cx = 0; cx < cnx; ++cx) {
        float sum = 0.0f;
        int cnt = 0;
        for (int dy = -1; dy <= 0; ++dy) {
          for (int dx = -1; dx <= 0; ++dx) {
            const int nx_i = cx + dx;
            const int ny_i = cy + dy;
            if (nx_i < 0 || ny_i < 0 || nx_i >= surface.nx || ny_i >= surface.ny) {
              continue;
            }
            const float d =
              surface.depth[static_cast<std::size_t>(ny_i) * surface.nx + nx_i];
            if (std::isfinite(d)) {
              sum += d;
              ++cnt;
            }
          }
        }
        if (cnt > 0) {
          corner_z[static_cast<std::size_t>(cy) * cnx + cx] =
            sum / static_cast<float>(cnt);
        }
      }
    }
    for (int y = 0; y < surface.ny; ++y) {
      for (int x = 0; x < surface.nx; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * surface.nx + x;
        if (!std::isfinite(surface.depth[i])) {
          continue;
        }
        const float cx0 = static_cast<float>(
          surface.origin_x + x * surface.cell_m);
        const float cy0 = static_cast<float>(
          surface.origin_y + y * surface.cell_m);
        // Quad corners on the corner lattice: (x, y), (x+1, y), (x+1, y+1),
        // (x, y+1) — an estimated node guarantees all four have heights.
        const float zs[4] = {
          corner_z[static_cast<std::size_t>(y) * cnx + x],
          corner_z[static_cast<std::size_t>(y) * cnx + x + 1],
          corner_z[static_cast<std::size_t>(y + 1) * cnx + x + 1],
          corner_z[static_cast<std::size_t>(y + 1) * cnx + x]};
        const float xs[4] = {cx0 - half, cx0 + half, cx0 + half, cx0 - half};
        const float ys[4] = {cy0 - half, cy0 - half, cy0 + half, cy0 + half};
        const auto base =
          static_cast<std::uint32_t>(mesh.positions.size() / 3);
        for (int k = 0; k < 4; ++k) {
          mesh.positions.push_back(xs[k]);
          mesh.positions.push_back(ys[k]);
          mesh.positions.push_back(zs[k]);
          mesh.colors.push_back(node_rgb[i * 3]);
          mesh.colors.push_back(node_rgb[i * 3 + 1]);
          mesh.colors.push_back(node_rgb[i * 3 + 2]);
        }
        mesh.indices.push_back(base);
        mesh.indices.push_back(base + 1);
        mesh.indices.push_back(base + 2);
        mesh.indices.push_back(base);
        mesh.indices.push_back(base + 2);
        mesh.indices.push_back(base + 3);
      }
    }
    return mesh;
  }

  // Stepped cells: each estimated node is one flat quad at its own depth
  // and colour — a CUBE cell reads as a single crisp "pixel", stepped
  // plateaus between neighbours. Unestimated nodes stay holes.
  if (style == CubeMeshStyle::CrispStepped) {
    const float half = static_cast<float>(0.5 * surface.cell_m);
    for (int y = 0; y < surface.ny; ++y) {
      for (int x = 0; x < surface.nx; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * surface.nx + x;
        if (!std::isfinite(surface.depth[i])) {
          continue;
        }
        const float cx = static_cast<float>(surface.origin_x + x * surface.cell_m);
        const float cy = static_cast<float>(surface.origin_y + y * surface.cell_m);
        const float cz = surface.depth[i];
        const auto base =
          static_cast<std::uint32_t>(mesh.positions.size() / 3);
        const float xs[4] = {cx - half, cx + half, cx + half, cx - half};
        const float ys[4] = {cy - half, cy - half, cy + half, cy + half};
        for (int k = 0; k < 4; ++k) {
          mesh.positions.push_back(xs[k]);
          mesh.positions.push_back(ys[k]);
          mesh.positions.push_back(cz);
          mesh.colors.push_back(node_rgb[i * 3]);
          mesh.colors.push_back(node_rgb[i * 3 + 1]);
          mesh.colors.push_back(node_rgb[i * 3 + 2]);
        }
        mesh.indices.push_back(base);
        mesh.indices.push_back(base + 1);
        mesh.indices.push_back(base + 2);
        mesh.indices.push_back(base);
        mesh.indices.push_back(base + 2);
        mesh.indices.push_back(base + 3);
      }
    }
    return mesh;
  }

  // Smooth relief: one vertex per estimated node; grid index -> compact
  // vertex index; two triangles per fully-estimated cell quad.
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
      mesh.colors.push_back(node_rgb[i * 3]);
      mesh.colors.push_back(node_rgb[i * 3 + 1]);
      mesh.colors.push_back(node_rgb[i * 3 + 2]);
    }
  }
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

CubeSurfaceMesh build_cube_mesh(
  const CubeSurface & surface, CubeShade shade,
  const std::vector<marine_colormap::Rgba8> & lut, CubeMeshStyle style,
  const std::optional<std::pair<float, float>> & range)
{
  if (!surface.ok() || lut.empty()) {
    return CubeSurfaceMesh{};
  }
  const auto & scalar =
    (shade == CubeShade::Depth) ? surface.depth :
    (shade == CubeShade::Uncertainty) ? surface.uncertainty : surface.intensity;

  // Colour-ramp range from the chosen scalar's finite values (nodes lacking
  // that scalar — e.g. no intensity reported — draw at the ramp's bottom).
  float lo = std::numeric_limits<float>::max();
  float hi = std::numeric_limits<float>::lowest();
  if (range) {
    lo = range->first;
    hi = range->second;
  } else {
    // Robust percentile contrast (the stores' convention, basemap_contrast):
    // a raw min/max ramp lets a few outlier cells own the whole scale, which
    // is exactly why the backscatter shade looked washed out next to the
    // store imagery.
    std::vector<double> samples;
    samples.reserve(scalar.size());
    for (std::size_t i = 0; i < scalar.size(); ++i) {
      if (std::isfinite(surface.depth[i]) && std::isfinite(scalar[i])) {
        samples.push_back(scalar[i]);
      }
    }
    const auto [rlo, rhi] = robust_range(samples);
    lo = static_cast<float>(rlo);
    hi = static_cast<float>(rhi);
  }
  if (!(hi >= lo)) {
    lo = 0.0f;
    hi = 1.0f;
  }
  const float span = (hi > lo) ? (hi - lo) : 1.0f;

  const std::size_t n_nodes =
    static_cast<std::size_t>(surface.nx) * static_cast<std::size_t>(surface.ny);
  std::vector<float> node_rgb(n_nodes * 3, 0.0f);
  for (std::size_t i = 0; i < n_nodes; ++i) {
    if (!std::isfinite(surface.depth[i])) {
      continue;
    }
    const float v = std::isfinite(scalar[i]) ? scalar[i] : lo;
    const float t = std::clamp((v - lo) / span, 0.0f, 1.0f);
    const auto & c = lut[static_cast<std::size_t>(
          t * static_cast<float>(lut.size() - 1) + 0.5f)];
    node_rgb[i * 3] = c.r / 255.0f;
    node_rgb[i * 3 + 1] = c.g / 255.0f;
    node_rgb[i * 3 + 2] = c.b / 255.0f;
  }
  auto mesh = build_cube_mesh_colored(surface, node_rgb, style);
  mesh.scalar_lo = lo;
  mesh.scalar_hi = hi;
  return mesh;
}

}  // namespace marine_perception_tools
