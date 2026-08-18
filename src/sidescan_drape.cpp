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

#include "sidescan_drape.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "sidescan_geometry.hpp"

namespace marine_perception_tools
{

namespace
{

// Grid index of the cell nearest a world position; -1 outside the grid.
inline std::ptrdiff_t cellAt(const CubeSurface & s, double x, double y)
{
  const auto cx = static_cast<std::ptrdiff_t>(
    std::llround((x - s.origin_x) / s.cell_m));
  const auto cy = static_cast<std::ptrdiff_t>(
    std::llround((y - s.origin_y) / s.cell_m));
  if (cx < 0 || cy < 0 || cx >= s.nx || cy >= s.ny) {
    return -1;
  }
  return cy * s.nx + cx;
}

// Estimated surface z at/near the position: the nearest cell, else a small
// ring search (the nadir anchor tolerates the gap under the towfish track).
inline bool surfaceZNear(
  const CubeSurface & s, double x, double y, int max_ring, double & z_out)
{
  const auto centre = cellAt(s, x, y);
  if (centre >= 0 && std::isfinite(s.depth[static_cast<std::size_t>(centre)])) {
    z_out = s.depth[static_cast<std::size_t>(centre)];
    return true;
  }
  for (int ring = 1; ring <= max_ring; ++ring) {
    for (int dy = -ring; dy <= ring; ++dy) {
      for (int dx = -ring; dx <= ring; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != ring) {
          continue;   // ring perimeter only
        }
        const auto c = cellAt(
          s, x + dx * s.cell_m, y + dy * s.cell_m);
        if (c >= 0 && std::isfinite(s.depth[static_cast<std::size_t>(c)])) {
          z_out = s.depth[static_cast<std::size_t>(c)];
          return true;
        }
      }
    }
  }
  return false;
}

// `half_width_m`: half the along-track strip this ping paints — the ping's
// share of the pass footprint (half the ping-to-ping spacing), so
// consecutive pings tile the grid without grey gaps between their rays.
// `ping_score` (0..1] is the ping's straightness quality; the per-cell
// score is ping_score x range-closeness, higher wins.
void drapePing(
  const CubeSurface & surface, const WindowPing & ping, double half_width_m,
  double ping_score, SidescanDrape & out)
{
  const PingGeometry & g = ping.geometry;
  if (g.metres_per_sample <= 0.0 || g.lateral_sign == 0 ||
    g.altitude <= 0.0 || ping.amplitudes.empty())
  {
    ++out.pings_skipped;
    return;
  }
  // Sensor z anchored to the CUBE surface under the track: nadir surface z
  // plus the sensor's measured height above bottom.
  double nadir_z = 0.0;
  if (!surfaceZNear(surface, g.sensor_x, g.sensor_y, 3, nadir_z)) {
    ++out.pings_skipped;   // ping is off the estimated surface entirely
    return;
  }
  const double sensor_z = nadir_z + g.altitude;

  // Across-track direction (project_sample's convention: port = left of
  // travel, yaw CCW from +x).
  const double dir_x = g.lateral_sign * -std::sin(g.yaw);
  const double dir_y = g.lateral_sign * std::cos(g.yaw);

  const double max_slant = slant_range_at(
    ping.amplitudes.size() - 1, g.sample0, g.metres_per_sample);
  if (max_slant <= 0.0) {
    ++out.pings_skipped;
    return;
  }

  // Along-track paint offsets: the strip of cells this ping owns. The
  // heading direction is perpendicular to the across-track march.
  const double head_x = std::cos(g.yaw);
  const double head_y = std::sin(g.yaw);
  std::vector<double> offsets{0.0};
  const double o_step = 0.5 * surface.cell_m;
  for (double o = o_step; o <= half_width_m; o += o_step) {
    offsets.push_back(o);
    offsets.push_back(-o);
  }

  // First-return march: outward from nadir in half-cell steps, tracking the
  // highest line-of-sight angle seen so far along the CENTRAL ray. A cell
  // whose ray falls below that line is in acoustic shadow. Terrain gaps
  // (unestimated cells) keep the previous z so the occlusion state stays
  // sane, but are never painted. Each step paints its along-track strip
  // with the same sample (the ping's footprint share).
  const double step = 0.5 * surface.cell_m;
  double phi_max = -std::numeric_limits<double>::infinity();
  double last_z = nadir_z;
  for (double t = step; t <= max_slant; t += step) {
    const double px = g.sensor_x + dir_x * t;
    const double py = g.sensor_y + dir_y * t;
    const auto centre = cellAt(surface, px, py);
    const bool centre_estimated = centre >= 0 &&
      std::isfinite(surface.depth[static_cast<std::size_t>(centre)]);
    const double z = centre_estimated ?
      static_cast<double>(surface.depth[static_cast<std::size_t>(centre)]) : last_z;
    last_z = z;

    const double dz = sensor_z - z;   // vertical drop to the cell (>0 below)
    if (dz <= 0.0) {
      // Terrain at/above the sensor: everything beyond is blocked.
      break;
    }
    const double phi = std::atan2(z - sensor_z, t);   // elevation (negative down)
    const bool visible = phi >= phi_max;
    phi_max = std::max(phi_max, phi);

    if (!visible) {
      // Acoustic shadow: mark the strip unless a nearer ping painted it.
      for (const double o : offsets) {
        const auto cell = cellAt(
          surface, px + head_x * o, py + head_y * o);
        if (cell < 0 ||
          !std::isfinite(surface.depth[static_cast<std::size_t>(cell)]))
        {
          continue;
        }
        const auto ci = static_cast<std::size_t>(cell);
        if (!std::isfinite(out.amplitude[ci])) {
          out.shadow[ci] = 1;
        }
      }
      continue;
    }
    const double slant = std::sqrt(t * t + dz * dz);
    const double sample_f =
      slant / g.metres_per_sample - static_cast<double>(g.sample0);
    const auto sample_i = static_cast<std::ptrdiff_t>(std::llround(sample_f));
    if (sample_i < 0 ||
      sample_i >= static_cast<std::ptrdiff_t>(ping.amplitudes.size()))
    {
      continue;   // beyond the recording (or inside the pre-gate)
    }
    const float amplitude = ping.amplitudes[static_cast<std::size_t>(sample_i)];
    // Nearest sample, no averaging; the QUALITY score decides conflicts:
    // straightness x range-closeness, so a straight-running near sample
    // beats a mid-turn or far-edge one (the composite's "better pixels
    // through" rule; single-pass conflicts reduce to nearer-wins).
    const double range_score =
      std::max(0.05, 1.0 - slant / std::max(1e-6, max_slant));
    const float score = static_cast<float>(ping_score * range_score);
    for (const double o : offsets) {
      const auto cell = cellAt(surface, px + head_x * o, py + head_y * o);
      if (cell < 0 ||
        !std::isfinite(surface.depth[static_cast<std::size_t>(cell)]))
      {
        continue;
      }
      const auto ci = static_cast<std::size_t>(cell);
      if (!std::isfinite(out.amplitude[ci]) || score > out.painted_score[ci]) {
        out.amplitude[ci] = amplitude;
        out.painted_score[ci] = score;
        out.shadow[ci] = 0;
      }
    }
  }
  ++out.pings_used;
}

}  // namespace

CubeSurface extend_surface_for_drape(
  const CubeSurface & surface, const std::vector<WindowPing> & pings,
  std::uint64_t max_nodes, std::string & note)
{
  if (!surface.ok()) {
    return surface;
  }
  // Desired bounds: the old grid plus every usable ping's across-track
  // reach (sensor -> outer endpoint, with a small along-track margin).
  double min_x = surface.origin_x;
  double min_y = surface.origin_y;
  double max_x = surface.origin_x + (surface.nx - 1) * surface.cell_m;
  double max_y = surface.origin_y + (surface.ny - 1) * surface.cell_m;
  const double margin = 2.0;
  bool any = false;
  for (const auto & p : pings) {
    const auto & g = p.geometry;
    if (g.metres_per_sample <= 0.0 || g.lateral_sign == 0 ||
      p.amplitudes.empty())
    {
      continue;
    }
    const double reach = slant_range_at(
      p.amplitudes.size() - 1, g.sample0, g.metres_per_sample);
    const double ex = g.sensor_x + g.lateral_sign * -std::sin(g.yaw) * reach;
    const double ey = g.sensor_y + g.lateral_sign * std::cos(g.yaw) * reach;
    min_x = std::min({min_x, g.sensor_x - margin, ex - margin});
    max_x = std::max({max_x, g.sensor_x + margin, ex + margin});
    min_y = std::min({min_y, g.sensor_y - margin, ey - margin});
    max_y = std::max({max_y, g.sensor_y + margin, ey + margin});
    any = true;
  }
  if (!any) {
    return surface;
  }

  // Grow in WHOLE cells so old and new node lattices align exactly.
  auto cellsBelow = [&](double lo, double origin) {
      return std::max<std::ptrdiff_t>(
        0, static_cast<std::ptrdiff_t>(std::ceil((origin - lo) / surface.cell_m)));
    };
  std::ptrdiff_t grow_left = cellsBelow(min_x, surface.origin_x);
  std::ptrdiff_t grow_down = cellsBelow(min_y, surface.origin_y);
  std::ptrdiff_t grow_right = std::max<std::ptrdiff_t>(
    0, static_cast<std::ptrdiff_t>(std::ceil(
      (max_x - (surface.origin_x + (surface.nx - 1) * surface.cell_m)) /
      surface.cell_m)));
  std::ptrdiff_t grow_up = std::max<std::ptrdiff_t>(
    0, static_cast<std::ptrdiff_t>(std::ceil(
      (max_y - (surface.origin_y + (surface.ny - 1) * surface.cell_m)) /
      surface.cell_m)));

  // The operator's grid guard caps the growth: shrink all sides by the same
  // factor until the node count fits, and say so.
  for (int guard = 0; guard < 64; ++guard) {
    const auto nx = static_cast<std::uint64_t>(surface.nx) + grow_left + grow_right;
    const auto ny = static_cast<std::uint64_t>(surface.ny) + grow_down + grow_up;
    if (nx * ny <= max_nodes) {
      break;
    }
    grow_left = grow_left * 3 / 4;
    grow_right = grow_right * 3 / 4;
    grow_down = grow_down * 3 / 4;
    grow_up = grow_up * 3 / 4;
    if (note.empty()) {
      note = "drape terrain clipped by the max-nodes limit";
    }
    if (grow_left + grow_right + grow_down + grow_up == 0) {
      break;
    }
  }

  CubeSurface ext;
  ext.cell_m = surface.cell_m;
  ext.origin_x = surface.origin_x - grow_left * surface.cell_m;
  ext.origin_y = surface.origin_y - grow_down * surface.cell_m;
  ext.nx = surface.nx + static_cast<int>(grow_left + grow_right);
  ext.ny = surface.ny + static_cast<int>(grow_down + grow_up);
  const std::size_t n =
    static_cast<std::size_t>(ext.nx) * static_cast<std::size_t>(ext.ny);
  ext.depth.assign(n, std::nanf(""));
  ext.uncertainty.assign(n, std::nanf(""));   // NaN == interpolated, not measured
  ext.intensity.assign(n, std::nanf(""));
  ext.soundings_in = surface.soundings_in;

  // Copy the measured region into the shifted lattice.
  for (int y = 0; y < surface.ny; ++y) {
    for (int x = 0; x < surface.nx; ++x) {
      const std::size_t src = static_cast<std::size_t>(y) * surface.nx + x;
      const std::size_t dst =
        static_cast<std::size_t>(y + grow_down) * ext.nx + (x + grow_left);
      ext.depth[dst] = surface.depth[src];
      ext.uncertainty[dst] = surface.uncertainty[src];
      ext.intensity[dst] = surface.intensity[src];
    }
  }

  // Membrane fill: multi-source BFS seeds every unmeasured node with its
  // nearest measured depth, then Jacobi relaxation smooths the fill while
  // measured nodes stay pinned. Display-grade interpolation/extrapolation —
  // terrain for the drape to land on, never presented as bathymetry
  // (uncertainty stays NaN on filled nodes).
  std::vector<std::int32_t> frontier;
  std::vector<std::uint8_t> measured(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    if (std::isfinite(ext.depth[i])) {
      measured[i] = 1;
      frontier.push_back(static_cast<std::int32_t>(i));
    }
  }
  if (frontier.empty() || frontier.size() == n) {
    return ext;
  }
  std::vector<std::int32_t> next;
  auto visit = [&](std::int32_t from, std::int32_t to) {
      if (!std::isfinite(ext.depth[static_cast<std::size_t>(to)])) {
        ext.depth[static_cast<std::size_t>(to)] =
          ext.depth[static_cast<std::size_t>(from)];
        next.push_back(to);
      }
    };
  while (!frontier.empty()) {
    next.clear();
    for (const auto i : frontier) {
      const int x = static_cast<int>(i % ext.nx);
      const int y = static_cast<int>(i / ext.nx);
      if (x > 0) {visit(i, i - 1);}
      if (x + 1 < ext.nx) {visit(i, i + 1);}
      if (y > 0) {visit(i, i - ext.nx);}
      if (y + 1 < ext.ny) {visit(i, i + ext.nx);}
    }
    frontier.swap(next);
  }
  std::vector<float> relaxed(ext.depth);
  for (int sweep = 0; sweep < 32; ++sweep) {
    for (std::size_t i = 0; i < n; ++i) {
      if (measured[i]) {
        continue;
      }
      const int x = static_cast<int>(i % ext.nx);
      const int y = static_cast<int>(i / ext.nx);
      float sum = 0.0f;
      int cnt = 0;
      if (x > 0) {sum += ext.depth[i - 1]; ++cnt;}
      if (x + 1 < ext.nx) {sum += ext.depth[i + 1]; ++cnt;}
      if (y > 0) {sum += ext.depth[i - ext.nx]; ++cnt;}
      if (y + 1 < ext.ny) {sum += ext.depth[i + ext.nx]; ++cnt;}
      if (cnt > 0) {
        relaxed[i] = sum / static_cast<float>(cnt);
      }
    }
    ext.depth.swap(relaxed);
  }
  return ext;
}

SidescanDrape drape_pass(
  const CubeSurface & surface, const std::vector<WindowPing> & pings)
{
  SidescanDrape out;
  if (!surface.ok()) {
    return out;
  }
  out.nx = surface.nx;
  out.ny = surface.ny;
  const std::size_t n =
    static_cast<std::size_t>(surface.nx) * static_cast<std::size_t>(surface.ny);
  out.amplitude.assign(n, std::nanf(""));
  out.shadow.assign(n, 0);
  out.painted_score.assign(n, 0.0f);

  // Per-ping along-track strip width: half the larger neighbour spacing (a
  // ping owns the ground up to halfway to its neighbours), floored at one
  // cell so a dense ping rate still tiles, capped so a recording gap or a
  // turn cannot smear one ping across metres of seabed.
  const double kMaxHalfWidth = 2.0;
  for (std::size_t i = 0; i < pings.size(); ++i) {
    double spacing = 0.0;
    const auto & g = pings[i].geometry;
    if (i > 0) {
      const auto & p = pings[i - 1].geometry;
      spacing = std::hypot(g.sensor_x - p.sensor_x, g.sensor_y - p.sensor_y);
    }
    if (i + 1 < pings.size()) {
      const auto & nxt = pings[i + 1].geometry;
      spacing = std::max(
        spacing,
        std::hypot(nxt.sensor_x - g.sensor_x, nxt.sensor_y - g.sensor_y));
    }
    const double half_width = std::min(
      kMaxHalfWidth, std::max(0.5 * spacing, surface.cell_m));

    // Straightness: the yaw rate against the neighbours (rad per metre of
    // track, angle-wrapped). ~3 deg/m halves the score; a pass boundary's
    // position jump drives the rate toward zero, so concatenated passes
    // score independently without explicit boundaries.
    double rate = 0.0;
    const auto yaw_rate = [&g](const PingGeometry & o) {
        const double d =
          std::hypot(g.sensor_x - o.sensor_x, g.sensor_y - o.sensor_y);
        if (d < 1e-6) {return 0.0;}
        const double dy = std::remainder(g.yaw - o.yaw, 2.0 * M_PI);
        return std::abs(dy) / d;
      };
    if (i > 0) {rate = yaw_rate(pings[i - 1].geometry);}
    if (i + 1 < pings.size()) {
      rate = std::max(rate, yaw_rate(pings[i + 1].geometry));
    }
    constexpr double kRateHalf = 0.05;   // rad/m at which the score halves
    const double ping_score =
      1.0 / (1.0 + (rate / kRateHalf) * (rate / kRateHalf));

    drapePing(surface, pings[i], half_width, ping_score, out);
  }
  return out;
}

}  // namespace marine_perception_tools
