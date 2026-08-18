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
void drapePing(
  const CubeSurface & surface, const WindowPing & ping, double half_width_m,
  SidescanDrape & out)
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
    // Nearest sample, no averaging; the smaller slant range wins a conflict
    // (better across-track resolution near nadir).
    for (const double o : offsets) {
      const auto cell = cellAt(surface, px + head_x * o, py + head_y * o);
      if (cell < 0 ||
        !std::isfinite(surface.depth[static_cast<std::size_t>(cell)]))
      {
        continue;
      }
      const auto ci = static_cast<std::size_t>(cell);
      if (!std::isfinite(out.amplitude[ci]) || slant < out.painted_slant[ci]) {
        out.amplitude[ci] = amplitude;
        out.painted_slant[ci] = static_cast<float>(slant);
        out.shadow[ci] = 0;
      }
    }
  }
  ++out.pings_used;
}

}  // namespace

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
  out.painted_slant.assign(n, std::numeric_limits<float>::max());

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
    drapePing(surface, pings[i], half_width, out);
  }
  return out;
}

}  // namespace marine_perception_tools
