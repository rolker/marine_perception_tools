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

#ifndef COASTLINE_DATA_HPP_
#define COASTLINE_DATA_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <istream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace marine_perception_tools
{

// The vendored world coastline (#41): loading, decimation and the scale rule
// that keeps it subordinate. Qt-free and ROS-free so all of it is unit-tested
// without a display (the package's header-only-unit pattern).
//
// The data is ORIENTATION, NOT NAVIGATION: a 1:50m generalised coastline is
// wrong by hundreds of metres to a couple of kilometres at survey scale.
// coastlineFadeAlpha() below is where that is enforced — it must reach zero
// before the operator is at a scale where the real layers answer the question.
// See data/coastline/README.md for the provenance and the file format.

// One coastline polyline: (lat, lon) degrees, plus its bounding box so a
// renderer can cull off-screen lines without walking their points.
struct CoastlinePolyline
{
  std::vector<std::pair<double, double>> points;
  double south = 0.0;
  double west = 0.0;
  double north = 0.0;
  double east = 0.0;
};

struct Coastline
{
  std::vector<CoastlinePolyline> lines;

  bool empty() const {return lines.empty();}
  std::size_t pointCount() const
  {
    std::size_t n = 0;
    for (const auto & line : lines) {
      n += line.points.size();
    }
    return n;
  }
};

// Douglas-Peucker decimation of a (lat, lon) polyline, tolerance in degrees.
// A tolerance of zero (or a polyline too short to thin) returns the input
// unchanged; endpoints are always kept.
//
// Distances are computed in raw degrees — longitude is NOT scaled by cos(lat).
// That overstates east-west deviation away from the equator, so the thinning
// is conservative there (it keeps points it could have dropped), which is the
// safe direction for a display layer.
inline std::vector<std::pair<double, double>> decimatePolyline(
  const std::vector<std::pair<double, double>> & points, double tolerance_deg)
{
  if (points.size() < 3 || !(tolerance_deg > 0.0)) {
    return points;
  }
  std::vector<bool> keep(points.size(), false);
  keep.front() = true;
  keep.back() = true;
  // Iterative (not recursive): a 20k-point polyline must not depend on the
  // stack depth of a pathological split sequence.
  std::vector<std::pair<std::size_t, std::size_t>> pending;
  pending.emplace_back(0, points.size() - 1);
  while (!pending.empty()) {
    const auto [first, last] = pending.back();
    pending.pop_back();
    if (last <= first + 1) {
      continue;
    }
    const double y0 = points[first].first;
    const double x0 = points[first].second;
    const double dy = points[last].first - y0;
    const double dx = points[last].second - x0;
    const double len = std::sqrt(dx * dx + dy * dy);
    double worst = -1.0;
    std::size_t worst_i = first;
    for (std::size_t i = first + 1; i < last; ++i) {
      const double py = points[i].first - y0;
      const double px = points[i].second - x0;
      // Degenerate segment (a closed ring's ends coincide): fall back to the
      // distance from the shared endpoint, which is what the split needs.
      const double d = (len > 0.0) ?
        std::abs(px * dy - py * dx) / len :
        std::sqrt(px * px + py * py);
      if (d > worst) {
        worst = d;
        worst_i = i;
      }
    }
    if (worst > tolerance_deg) {
      keep[worst_i] = true;
      pending.emplace_back(first, worst_i);
      pending.emplace_back(worst_i, last);
    }
  }
  std::vector<std::pair<double, double>> out;
  out.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (keep[i]) {
      out.push_back(points[i]);
    }
  }
  return out;
}

// Ground metres per pixel at which the coastline is drawn at full strength
// (collection-wide / regional zoom, where nothing else tells the operator
// where they are) and at which it has faded to nothing (well above survey
// zoom: at 15 m/px a screenful is kilometres across and the store basemap
// carries the answer). Between them the layer fades logarithmically, so it
// visibly retreats as the operator zooms in rather than switching off.
constexpr double kCoastlineFullScaleMpp = 150.0;
constexpr double kCoastlineFadeFloorMpp = 15.0;

// Opacity multiplier in [0, 1] for the current map scale. Zero means the
// coastline must not be drawn at all: below the floor it is finer than the
// error of the data behind it, and would read as chart detail. A non-finite
// scale is broken view state rather than a zoom level, and also yields zero
// — the layer stays off rather than painting the world over whatever the
// operator is actually looking at.
inline double coastlineFadeAlpha(double ground_metres_per_pixel)
{
  if (!std::isfinite(ground_metres_per_pixel) ||
    ground_metres_per_pixel <= kCoastlineFadeFloorMpp)
  {
    return 0.0;
  }
  if (ground_metres_per_pixel >= kCoastlineFullScaleMpp) {
    return 1.0;
  }
  return std::log(ground_metres_per_pixel / kCoastlineFadeFloorMpp) /
         std::log(kCoastlineFullScaleMpp / kCoastlineFadeFloorMpp);
}

// Parse the vendored coastline format (data/coastline/README.md): comment and
// blank lines are ignored, "> <lat> <lon>" starts a polyline at an absolute
// position in milli-degrees, and any other line is the next point as a delta
// in milli-degrees. A malformed or out-of-range record drops the polyline it
// belongs to and parsing resumes at the next ">", so a damaged or truncated
// file degrades to less coastline rather than to a crash or a smeared map.
// Polylines are decimated with `tolerance_deg` (see decimatePolyline) and
// those left with fewer than two points are dropped.
inline Coastline parseCoastline(std::istream & in, double tolerance_deg = 0.0)
{
  constexpr double kMilli = 0.001;
  constexpr int64_t kMaxLatMilli = 90000;
  constexpr int64_t kMaxLonMilli = 180000;

  Coastline coastline;
  std::vector<std::pair<double, double>> current;
  bool in_polyline = false;   // false: skipping until the next ">"
  int64_t lat_milli = 0;
  int64_t lon_milli = 0;

  const auto flush = [&coastline, &current, tolerance_deg]() {
      if (current.size() < 2) {
        current.clear();
        return;
      }
      CoastlinePolyline line;
      line.points = decimatePolyline(current, tolerance_deg);
      current.clear();
      if (line.points.size() < 2) {
        return;
      }
      line.south = line.north = line.points.front().first;
      line.west = line.east = line.points.front().second;
      for (const auto & [lat, lon] : line.points) {
        line.south = std::min(line.south, lat);
        line.north = std::max(line.north, lat);
        line.west = std::min(line.west, lon);
        line.east = std::max(line.east, lon);
      }
      coastline.lines.push_back(std::move(line));
    };

  std::string raw;
  while (std::getline(in, raw)) {
    if (!raw.empty() && raw.back() == '\r') {
      raw.pop_back();   // a file written on Windows must still load
    }
    const std::size_t begin = raw.find_first_not_of(" \t");
    if (begin == std::string::npos || raw[begin] == '#') {
      continue;
    }
    const bool starts_polyline = raw[begin] == '>';
    std::istringstream fields(raw.substr(begin + (starts_polyline ? 1 : 0)));
    int64_t a = 0;
    int64_t b = 0;
    std::string trailing;
    const bool parsed = static_cast<bool>(fields >> a >> b) && !(fields >> trailing);
    if (starts_polyline) {
      flush();
      in_polyline = false;
      if (!parsed || std::abs(a) > kMaxLatMilli || std::abs(b) > kMaxLonMilli) {
        continue;   // unusable start: skip to the next polyline
      }
      lat_milli = a;
      lon_milli = b;
      in_polyline = true;
      current.emplace_back(lat_milli * kMilli, lon_milli * kMilli);
      continue;
    }
    if (!in_polyline) {
      continue;   // a point before any ">", or inside a dropped polyline
    }
    if (!parsed) {
      current.clear();   // drop the whole polyline: a lost delta shifts the rest
      in_polyline = false;
      continue;
    }
    lat_milli += a;
    lon_milli += b;
    if (std::abs(lat_milli) > kMaxLatMilli || std::abs(lon_milli) > kMaxLonMilli) {
      current.clear();
      in_polyline = false;
      continue;
    }
    current.emplace_back(lat_milli * kMilli, lon_milli * kMilli);
  }
  flush();
  return coastline;
}

// Load the vendored coastline from `path`. A missing or unreadable file is
// not an error the operator can act on and must not stop the explorer from
// opening: it yields an empty Coastline, i.e. no coastline layer.
inline Coastline loadCoastline(const std::string & path, double tolerance_deg = 0.0)
{
  std::ifstream in(path);
  if (!in) {
    return Coastline{};
  }
  return parseCoastline(in, tolerance_deg);
}

}  // namespace marine_perception_tools

#endif  // COASTLINE_DATA_HPP_
