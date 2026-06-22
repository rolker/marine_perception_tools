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

#include "coverage_raster.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace marine_perception_tools
{

CoverageRaster::CoverageRaster(
  double origin_x, double origin_y, double resolution, int width, int height)
: origin_x_(origin_x),
  origin_y_(origin_y),
  resolution_(resolution > 0.0 ? resolution : 1.0),
  width_(width > 0 ? width : 0),
  height_(height > 0 ? height : 0),
  amplitude_(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_), -1.0f),
  quality_(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_), 0.0f),
  covered_(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_), 0u)
{
}

bool CoverageRaster::cellIndex(double x, double y, int & col, int & row) const
{
  if (width_ <= 0 || height_ <= 0) {return false;}
  const double fc = (x - origin_x_) / resolution_;
  const double fr = (y - origin_y_) / resolution_;
  if (!std::isfinite(fc) || !std::isfinite(fr)) {return false;}
  const int c = static_cast<int>(std::floor(fc));
  const int r = static_cast<int>(std::floor(fr));
  if (c < 0 || c >= width_ || r < 0 || r >= height_) {return false;}
  col = c;
  row = r;
  return true;
}

bool CoverageRaster::paint(double x, double y, float amplitude, float quality)
{
  int col = 0;
  int row = 0;
  if (!cellIndex(x, y, col, row)) {return false;}
  const std::size_t i = idx(col, row);
  if (covered_[i] && quality < quality_[i]) {return false;}  // a better look already won
  if (!covered_[i]) {
    covered_[i] = 1u;
    ++covered_count_;
  }
  amplitude_[i] = std::clamp(amplitude, 0.0f, 1.0f);
  quality_[i] = quality;
  return true;
}

bool CoverageRaster::coveredAt(int col, int row) const
{
  if (col < 0 || col >= width_ || row < 0 || row >= height_) {return false;}
  return covered_[idx(col, row)] != 0u;
}

float CoverageRaster::amplitudeAt(int col, int row) const
{
  if (col < 0 || col >= width_ || row < 0 || row >= height_) {return -1.0f;}
  const std::size_t i = idx(col, row);
  return covered_[i] ? amplitude_[i] : -1.0f;
}

bool CoverageRaster::covered(double x, double y) const
{
  int col = 0;
  int row = 0;
  if (!cellIndex(x, y, col, row)) {return false;}
  return covered_[idx(col, row)] != 0u;
}

std::size_t paint_ping(
  CoverageRaster & raster, const PingGeometry & geom, const std::vector<float> & amplitudes)
{
  std::size_t written = 0;
  for (std::size_t i = 0; i < amplitudes.size(); ++i) {
    const GroundPoint gp = project_sample(geom, i);
    if (!gp.valid) {continue;}
    if (raster.paint(gp.x, gp.y, amplitudes[i], range_quality(gp.ground_range))) {
      ++written;
    }
  }
  return written;
}

double ping_covered_fraction(
  const CoverageRaster & raster, const PingGeometry & geom,
  const std::vector<float> & amplitudes)
{
  std::size_t valid = 0;
  std::size_t already = 0;
  for (std::size_t i = 0; i < amplitudes.size(); ++i) {
    const GroundPoint gp = project_sample(geom, i);
    if (!gp.valid) {continue;}
    int col = 0;
    int row = 0;
    if (!raster.cellIndex(gp.x, gp.y, col, row)) {continue;}  // out of bounds: not "covered"
    ++valid;
    if (raster.coveredAt(col, row)) {++already;}
  }
  if (valid == 0) {return 0.0;}
  return static_cast<double>(already) / static_cast<double>(valid);
}

}  // namespace marine_perception_tools
