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

#ifndef COVERAGE_RASTER_HPP_
#define COVERAGE_RASTER_HPP_

#include <cstdint>
#include <vector>

#include "sidescan_geometry.hpp"

// A north-up backscatter raster in the bizzy/map ENU plane (metres). Each cell
// holds a backscatter amplitude [0, 1] and the quality score of the sample that
// wrote it; a later sample overwrites a cell only when its quality is at least
// the stored quality (quality-wins, the offline-store default — no liveness
// pressure, so the best look survives rather than the newest). Coverage is
// tracked so a ping whose ground footprint is already fully covered can be
// skipped. Kept Qt-free and self-contained (no grid_map dependency) so the paint
// + quality-wins + coverage logic is deterministically unit-testable.

namespace marine_perception_tools
{

// Quality score for a projected sample at a given ground range. Higher wins.
// Placeholder proxy for a grazing-angle metric (ADR-0006/0007 territory): nearer
// ground range = better resolution + SNR, so quality decreases with range. The
// nadir gap is already excluded upstream (project_sample marks it invalid).
inline float range_quality(double ground_range)
{
  return static_cast<float>(1.0 / (1.0 + (ground_range > 0.0 ? ground_range : 0.0)));
}

class CoverageRaster
{
public:
  // Raster covering [origin_x, origin_x + width*resolution) east and similarly
  // north, with square cells of `resolution` metres. Cells start uncovered.
  CoverageRaster(double origin_x, double origin_y, double resolution, int width, int height);

  int width() const {return width_;}
  int height() const {return height_;}
  double resolution() const {return resolution_;}
  double originX() const {return origin_x_;}
  double originY() const {return origin_y_;}

  // Map (x, y) -> cell (col, row). Returns false (and leaves col/row unset) when
  // the point is outside the raster.
  bool cellIndex(double x, double y, int & col, int & row) const;

  // Paint one sample at map (x, y). Writes the cell when it is uncovered or when
  // `quality` >= the stored quality. Returns true if the cell was written.
  // Out-of-bounds points are ignored (returns false).
  bool paint(double x, double y, float amplitude, float quality);

  bool coveredAt(int col, int row) const;
  // Amplitude at a cell, or -1 when uncovered / out of bounds.
  float amplitudeAt(int col, int row) const;
  bool covered(double x, double y) const;

  std::size_t coveredCellCount() const {return covered_count_;}

private:
  std::size_t idx(int col, int row) const
  {
    return static_cast<std::size_t>(row) * static_cast<std::size_t>(width_) +
           static_cast<std::size_t>(col);
  }

  double origin_x_;
  double origin_y_;
  double resolution_;
  int width_;
  int height_;
  std::vector<float> amplitude_;   // -1 == uncovered
  std::vector<float> quality_;
  std::vector<uint8_t> covered_;
  std::size_t covered_count_ = 0;
};

// Paint every valid (past-nadir, in-bounds) sample of one ping's beam-0 amplitudes
// into the raster, using range_quality() for the quality-wins comparison. Returns
// the number of cells written.
std::size_t paint_ping(
  CoverageRaster & raster, const PingGeometry & geom, const std::vector<float> & amplitudes);

// Fraction [0, 1] of a ping's valid, in-bounds samples that land on already-covered
// cells. 1.0 means the ping adds nothing new (caller skips it); the result is 0.0
// when the ping has no valid in-bounds samples at all.
double ping_covered_fraction(
  const CoverageRaster & raster, const PingGeometry & geom,
  const std::vector<float> & amplitudes);

}  // namespace marine_perception_tools

#endif  // COVERAGE_RASTER_HPP_
