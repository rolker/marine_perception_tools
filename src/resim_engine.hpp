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

#ifndef RESIM_ENGINE_HPP_
#define RESIM_ENGINE_HPP_

#include <cstddef>
#include <string>
#include <utility>

#include <opencv2/core.hpp>

#include "sea_surface_segmentation/occupancy_accumulator.hpp"
#include "sea_surface_segmentation/occupancy_buffer.hpp"

#include "bag_loader.hpp"

namespace marine_perception_tools
{

// Drives the real sea_surface_segmentation marking algorithm over a preloaded
// frame window: replays frames through `accumulate_frame` into an OccupancyBuffer
// and renders the result, re-simulating when a parameter changes. Reproduces the
// #23 shared offline core (`bag_to_costmap_video`) — boat-centred iteration; the
// live SeaSurfaceLayer centres on the camera instead, so the match is exact for
// the offline core and up to that small window-centre offset for the live layer.
//
// Qt-free and ROS-message-free so the breakable logic (path-dependent re-sim) is
// unit-testable without a display.
class ReSimEngine
{
public:
  // `window_m` (full side) and `res` (cell pitch) size the OccupancyBuffer at
  // construction. Buffer geometry is not resizable, so these are fixed for the
  // engine's lifetime — change them by constructing a new engine. The engine
  // takes ownership of `bag` (by value) so the window can replace the whole
  // engine on File->Open without dangling references into a prior load.
  ReSimEngine(
    LoadedBag bag, double window_m, double res, double max_range,
    const sea_surface_segmentation::OccupancyParams & occ =
    sea_surface_segmentation::OccupancyParams{},
    double min_grazing_angle_deg = 0.0);

  std::size_t frameCount() const {return bag_.frames.size();}
  std::size_t currentIndex() const {return current_;}

  // True when the segmentation came from the lossy-capable `.../compressed`
  // topic (raw Image absent) — the window surfaces the fidelity caveat.
  bool usedCompressedSegmentation() const {return bag_.used_compressed_segmentation;}

  // Move the simulation to frame k (clamped to a valid index). A forward move
  // accumulates the intervening frames incrementally; a rewind clears and
  // replays [0, k] (evidence can't be un-accumulated).
  void seekTo(std::size_t k);

  // Live-tunable knobs. On rejection, return false and set `why`, leaving engine
  // state unchanged. On success, apply and re-simulate to the current frame.
  bool setOccupancyParams(
    const sea_surface_segmentation::OccupancyParams & p, std::string & why);
  // The window geometry fields of `p` (res / half_extent / plane_z) are ignored —
  // they are construction-fixed; only the tunable accumulate knobs are applied.
  bool setAccumulateParams(
    const sea_surface_segmentation::AccumulateParams & p, std::string & why);

  const sea_surface_segmentation::OccupancyParams & occupancyParams() const {return occ_;}
  const sea_surface_segmentation::AccumulateParams & accumulateParams() const {return acc_;}

  // Latest segmentation mask for `cam` (index into kCameraNames) at or before the
  // current frame, or an empty Mat if that camera has no frame yet — drives the
  // per-camera segmentation panes. `latestRgb` is the same for the display RGB.
  cv::Mat latestMask(int cam) const;
  cv::Mat latestRgb(int cam) const;
  double currentStamp() const {return bag_.frames[current_].stamp_s;}

  // The recorded costmap sample at or before the current frame's stamp (an empty
  // RecordedCostmap — resolution <= 0 — if none yet). Display-only comparison.
  RecordedCostmap currentRecordedCostmap() const;

  // Boat-centred, world-aligned (N-up) BGR render of the occupancy buffer at the
  // current frame's boat position (panel_px square). Reproduces the offline
  // tool's `render_new()`/`colour_logodds()` palette (not exported, so copied).
  cv::Mat renderGrid(int panel_px) const;

  // The recorded costmap sampled into the SAME boat-centred window/scale as
  // renderGrid, so the two panes overlay 1:1. Promotes the offline driver's
  // `render_live()`/`colour_cost()` (nav2 cost palette matched to the log-odds
  // one). An all-grey panel means no recorded costmap at/before the current frame.
  cv::Mat renderRecorded(int panel_px) const;

  // Raw log-odds at a world cell — exposed for tests/diagnostics.
  double logOddsAt(double wx, double wy) const {return buffer_.logOdds(grid_position(wx, wy));}

private:
  static grid_map::Position grid_position(double wx, double wy) {return {wx, wy};}
  void accumulate(std::size_t i);
  void resimToCurrent();  // clear + replay [0, current_]

  LoadedBag bag_;
  sea_surface_segmentation::AccumulateParams acc_;  // res/half_extent fixed; rest tunable
  sea_surface_segmentation::OccupancyParams occ_;
  sea_surface_segmentation::OccupancyBuffer buffer_;
  std::size_t current_ = 0;
};

}  // namespace marine_perception_tools

#endif  // RESIM_ENGINE_HPP_
