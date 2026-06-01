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
#include <map>
#include <string>
#include <utility>
#include <vector>

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
  // accumulates the intervening frames incrementally; a rewind restores the
  // nearest checkpoint at or before k and replays only the few frames from there
  // (evidence can't be un-accumulated, so a rewind without a checkpoint would
  // replay [0, k] — the checkpoint store bounds that to one snapshot interval).
  // Checkpoints are dropped as the window is replayed forward, so re-visiting a
  // span already seen is cheap. A parameter change invalidates them (the buffer
  // contents change), forcing one full replay.
  void seekTo(std::size_t k);

  // Seek to the frame whose stamp is the largest <= `stamp_s` (clamped to the
  // engine's covered range). The window manager scrubs by time, not index, so
  // this maps a target stamp onto the loaded window's frames. Frames are
  // stamp-sorted, so this is a binary search + seekTo.
  void seekToStamp(double stamp_s);

  // Set the current frame for `stamp_s` WITHOUT running the warm-up replay — the
  // camera RGB / segmentation / recorded-costmap views index `current_` only and
  // don't need the accumulated buffer, so this paints them instantly while the
  // (slow) regenerated costmap is still warming on a worker. Marks the engine
  // displayOnly(): renderGrid is not meaningful until a real seek warms it.
  void seekToStampDisplayOnly(double stamp_s);

  // True between a seekToStampDisplayOnly() and the next warming seek — the
  // regenerated costmap is stale; the UI shows a "computing…" placeholder.
  bool displayOnly() const {return display_only_;}

  // The stamp range this engine's loaded window covers (absolute header-stamp
  // seconds). A target stamp inside [firstStamp, lastStamp] can be rendered by an
  // in-window seek (cheap via the checkpoint store); outside, the window manager
  // must reload a new span.
  double firstStamp() const {return bag_.frames.front().stamp_s;}
  double lastStamp() const {return bag_.frames.back().stamp_s;}

  // Live-tunable knobs. On rejection, return false and set `why`, leaving engine
  // state unchanged. On success, apply and re-simulate to the current frame.
  bool setOccupancyParams(
    const sea_surface_segmentation::OccupancyParams & p, std::string & why);
  // The window geometry fields of `p` (res / half_extent / plane_z) are ignored —
  // they are construction-fixed; only the tunable accumulate knobs are applied.
  bool setAccumulateParams(
    const sea_surface_segmentation::AccumulateParams & p, std::string & why);

  // Apply BOTH param structs in one batch: validate both first (no state change
  // if either is rejected — sets `why` to the first failure), then apply both and
  // re-simulate ONCE. The Apply button (UI) uses this so a batch of edits across
  // the occupancy and accumulate knobs pays a single warm-up replay, not two.
  bool setParams(
    const sea_surface_segmentation::OccupancyParams & occ,
    const sea_surface_segmentation::AccumulateParams & acc, std::string & why);

  // Validate a param batch WITHOUT applying it (same checks as setParams). The UI
  // calls this on the GUI thread so an invalid Apply is rejected instantly,
  // before spawning the background warm-up.
  static bool validateParams(
    const sea_surface_segmentation::OccupancyParams & occ,
    const sea_surface_segmentation::AccumulateParams & acc, std::string & why);

  const sea_surface_segmentation::OccupancyParams & occupancyParams() const {return occ_;}
  const sea_surface_segmentation::AccumulateParams & accumulateParams() const {return acc_;}

  // Latest segmentation mask for `cam` (index into kCameraNames) at or before the
  // current frame, or an empty Mat if that camera has no frame yet — drives the
  // per-camera segmentation panes. `latestRgb` is the same for the display RGB.
  cv::Mat latestMask(int cam) const;
  cv::Mat latestRgb(int cam) const;
  double currentStamp() const {return bag_.frames[current_].stamp_s;}

  // Horizon polyline for the overlay: image-space (x,y) points tracing where the
  // water plane vanishes, in the coordinates of the latest RGB / segmentation
  // frame for `cam` at or before the current stamp. Empty if that frame is
  // absent or its pose is unavailable. Computed from the camera model
  // (distortion-aware projectPixelTo3dRay) + that frame's own-stamp camera→world
  // rotation, so a TF/image timing desync shows as a mismatched line. `n_cols`
  // samples evenly across the image width.
  std::vector<cv::Point2d> rgbHorizon(int cam, int n_cols = 64) const;
  std::vector<cv::Point2d> segHorizon(int cam, int n_cols = 64) const;

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
  void resimToCurrent();  // clear + replay [0, current_]; invalidates checkpoints

  // Checkpoint store: the OccupancyBuffer is copyable and a copy captures the
  // full decay state (last_decay_s_, seeded_), so restoring a snapshot and
  // replaying forward is bit-identical to replaying [0, k] from scratch (the R1
  // determinism guarantee). Snapshots are keyed by the frame index they were
  // taken AFTER (i.e. the buffer state with frames [0, idx] accumulated), spaced
  // at least kSnapshotIntervalS of bag time apart so the count is bounded by the
  // window length. A parameter change clears them (clearCheckpoints()).
  void maybeCheckpoint(std::size_t i);  // snapshot after accumulating frame i, if due
  void clearCheckpoints();

  // ~1 s of bag time between snapshots. Trades memory (≈0.9 MB per 480^2-cell
  // snapshot) for back-scrub latency (replay <= one interval). Tunable later.
  static constexpr double kSnapshotIntervalS = 1.0;

  LoadedBag bag_;
  sea_surface_segmentation::AccumulateParams acc_;  // res/half_extent fixed; rest tunable
  sea_surface_segmentation::OccupancyParams occ_;
  sea_surface_segmentation::OccupancyBuffer buffer_;
  std::size_t current_ = 0;

  // frame index -> buffer snapshot after accumulating [0, index]. current_ is
  // always representable from the nearest entry <= current_ plus a short replay.
  std::map<std::size_t, sea_surface_segmentation::OccupancyBuffer> checkpoints_;
  double last_checkpoint_stamp_s_ = 0.0;  // stamp of the most recent snapshot
  bool have_checkpoint_ = false;          // false until the first snapshot
  // True after seekToStampDisplayOnly: current_ moved for the image views but the
  // buffer was NOT replayed, so renderGrid is stale. Cleared by any real seek.
  bool display_only_ = false;
};

}  // namespace marine_perception_tools

#endif  // RESIM_ENGINE_HPP_
