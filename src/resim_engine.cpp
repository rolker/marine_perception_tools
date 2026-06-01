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

#include "resim_engine.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "sea_surface_segmentation/occupancy_accumulator.hpp"

namespace marine_perception_tools
{
namespace
{

// Colour a log-odds value for the costmap render (BGR). Reproduces the offline
// tool's `colour_logodds()` palette (anonymous-namespace in
// bag_to_costmap_video.cpp, so copied rather than linked — purely cosmetic; can
// drift from the tool without affecting the reused algorithm).
cv::Vec3b colour_logodds(double v, double threshold)
{
  if (std::isnan(v)) {return {110, 110, 110};}    // unknown — grey
  if (v >= threshold) {return {40, 40, 230};}     // lethal — red
  if (v > 0.0) {                                   // sub-threshold occupied — green→yellow
    const double f = std::min(v / threshold, 1.0);
    return {30, static_cast<uchar>(120 + 100 * f), static_cast<uchar>(200 * f)};
  }
  return {70, 40, 20};  // free / negative — dark blue
}

// Colour a nav2 OccupancyGrid cost (-1 unknown, 0 free, 100 lethal) with the SAME
// palette as colour_logodds so recorded and regenerated compare apples-to-apples.
// Reproduces the offline tool's `colour_cost()` (anonymous-namespace, so copied).
cv::Vec3b colour_cost(int v)
{
  if (v < 0) {return {110, 110, 110};}            // NO_INFORMATION — grey
  if (v >= 99) {return {40, 40, 230};}            // inscribed/lethal — red
  if (v > 0) {                                     // intermediate cost — green→yellow
    const double f = std::min(v / 99.0, 1.0);
    return {30, static_cast<uchar>(120 + 100 * f), static_cast<uchar>(200 * f)};
  }
  return {70, 40, 20};  // free — dark blue
}

// Draw a boat marker (heading arrow) at the panel centre on a boat-centred,
// north-up costmap render. `yaw` is the world-frame heading (rad, CCW from +x);
// the panel maps world +x → +u (right) and world +y → −v (up), so the screen
// direction is (cos yaw, −sin yaw). White with a thin black outline so it reads
// on any palette cell underneath.
void draw_boat_marker(cv::Mat & panel, double yaw)
{
  const cv::Point2d c(panel.cols / 2.0, panel.rows / 2.0);
  const double len = std::max(8.0, panel.rows * 0.06);  // arrow length ~6% of panel
  const cv::Point2d dir(std::cos(yaw), -std::sin(yaw));  // heading in screen coords
  const cv::Point2d perp(-dir.y, dir.x);
  const cv::Point2d tip = c + dir * len;
  const cv::Point2d tail = c - dir * (len * 0.6);
  // Triangle: tip + two tail corners, so it reads as a pointer not just a line.
  const std::array<cv::Point, 3> tri{
    cv::Point(tip),
    cv::Point(tail + perp * (len * 0.45)),
    cv::Point(tail - perp * (len * 0.45))};
  const cv::Point * pts = tri.data();
  int npts = 3;
  cv::fillConvexPoly(panel, pts, npts, cv::Scalar(255, 255, 255), cv::LINE_AA);
  cv::polylines(panel, &pts, &npts, 1, true, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
}

}  // namespace

ReSimEngine::ReSimEngine(
  LoadedBag bag, double window_m, double res, double max_range,
  const sea_surface_segmentation::OccupancyParams & occ, double min_grazing_angle_deg)
: bag_(std::move(bag)),
  occ_(occ),
  buffer_(window_m, window_m, res, grid_map::Position(0.0, 0.0), occ)
{
  // Window geometry is construction-fixed; the rest of acc_ keeps the struct's
  // defaults (obstacle_prob_min, max_evidence_step) until tuned via the dock.
  acc_.res = res;
  acc_.half_extent = window_m / 2.0;
  acc_.plane_z = 0.0;  // map_tide water plane
  acc_.max_range = max_range;
  acc_.min_grazing_angle_deg = min_grazing_angle_deg;
  // The engine requires at least one frame: the ctor seeds the buffer from
  // frame 0 (below), and every accessor indexes bag_.frames[current_]. The bag
  // loaders (load_bag / BagSession::loadWindow) already throw on an empty window,
  // but guard here so a direct/empty construction fails loudly instead of reading
  // past the vector.
  if (bag_.frames.empty()) {
    throw std::invalid_argument("ReSimEngine requires a LoadedBag with >= 1 frame");
  }
  // Populate the buffer for the first frame so the engine is immediately
  // renderable. accumulate() moves the window to the frame's boat position.
  accumulate(0);
  maybeCheckpoint(0);  // anchor at frame 0 so any rewind has a base to restore
}

void ReSimEngine::accumulate(std::size_t i)
{
  const PreparedFrame & f = bag_.frames[i];
  // Every camera's frames feed one shared buffer (the frame carries its own
  // camera model), so the four streams fuse exactly as the deployed layer does.
  sea_surface_segmentation::accumulate_frame(
    buffer_, f.mask_rgb8, bag_.camera_models[f.cam], f.camera_origin,
    f.rotation_cam_to_target, f.boat_x, f.boat_y, f.stamp_s, acc_);
}

cv::Mat ReSimEngine::latestMask(int cam) const
{
  if (bag_.frames.empty()) {return cv::Mat();}
  for (std::size_t i = current_ + 1; i-- > 0; ) {
    if (bag_.frames[i].cam == cam) {return bag_.frames[i].mask_rgb8;}
  }
  return cv::Mat();
}

cv::Mat ReSimEngine::latestRgb(int cam) const
{
  // rgb_frames are time-sorted; return the last one for this camera at or before
  // the current frame's stamp. Empty until the H.265 stream is decoded.
  if (bag_.rgb_frames.empty()) {return cv::Mat();}
  const double t = currentStamp();
  cv::Mat best;
  for (const auto & r : bag_.rgb_frames) {
    if (r.cam != cam) {continue;}
    if (r.stamp_s > t) {break;}  // sorted — nothing later qualifies
    best = r.bgr;
  }
  return best;
}

RecordedCostmap ReSimEngine::currentRecordedCostmap() const
{
  RecordedCostmap best;  // resolution 0 == none
  if (bag_.costmaps.empty()) {return best;}
  const double t = currentStamp();
  for (const auto & c : bag_.costmaps) {
    if (c.stamp_s > t) {break;}  // sorted
    best = c;
  }
  return best;
}

void ReSimEngine::seekTo(std::size_t k)
{
  if (bag_.frames.empty()) {return;}
  k = std::min(k, bag_.frames.size() - 1);
  // After a display-only seek the buffer is stale even at the same index, so a
  // warming seek to current_ must NOT early-return — force a full replay to k.
  if (k == current_ && !display_only_) {return;}
  if (display_only_) {
    // Buffer doesn't reflect current_; rebuild from scratch to k (the checkpoint
    // forward/rewind shortcuts assume buffer↔current_ consistency, which a
    // display-only jump broke).
    display_only_ = false;
    current_ = 0;
    clearCheckpoints();
    buffer_.clear();
    for (std::size_t i = 0; i <= k; ++i) {
      accumulate(i);
      maybeCheckpoint(i);
    }
    current_ = k;
    return;
  }
  if (k > current_) {
    // Forward: accumulate the intervening frames incrementally, snapshotting as
    // we pass so a later rewind to this span is cheap.
    for (std::size_t i = current_ + 1; i <= k; ++i) {
      accumulate(i);
      maybeCheckpoint(i);
    }
  } else {
    // Rewind: restore the nearest checkpoint at or before k, then replay only the
    // frames after it. A checkpoint copy carries the full decay state, so the
    // restored-then-replayed buffer is identical to a clear()+replay([0,k]) (the
    // determinism the IncrementalEqualsReplay / checkpoint tests assert). Without
    // a usable checkpoint (e.g. none below k) fall back to clear()+replay from 0.
    auto it = checkpoints_.upper_bound(k);  // first entry with index > k
    if (it == checkpoints_.begin()) {
      buffer_.clear();
      for (std::size_t i = 0; i <= k; ++i) {
        accumulate(i);
      }
    } else {
      --it;                       // greatest index <= k
      buffer_ = it->second;       // restore snapshot (frames [0, it->first] applied)
      for (std::size_t i = it->first + 1; i <= k; ++i) {
        accumulate(i);
      }
    }
  }
  current_ = k;
}

namespace
{
// Largest frame index whose stamp <= stamp_s (frames are stamp-sorted); clamps
// to 0 if the target is before the first frame.
std::size_t frame_index_for_stamp(
  const std::vector<PreparedFrame> & frames, double stamp_s)
{
  auto it = std::upper_bound(
    frames.begin(), frames.end(), stamp_s,
    [](double s, const PreparedFrame & f) {return s < f.stamp_s;});
  return (it == frames.begin()) ? 0 :
         static_cast<std::size_t>((it - frames.begin()) - 1);
}
}  // namespace

void ReSimEngine::seekToStamp(double stamp_s)
{
  if (bag_.frames.empty()) {return;}
  seekTo(frame_index_for_stamp(bag_.frames, stamp_s));
}

void ReSimEngine::seekToStampDisplayOnly(double stamp_s)
{
  if (bag_.frames.empty()) {return;}
  // Move the frame index for the image/seg/recorded views ONLY — no accumulate,
  // no replay. The buffer (and thus renderGrid) is now stale: flag it so the UI
  // shows a "computing…" placeholder until a warming seek runs. This engine is
  // display-only (stage A); the warmed regenerated costmap comes from a separate
  // engine built in stage B, so the broken buffer↔current_ invariant is fine —
  // renderGrid is never trusted while display_only_ is set.
  current_ = frame_index_for_stamp(bag_.frames, stamp_s);
  display_only_ = true;
}

void ReSimEngine::resimToCurrent()
{
  // A parameter change alters the buffer contents at every frame, so every
  // existing snapshot is stale: drop them and rebuild from scratch, re-anchoring
  // checkpoints along the replay so subsequent scrubs under the new params are
  // cheap again.
  clearCheckpoints();
  buffer_.clear();
  for (std::size_t i = 0; i <= current_; ++i) {
    accumulate(i);
    maybeCheckpoint(i);
  }
}

void ReSimEngine::maybeCheckpoint(std::size_t i)
{
  // Snapshot the buffer state (frames [0, i] applied) when at least
  // kSnapshotIntervalS of bag time has elapsed since the last snapshot — or
  // always for the very first one. Keying by frame index lets a rewind restore
  // the nearest <= target; spacing by stamp bounds the count to ~window/interval.
  if (checkpoints_.count(i) != 0) {
    return;  // already snapshotted here (re-visited frame) — nothing to do
  }
  const double stamp = bag_.frames[i].stamp_s;
  if (have_checkpoint_ && (stamp - last_checkpoint_stamp_s_) < kSnapshotIntervalS) {
    return;  // too soon since the last snapshot
  }
  checkpoints_.emplace(i, buffer_);  // copy current buffer state
  last_checkpoint_stamp_s_ = stamp;
  have_checkpoint_ = true;
}

void ReSimEngine::clearCheckpoints()
{
  checkpoints_.clear();
  have_checkpoint_ = false;
  last_checkpoint_stamp_s_ = 0.0;
}

namespace
{
// AccumulateParams has no library validator; guard the tunable fields here
// (mirrors the offline tool's --arg checks + the per-pixel-log-odds gate range).
// Window geometry (res/half_extent/plane_z) is construction-fixed and not checked.
bool validate_accumulate(
  const sea_surface_segmentation::AccumulateParams & p, std::string & why)
{
  if (!(std::isfinite(p.max_range) && p.max_range > 0.0)) {
    why = "max_range must be finite and > 0";
    return false;
  }
  if (!(std::isfinite(p.min_grazing_angle_deg) &&
    p.min_grazing_angle_deg >= 0.0 && p.min_grazing_angle_deg < 90.0))
  {
    why = "min_grazing_angle_deg must be in [0, 90)";
    return false;
  }
  if (!(std::isfinite(p.obstacle_prob_min) &&
    p.obstacle_prob_min >= 0.0 && p.obstacle_prob_min <= 1.0))
  {
    why = "obstacle_prob_min must be in [0, 1]";
    return false;
  }
  if (!(std::isfinite(p.max_evidence_step) && p.max_evidence_step > 0.0)) {
    why = "max_evidence_step must be finite and > 0";
    return false;
  }
  return true;
}
}  // namespace

bool ReSimEngine::validateParams(
  const sea_surface_segmentation::OccupancyParams & occ,
  const sea_surface_segmentation::AccumulateParams & acc, std::string & why)
{
  return sea_surface_segmentation::OccupancyBuffer::validate(occ, why) &&
         validate_accumulate(acc, why);
}

bool ReSimEngine::setOccupancyParams(
  const sea_surface_segmentation::OccupancyParams & p, std::string & why)
{
  if (!sea_surface_segmentation::OccupancyBuffer::validate(p, why)) {
    return false;
  }
  occ_ = p;
  buffer_.setParams(p);
  resimToCurrent();  // increments + decay change → re-accumulate the window
  return true;
}

bool ReSimEngine::setAccumulateParams(
  const sea_surface_segmentation::AccumulateParams & p, std::string & why)
{
  if (!validate_accumulate(p, why)) {
    return false;
  }
  // Window geometry is construction-fixed — preserve it regardless of `p`.
  acc_.max_range = p.max_range;
  acc_.min_grazing_angle_deg = p.min_grazing_angle_deg;
  acc_.obstacle_prob_min = p.obstacle_prob_min;
  acc_.max_evidence_step = p.max_evidence_step;
  resimToCurrent();
  return true;
}

bool ReSimEngine::setParams(
  const sea_surface_segmentation::OccupancyParams & occ,
  const sea_surface_segmentation::AccumulateParams & acc, std::string & why)
{
  // Validate BOTH before applying EITHER, so a rejected batch leaves the engine
  // untouched (no half-applied params, no wasted re-sim).
  if (!sea_surface_segmentation::OccupancyBuffer::validate(occ, why)) {
    return false;
  }
  if (!validate_accumulate(acc, why)) {
    return false;
  }
  occ_ = occ;
  buffer_.setParams(occ);
  acc_.max_range = acc.max_range;
  acc_.min_grazing_angle_deg = acc.min_grazing_angle_deg;
  acc_.obstacle_prob_min = acc.obstacle_prob_min;
  acc_.max_evidence_step = acc.max_evidence_step;
  resimToCurrent();  // single warm-up replay for the whole batch
  return true;
}

cv::Mat ReSimEngine::renderGrid(int panel_px) const
{
  const double bx = bag_.frames[current_].boat_x;
  const double by = bag_.frames[current_].boat_y;
  cv::Mat panel(panel_px, panel_px, CV_8UC3);
  for (int v = 0; v < panel_px; ++v) {
    for (int u = 0; u < panel_px; ++u) {
      const double wx = bx + (u - panel_px / 2) * acc_.res;
      const double wy = by - (v - panel_px / 2) * acc_.res;  // image y down → world y up
      panel.at<cv::Vec3b>(v, u) =
        colour_logodds(buffer_.logOdds(grid_map::Position(wx, wy)), occ_.lethal_threshold);
    }
  }
  draw_boat_marker(panel, bag_.frames[current_].boat_yaw);
  return panel;
}

cv::Mat ReSimEngine::renderRecorded(int panel_px) const
{
  // Boat-centred, world-aligned (N-up) sample of the recorded OccupancyGrid into
  // the same window as renderGrid. Grid origin is in its own frame (bizzy/map),
  // which shares the xy plane with map_tide (the tide transform is z-only), so
  // boat-xy and grid-xy are consistent. Outside the grid reads as unknown.
  const RecordedCostmap c = currentRecordedCostmap();
  const double bx = bag_.frames[current_].boat_x;
  const double by = bag_.frames[current_].boat_y;
  cv::Mat panel(panel_px, panel_px, CV_8UC3, cv::Scalar(110, 110, 110));
  if (c.resolution <= 0.0) {
    draw_boat_marker(panel, bag_.frames[current_].boat_yaw);  // marker even with no costmap
    return panel;
  }
  for (int v = 0; v < panel_px; ++v) {
    for (int u = 0; u < panel_px; ++u) {
      const double wx = bx + (u - panel_px / 2) * acc_.res;
      const double wy = by - (v - panel_px / 2) * acc_.res;  // image y down → world y up
      const int gx = static_cast<int>(std::floor((wx - c.origin_x) / c.resolution));
      const int gy = static_cast<int>(std::floor((wy - c.origin_y) / c.resolution));
      int cost = -1;  // outside the recorded window reads as unknown
      if (gx >= 0 && gx < c.width && gy >= 0 && gy < c.height) {
        cost = c.data[static_cast<std::size_t>(gy) * c.width + gx];
      }
      panel.at<cv::Vec3b>(v, u) = colour_cost(cost);
    }
  }
  draw_boat_marker(panel, bag_.frames[current_].boat_yaw);
  return panel;
}

}  // namespace marine_perception_tools
