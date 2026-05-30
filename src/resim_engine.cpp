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
#include <cmath>
#include <string>
#include <utility>

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
  // Populate the buffer for the first frame so the engine is immediately
  // renderable. accumulate() moves the window to the frame's boat position.
  accumulate(0);
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
  if (k == current_) {return;}
  if (k > current_) {
    for (std::size_t i = current_ + 1; i <= k; ++i) {
      accumulate(i);
    }                                                                   // incremental
  } else {
    // rewind — full replay. clear() also re-seeds the decay clock, which is what
    // makes the replay's decay sequence identical to a forward run (relied on by
    // the IncrementalEqualsReplay test).
    buffer_.clear();
    for (std::size_t i = 0; i <= k; ++i) {
      accumulate(i);
    }
  }
  current_ = k;
}

void ReSimEngine::resimToCurrent()
{
  buffer_.clear();
  for (std::size_t i = 0; i <= current_; ++i) {
    accumulate(i);
  }
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
  // AccumulateParams has no library validator; guard the tunable fields here
  // (mirrors the offline tool's --arg checks + the per-pixel-log-odds gate range).
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
  // Window geometry is construction-fixed — preserve it regardless of `p`.
  acc_.max_range = p.max_range;
  acc_.min_grazing_angle_deg = p.min_grazing_angle_deg;
  acc_.obstacle_prob_min = p.obstacle_prob_min;
  acc_.max_evidence_step = p.max_evidence_step;
  resimToCurrent();
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
  if (c.resolution <= 0.0) {return panel;}  // no recorded costmap yet
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
  return panel;
}

}  // namespace marine_perception_tools
