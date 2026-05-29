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

}  // namespace

ReSimEngine::ReSimEngine(
  const LoadedBag & bag, double window_m, double res, double max_range,
  const sea_surface_segmentation::OccupancyParams & occ, double min_grazing_angle_deg)
: bag_(bag),
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
  sea_surface_segmentation::accumulate_frame(
    buffer_, f.mask_rgb8, bag_.camera_model, f.camera_origin, f.rotation_cam_to_target,
    f.boat_x, f.boat_y, f.stamp_s, acc_);
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

}  // namespace marine_perception_tools
