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

#ifndef MBES_PASS_LOADER_HPP_
#define MBES_PASS_LOADER_HPP_

// The multi-pass MBES cloud loader (#21, moved out of MbesCloudWindow for the
// integrated explorer, #24): windowed bag reads via read_mbes_window, with
// cross-bag soundings reprojected through the earth anchor into the FIRST
// pass's world frame. Widget-free (QStringList only) so it runs on a worker
// thread and unit-tests without a display.

#include <QStringList>

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mbes_geometry.hpp"

namespace marine_perception_tools
{

// One pass to load into the cloud: the bag it lives in, its time window (from
// the survey index), and a human label for the legend.
struct CloudPassInfo
{
  std::string bag_path;
  std::string label;
  std::int64_t t_start_ns = 0;
  std::int64_t t_end_ns = 0;
};

// Everything the worker thread hands back to the UI thread in one piece.
struct CloudLoadOutcome
{
  std::vector<std::vector<MbesSounding>> pass_clouds;  // reference world frame
  std::vector<int> sounding_counts;   // per input pass (0 = empty/skipped)
  QStringList notes;                  // per-pass problems, human-readable
  int skipped_passes = 0;             // passes dropped (frame not resolvable)
  int skipped_pings = 0;              // pings dropped (no TF), summed
  // The reference frame's identity (#29): the first loadable pass's bag,
  // frame name and earth anchor — so later loads (the sidescan drape) can
  // reproject into the SAME frame the clouds and CUBE surface live in.
  std::string ref_bag;
  std::string ref_frame;
  bool ref_has_geo = false;
  geometry_msgs::msg::TransformStamped ref_earth_from_world;
  // Set when the load was cancelled mid-flight (#44): whatever it had
  // gathered is dropped, because a partial multi-pass load is indistinguish-
  // able from a complete one once it reaches the legend.
  bool cancelled = false;
};

// Optional clip region: keep only soundings within `margin_m` horizontally
// of the geographic point (a contact + margin, #24 desk finding — 6-7 passes
// over a tile is millions of soundings; the inspect-a-contact workflow needs
// just its neighbourhood). Applied per pass in that bag's own world frame
// via its earth anchor; passes without a geo anchor stay unclipped (noted).
struct GeoClip
{
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
  double margin_m = 25.0;
  // Box clip (#27, the CUBE lab): when both half-extents are positive the
  // clip is the axis-aligned box |east| <= half_east_m, |north| <= half_north_m
  // about (lat, lon) instead of the circle — evaluated in each pass's own
  // world frame, whose axes are assumed ENU-aligned (the same isotropy
  // assumption the circular clip already makes).
  double half_east_m = 0.0;
  double half_north_m = 0.0;
  bool isBox() const {return half_east_m > 0.0 && half_north_m > 0.0;}
};

// Load every pass's soundings into the first loadable pass's world frame.
// Never throws: a failing bag costs only its own pass (skipped + noted).
//
// `cancel` (optional) is polled between passes AND handed to read_mbes_window,
// which polls it per bag message (#44) — one pass over one bag is minutes of
// reading, so a between-passes check alone would bound nothing. A cancelled
// load returns `cancelled` with no clouds.
CloudLoadOutcome load_cloud_passes(
  const std::vector<CloudPassInfo> & passes,
  const std::optional<GeoClip> & clip = std::nullopt,
  const std::shared_ptr<std::atomic<bool>> & cancel = {});

}  // namespace marine_perception_tools

#endif  // MBES_PASS_LOADER_HPP_
