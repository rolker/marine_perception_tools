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

#ifndef MBES_WINDOW_READER_HPP_
#define MBES_WINDOW_READER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "mbes_geometry.hpp"
#include "tf_lift.hpp"

namespace marine_perception_tools
{

// One-shot windowed MBES pull for the multi-pass cloud (#21): soundings for a
// known [t_start_ns, t_end_ns] pass interval, WITHOUT the full-bag metadata
// index a SidescanBagSession builds (right for the scrub viewer, too costly
// when several bags load at once). Two topic-filtered passes over the bag: a
// TF prepass from the bag start (static transforms + chain history live
// there) up to the window end, then a seek to the window for the detections.

// Frame/topic knobs, defaulting to the same conventions as SidescanBagOptions.
struct MbesWindowOptions
{
  std::string world_frame = "bizzy/map";  // local-tangent ENU render frame
  std::string geo_frame = "earth";        // geo anchor for cross-bag reprojection
  std::string detections_topic;           // empty -> kMbesDetectionsTopic
};

struct MbesWindowResult
{
  // Valid-beam soundings in the BAG's world frame (see earth_from_world for
  // combining across bags whose local world frames differ).
  std::vector<MbesSounding> world_soundings;
  int used_pings = 0;
  int skipped_pings = 0;  // detections in the window with no resolvable TF
  // earth<-world at the window midpoint, when the bag carries the geo anchor.
  bool has_geo = false;
  geometry_msgs::msg::TransformStamped earth_from_world;
  std::string world_frame;  // echo of the frame the soundings are in
};

MbesWindowResult read_mbes_window(
  const std::string & bag_uri, std::int64_t t_start_ns, std::int64_t t_end_ns,
  const MbesWindowOptions & options = {});

// Rigid transform taking points from one bag's world frame into another's,
// composed through the shared geo frame: T_ref<-src = inv(T_earth<-ref) *
// T_earth<-src. Pure math — pinned in test_mbes_window_reader.
struct FrameReprojection
{
  bool identity = true;
  double tx = 0.0, ty = 0.0, tz = 0.0;
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
};

FrameReprojection make_reprojection(
  const geometry_msgs::msg::TransformStamped & earth_from_ref,
  const geometry_msgs::msg::TransformStamped & earth_from_src);

inline void apply_reprojection(
  const FrameReprojection & r, double & x, double & y, double & z)
{
  if (r.identity) {
    return;
  }
  double rx = 0.0;
  double ry = 0.0;
  double rz = 0.0;
  rotate_by_quat(r.qx, r.qy, r.qz, r.qw, x, y, z, rx, ry, rz);
  x = r.tx + rx;
  y = r.ty + ry;
  z = r.tz + rz;
}

}  // namespace marine_perception_tools

#endif  // MBES_WINDOW_READER_HPP_
