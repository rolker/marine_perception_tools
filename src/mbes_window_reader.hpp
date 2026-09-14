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

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "cube_bathymetry/projection_summary.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "mbes_geometry.hpp"
#include "mbes_projection.hpp"
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
//
// The three projector frames (#55) are the ones cube::DetectionsProjector
// needs to resolve attitude (level <- base_link) and heave (tide <- base_link)
// for the error model. Their defaults are NOT the cube library's — they were
// read out of a real BizzyBoat M3 recording (the 2026-08-20 bizzy_timing bag)
// and are namespaced to match, like SidescanBagOptions::base_frame.
//
// This matters more than a default usually does. cube::ProjectorParams
// defaults base_link_frame to an unprefixed "base_link", and these bags carry
// a `bizzy/base_link -> base_link -> base_link_frd` alias chain, so an
// unprefixed lookup resolves to SOMETHING without throwing: the attitude would
// silently come from a frame that is not the boat, and every sounding's
// uncertainty with it. Carrying the frame here, next to the two that were
// already namespaced, is what stops that.
struct MbesWindowOptions
{
  std::string world_frame = "bizzy/map";  // local-tangent ENU render frame
  std::string geo_frame = "earth";        // geo anchor for cross-bag reprojection
  std::string detections_topic;           // empty -> kMbesDetectionsTopic
  // cube::DetectionsProjector frames, verified against a real bag (#55).
  std::string base_link_frame = "bizzy/base_link";
  std::string level_frame = "bizzy/base_link_north_up";
  std::string tide_frame = "bizzy/map_tide";
};

struct MbesWindowResult
{
  // Valid-beam soundings in the BAG's world frame (see earth_from_world for
  // combining across bags whose local world frames differ).
  std::vector<MbesSounding> world_soundings;
  int used_pings = 0;
  int skipped_pings = 0;  // detections in the window with no resolvable TF
  // What the real projector did with this window (#55), in the accumulator
  // type cube_bathymetry's own offline tools use — so report_projection_summary
  // can format it, and so a caller can sum several windows field by field.
  // `reports_georeferencing` is false: this path's "georeferenced" concept
  // (has_geo / earth_from_world, the cross-bag earth-anchor reprojection) is a
  // different thing from per-sounding georeferencing, and the summary must not
  // conflate the two.
  cube::ProjectionRunTotals diagnostics;
  // Drops the projector's own counters have no field for: a ping refused for a
  // non-positive sound speed, and a sounding refused for a non-positive slant
  // range. See project_ping().
  int invalid_pings = 0;
  int invalid_beams = 0;
  // earth<-world at the window midpoint, when the bag carries the geo anchor.
  bool has_geo = false;
  geometry_msgs::msg::TransformStamped earth_from_world;
  std::string world_frame;  // echo of the frame the soundings are in
  // Set when `cancel` fired mid-read (#44). The soundings are cleared with
  // it: a half-read window is not a short window, and no caller may treat
  // one as data.
  bool cancelled = false;
};

// Fold one ping's projection into a window result's running totals (#55).
//
// Named and exposed rather than inlined in the read loop so it can be checked
// without a bag: it is the step every count in the lab's load note is built
// from, and a silent slip here (a field added to ProjectionDiagnostics and not
// accumulated, say) would understate a drop population rather than fail.
//
// `beams` comes from ProjectionDiagnostics::total — the soundings the error
// model produced BEFORE the range gate, one per beam — so
// beams = soundings + filtered_range + invalid_beams holds per ping and
// therefore over the sum.
inline void accumulate_ping(MbesWindowResult & result, const PingProjection & p)
{
  result.diagnostics.pings += 1;
  result.diagnostics.beams += p.diagnostics.total;
  result.diagnostics.soundings += p.soundings.size();
  result.diagnostics.filtered_range += p.diagnostics.filtered_range;
  result.diagnostics.missing_attitude += p.diagnostics.missing_attitude;
  result.diagnostics.missing_heave += p.diagnostics.missing_heave;
  result.diagnostics.default_beamwidth_beams += p.diagnostics.default_beamwidth_beams;
  result.diagnostics.missing_rx_angle_beams += p.diagnostics.missing_rx_angle_beams;
  result.invalid_pings += static_cast<int>(p.invalid_pings);
  result.invalid_beams += static_cast<int>(p.invalid_beams);
}

// `cancel` (optional) is polled per bag message in every read loop — the TF
// prepasses and the detections pass alike (#44). A window read is minutes of
// I/O on a long bag, so anything coarser (per bag, per pass) would leave the
// operator's close waiting on the whole thing. Once set, the read abandons
// its work and returns `cancelled` with no soundings.
MbesWindowResult read_mbes_window(
  const std::string & bag_uri, std::int64_t t_start_ns, std::int64_t t_end_ns,
  const MbesWindowOptions & options = {},
  const std::shared_ptr<std::atomic<bool>> & cancel = {});

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
