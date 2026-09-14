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

#ifndef MBES_PROJECTION_HPP_
#define MBES_PROJECTION_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "cube_bathymetry/detections_projector.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "marine_acoustic_msgs/msg/sonar_detections.hpp"
#include "tf2/buffer_core.h"

#include "mbes_geometry.hpp"

// The CUBE-lab estimator path's projection (#55): the REAL
// cube::DetectionsProjector and cube::ErrorModel, in place of the angle-aware
// placeholder that used to compute a stand-in uncertainty inside run_cube.
//
// Everything offline-specific lives here, in one place, so the reader
// (mbes_window_reader.cpp) does not hand-roll projector setup and so the
// CUBE-tuning dialog and the lab's load notes quote ONE caveat string instead
// of two copies that drift.
//
// WHAT THIS PATH CANNOT CONFIGURE. There is no offline vessel/device
// configuration record to read (cube_bathymetry#145), so the projector runs on
// library-default cube::Vessel / cube::Device. That is not a lab-only
// shortcut: the live projector (detections_to_pointcloud) sets only
// `ellipsoidal_referenced` and the two range-error parameters from ROS
// parameters, and BizzyBoat's config sets only frame names — so the boat,
// import_bag, batch_regen_bag, bag_to_geotiff and this lab all run the same
// defaults. Matching them is what keeps the lab's surface comparable with the
// store's. The general fix — a survey-configuration record the live node
// publishes and every offline tool reads — is unh_marine_autonomy#385, with
// cube_bathymetry#145 as its consumer half. See offline_projection_caveat().

namespace marine_perception_tools
{

/// Near-field range gate for the offline projection, metres.
///
/// cube::ProjectorParams defaults `minimum_range` to 0.0, which lets a beam of
/// exactly zero range through as a sounding sitting at the sonar head with a
/// spurious 0 m depth. 0.05 m is small enough that no genuine M3 return is
/// filtered (its practical near-field range is tens of centimetres or more).
///
/// It equals cube::Device::range_error_floor_m only coincidentally — the two
/// are unrelated gates, and neither derives from the other.
///
/// This is a small, deliberate divergence from production: the live projector
/// and the three offline cube tools run the library default 0.0, so this lab
/// filters a sliver of range they do not. Recorded rather than hidden; the
/// value should eventually live once, for all paths, in unh_marine_autonomy#385's
/// configuration record.
constexpr double kOfflineMinimumRangeM = 0.05;

/// ProjectorParams for the offline CUBE-lab path: the caller's frame names,
/// kOfflineMinimumRangeM, and library-default vessel/device (see the file
/// comment and offline_projection_caveat()).
cube::ProjectorParams offline_projector_params(
  const std::string & base_link_frame,
  const std::string & level_frame,
  const std::string & tide_frame);

/// One ping's worth of projected soundings, plus the counts the caller reports.
struct PingProjection
{
  /// Sonar-frame soundings, carrying real cube::ErrorModel variances.
  std::vector<MbesSounding> soundings;

  /// The projector's own per-ping diagnostics, passed through unchanged.
  cube::ProjectionDiagnostics diagnostics;

  /// 1 when the whole ping was dropped for a non-positive sound speed.
  std::size_t invalid_pings = 0;

  /// Soundings dropped for a non-positive or non-finite slant range — the
  /// index-free equivalent of the `twtt <= 0` guard project_detections had.
  std::size_t invalid_beams = 0;
};

/// Project one detections ping through `projector`, converting each
/// cube::Sounding to an MbesSounding.
///
/// THE VALIDITY GUARD, and why cube's range gate does not replace it.
/// `project_detections` (mbes_geometry.hpp) skipped a beam when `twtt <= 0` OR
/// `sound_speed <= 0`. cube's gate instead tests `range^2 ∈ [min^2, max^2]`, so
/// a NEGATIVE travel time or sound speed produces a mirrored sounding whose
/// SQUARED range passes the gate. So the guard is applied here:
///
///   * `ping_info.sound_speed <= 0` (or non-finite) drops the whole ping and
///     counts one `invalid_pings` — every beam of that ping is unusable.
///   * a returned sounding whose `slant_range` is non-positive or non-finite
///     is dropped and counted in `invalid_beams`. cube::Sounding::slant_range
///     is `twtt * sound_speed / 2`, so with a positive sound speed this is
///     exactly the retired `twtt <= 0` test — and it is applied BY VALUE, not
///     by index: DetectionsProjector::project applies its range gate before
///     returning, so sounding index != beam index whenever any beam was
///     filtered, which with kOfflineMinimumRangeM > 0 and NaN rx_angles is the
///     normal case.
///
/// `vessel_speed_mps` is speed-over-ground; the offline path has no odometry
/// source and passes NaN (the error model floors it to 0, which makes the
/// horizontal variance optimistic — see offline_projection_caveat()).
PingProjection project_ping(
  const cube::DetectionsProjector & projector,
  const marine_acoustic_msgs::msg::SonarDetections & detections,
  const tf2::BufferCore & tf,
  float vessel_speed_mps);

/// The one operator-facing statement of what the offline projection assumes,
/// in order of consequence. Consumed by BOTH the CUBE-tuning dialog and the
/// lab's load note, so the wording cannot drift between them.
const char * offline_projection_caveat();

}  // namespace marine_perception_tools

#endif  // MBES_PROJECTION_HPP_
