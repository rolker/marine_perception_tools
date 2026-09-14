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

#include "mbes_projection.hpp"

#include <cmath>
#include <string>

namespace marine_perception_tools
{

cube::ProjectorParams offline_projector_params(
  const std::string & base_link_frame,
  const std::string & level_frame,
  const std::string & tide_frame)
{
  cube::ProjectorParams params;
  params.base_link_frame = base_link_frame;
  params.level_frame = level_frame;
  params.tide_frame = tide_frame;
  params.minimum_range = kOfflineMinimumRangeM;
  // vessel / device deliberately left default-constructed: there is no offline
  // configuration record to read them from, and the live projector plus the
  // three offline cube tools run the same defaults (see the header).
  return params;
}

PingProjection project_ping(
  const cube::DetectionsProjector & projector,
  const marine_acoustic_msgs::msg::SonarDetections & detections,
  const tf2::BufferCore & tf,
  float vessel_speed_mps)
{
  PingProjection out;

  // Ping-level guard: a non-positive or non-finite sound speed scales every
  // beam of this ping, so none of them is usable. cube's range gate would let
  // a NEGATIVE sound speed through as a mirrored sounding with a passing
  // squared range.
  const double sound_speed = detections.ping_info.sound_speed;
  if (!std::isfinite(sound_speed) || sound_speed <= 0.0) {
    out.invalid_pings = 1;
    return out;
  }

  const cube::ProjectionResult result =
    projector.project(detections, tf, vessel_speed_mps);
  out.diagnostics = result.diagnostics;

  out.soundings.reserve(result.soundings.size());
  for (const auto & cs : result.soundings) {
    // Per-beam guard, BY VALUE: the projector's range gate has already
    // reordered/shortened the vector, so a beam index cannot be recovered.
    // slant_range is twtt * sound_speed / 2, so with the positive sound speed
    // checked above this is exactly the retired `twtt <= 0` test.
    if (!std::isfinite(cs.slant_range) || cs.slant_range <= 0.0f) {
      ++out.invalid_beams;
      continue;
    }
    MbesSounding s;
    s.x = cs.sonar_relative_position.x;
    s.y = cs.sonar_relative_position.y;
    s.z = cs.sonar_relative_position.z;
    s.intensity = cs.intensity;
    s.beam_angle = cs.beam_angle;
    s.slant_range = cs.slant_range;
    // THE ONE RENAME SITE (#55, leading cube_bathymetry#158): cube::Sounding
    // spells these two variances `*_error`; MbesSounding names them for what
    // they are. No other file translates between the two spellings.
    s.vertical_variance = cs.vertical_error;
    s.horizontal_variance = cs.horizontal_error;
    out.soundings.push_back(s);
  }
  return out;
}

const char * offline_projection_caveat()
{
  return
    "Real cube::ErrorModel uncertainty, on LIBRARY-DEFAULT vessel and device "
    "settings — there is no offline survey configuration to read "
    "(unh_marine_autonomy#385). In order of consequence: (1) a generic 2 m "
    "GPS drms is assumed for every sounding, so the horizontal variance never "
    "falls below 4 m² — the same assumption the live boat, import_bag, "
    "batch_regen_bag and bag_to_geotiff all run, so this lab agrees with the "
    "store rather than inventing a tighter number. That floor does NOT smear "
    "the surface: Parameters::influenceRadius SUBTRACTS the 99% horizontal "
    "term from the depth-budget term and floors what is left at the cell "
    "size, so at the cell sizes this lab runs each sounding's influence "
    "radius is one cell — the ~5.15 m radius that 4 m² allows is a CEILING, "
    "reached only at coarse cells under a loose vertical budget; (2) lever "
    "arms are zero and "
    "the device is generic, so the M3's across-track beamwidth falls back to "
    "the device default (kongsberg_em_bridge reports no per-beam beamwidths, "
    "so this applies to every beam); (3) speed over ground is unavailable "
    "offline and is passed as NaN, which the error model floors to zero — the "
    "speed-dependent latency terms therefore drop out and the horizontal "
    "error is OPTIMISTIC for a moving vessel. It does not correct the "
    "refraction smile: that is a systematic error (mpt#28).";
}

}  // namespace marine_perception_tools
