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

#ifndef SIDESCAN_GEOMETRY_HPP_
#define SIDESCAN_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

// Pure sidescan projection geometry — no Qt, no ROS, so the breakable math is
// unit-testable in isolation (mirrors buffer_policy.hpp). Everything here maps a
// single ping's samples from acoustic (slant-range) space into the planar world
// frame (bizzy/map, local-tangent ENU metres); the bag session resolves the
// per-ping pose from TF and feeds it in as a PingGeometry.

namespace marine_perception_tools
{

// Which transducer a ping came from. Port ensonifies to the sensor's left,
// starboard to its right; the nadir/down channel looks straight down and is used
// only to estimate height-above-bottom (it is never painted as a lateral swath).
enum class SidescanChannel : uint8_t
{
  Port = 0,
  Starboard = 1,
  Down = 2,
};

// Across-track sign for a channel in the world plane, relative to "left of
// heading" (+1 = port/left, -1 = starboard/right, 0 = nadir/no lateral throw).
inline int channel_lateral_sign(SidescanChannel ch)
{
  switch (ch) {
    case SidescanChannel::Port: return +1;
    case SidescanChannel::Starboard: return -1;
    case SidescanChannel::Down: return 0;
  }
  return 0;
}

// One-way metres of range per sample for a given sound speed and sample rate.
// A ping samples the two-way return at `sample_rate` Hz, so each sample step is
// sound_speed / (2 * sample_rate) of one-way (slant) range. Returns 0 for
// non-positive sample_rate (caller treats a 0 step as "geometry unavailable").
inline double slant_metres_per_sample(double sound_speed, double sample_rate)
{
  if (sample_rate <= 0.0) {return 0.0;}
  return sound_speed / (2.0 * sample_rate);
}

// Slant range (one-way, metres) of data-array sample index `i`. `sample0` is the
// RawSonarImage upper gate (the sample number the data array starts at), so the
// data element `i` corresponds to acoustic sample `sample0 + i`.
inline double slant_range_at(std::size_t i, uint32_t sample0, double metres_per_sample)
{
  return (static_cast<double>(sample0) + static_cast<double>(i)) * metres_per_sample;
}

// Horizontal (ground) range for a slant range over a flat bottom, given the
// transducer's height above that bottom. Samples whose slant range is shorter
// than the altitude are in the water column above the seabed return — there is no
// ground intersection, so they project to range 0 and are flagged invalid by the
// caller. Negative/zero altitude (unknown) degrades gracefully to ground == slant.
inline double ground_range(double slant, double altitude)
{
  if (!(altitude > 0.0)) {return slant;}        // unknown altitude → flat (slant≈ground)
  const double d2 = slant * slant - altitude * altitude;
  return (d2 > 0.0) ? std::sqrt(d2) : 0.0;
}

// A ping's pose + acoustic scale, everything project_sample() needs. The pose is
// the *sensor* frame resolved in the world plane (so the transducer lever-arm is
// already baked in); `yaw` is the sensor heading in the world frame (ENU, radians
// CCW from +x/east). `lateral_sign` comes from the channel.
struct PingGeometry
{
  double sensor_x = 0.0;
  double sensor_y = 0.0;
  double yaw = 0.0;                 // world-frame heading, rad (ENU, CCW from +x)
  uint32_t sample0 = 0;
  double metres_per_sample = 0.0;   // one-way slant metres per sample
  double altitude = 0.0;            // sensor height above bottom, m (<=0 == unknown)
  int lateral_sign = 0;             // +1 port, -1 starboard, 0 nadir
};

// A projected sample in the world plane.
struct GroundPoint
{
  double x = 0.0;
  double y = 0.0;
  double ground_range = 0.0;   // horizontal distance from the sensor, m
  bool valid = false;          // false == within the nadir gap (slant <= altitude)
};

// Project data-array sample `i` of a ping into the world plane. The across-track
// direction is perpendicular to the sensor heading: "left of heading" is
// (-sin yaw, cos yaw), and the channel's lateral sign places the sample on the
// correct side. A nadir channel (sign 0) has no lateral throw, so every sample
// lands at the sensor position (valid only past the first bottom return).
inline GroundPoint project_sample(const PingGeometry & g, std::size_t i)
{
  GroundPoint p;
  const double slant = slant_range_at(i, g.sample0, g.metres_per_sample);
  // A sample is only a real seabed hit once its slant range clears the altitude
  // (the first bottom return). Inside that gap there is no ground intersection.
  if (g.altitude > 0.0 && slant <= g.altitude) {
    return p;  // valid == false
  }
  const double gr = ground_range(slant, g.altitude);
  const double left_x = -std::sin(g.yaw);
  const double left_y = std::cos(g.yaw);
  const double s = static_cast<double>(g.lateral_sign);
  p.x = g.sensor_x + s * gr * left_x;
  p.y = g.sensor_y + s * gr * left_y;
  p.ground_range = gr;
  p.valid = true;
  return p;
}

// Estimate the transducer's height above the bottom from a nadir (down-channel)
// ping: the slant range of the first sample whose amplitude crosses
// `threshold_frac` of the ping's peak amplitude, ignoring the first `min_gate`
// samples (transmit/near-field ringing). For a straight-down beam the first
// bottom return's slant range *is* the height above bottom. Returns NaN when no
// sample crosses the threshold (e.g. no bottom in range).
inline double estimate_altitude_from_nadir(
  const std::vector<float> & amplitudes, uint32_t sample0, double metres_per_sample,
  double threshold_frac = 0.5, std::size_t min_gate = 1)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (amplitudes.empty() || metres_per_sample <= 0.0) {return nan;}
  float peak = 0.0f;
  for (float a : amplitudes) {
    peak = std::max(peak, a);
  }
  if (!(peak > 0.0f)) {return nan;}
  const double thresh = threshold_frac * static_cast<double>(peak);
  for (std::size_t i = min_gate; i < amplitudes.size(); ++i) {
    if (static_cast<double>(amplitudes[i]) >= thresh) {
      return slant_range_at(i, sample0, metres_per_sample);
    }
  }
  return nan;
}

}  // namespace marine_perception_tools

#endif  // SIDESCAN_GEOMETRY_HPP_
