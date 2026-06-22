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

#ifndef SIDESCAN_BAG_SESSION_HPP_
#define SIDESCAN_BAG_SESSION_HPP_

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "sidescan_geometry.hpp"

// Forward-declared so the persistent TF cache stays out of TUs that only consume
// the ping index (mirrors bag_loader.hpp).
namespace tf2 {class BufferCore;}

namespace marine_perception_tools
{

// The three Garmin sidescan channels, in SidescanChannel order (Port, Starboard,
// Down). `kSidescanTopics` are the recorded topics; `kSidescanSensorFrames` are
// the TF frames each ping's pose is resolved in. Verified against
// bizzyboat_sonar/2026-06-18T19-39-06+00-00.
inline constexpr int kNumSidescanChannels = 3;
inline constexpr std::array<const char *, kNumSidescanChannels> kSidescanTopics{
  "/bizzy/sensors/sidescan/garmin_sidescan/sonar_image_port",
  "/bizzy/sensors/sidescan/garmin_sidescan/sonar_image_starboard",
  "/bizzy/sensors/sidescan/garmin_sidescan/sonar_image_down"};
inline constexpr std::array<const char *, kNumSidescanChannels> kSidescanSensorFrames{
  "bizzy/garmin_sidescan_port",
  "bizzy/garmin_sidescan_starboard",
  "bizzy/garmin_sidescan_down"};

// One ingested ping: its channel, time, the along-track distance of the boat when
// it was transmitted, the world-plane geometry needed to project it (pose +
// acoustic scale + resolved altitude), and the beam-0 sample amplitudes
// normalized to [0, 1]. `has_pose` is false when TF could not resolve the sensor
// pose at the ping stamp (the ping is kept for the timeline but cannot be
// painted).
struct SidescanPing
{
  SidescanChannel channel = SidescanChannel::Port;
  double stamp_s = 0.0;
  double cumulative_distance_m = 0.0;
  bool has_pose = false;
  PingGeometry geometry;            // altitude filled from the nearest nadir ping
  std::vector<float> amplitudes;    // beam-0 samples, normalized [0,1]
  double sound_speed = 0.0;         // m/s (from ping_info; retained for display)
  double sample_rate = 0.0;         // Hz
};

struct SidescanBagOptions
{
  std::string world_frame = "bizzy/map";     // local-tangent ENU render frame
  std::string base_frame = "bizzy/base_link";  // along-track distance reference
  std::string geo_frame = "earth";           // earth->world presence => geo export OK
  double altitude_threshold_frac = 0.5;      // nadir first-return detection level
};

// Opens a sidescan bag once, builds the full TF cache, then reads every ping into
// an in-memory index: each ping's sensor pose is resolved from TF in the world
// frame, its samples decoded + normalized, the boat's along-track distance
// accumulated from base_link, and its height-above-bottom taken from the nearest
// nadir ping. `window()` then slices the index by along-track distance — the
// distance-buffered re-read policy (the distance analogue of buffer_policy.hpp)
// is a later milestone; PR1 builds the whole index and filters it.
//
// Throws std::runtime_error if the bag has no sidescan ping topics at all.
class SidescanBagSession
{
public:
  explicit SidescanBagSession(
    const std::string & bag_uri, const SidescanBagOptions & opts = {});
  ~SidescanBagSession();

  SidescanBagSession(const SidescanBagSession &) = delete;
  SidescanBagSession & operator=(const SidescanBagSession &) = delete;

  // All pings, ordered by stamp, with cumulative along-track distance assigned.
  const std::vector<SidescanPing> & pings() const {return pings_;}

  // Pings whose along-track distance lies in [dist_lo, dist_hi] (metres). A
  // negative dist_hi means "to the end of the track". Returned in index order.
  std::vector<const SidescanPing *> window(double dist_lo, double dist_hi) const;

  // Total along-track distance of the recording (metres).
  double totalDistance() const {return total_distance_m_;}

  std::size_t channelCount(SidescanChannel ch) const;
  std::size_t posesResolved() const {return poses_resolved_;}
  std::size_t posesSkipped() const {return poses_skipped_;}

  // True when earth->world_frame resolved at least once, i.e. marked contacts can
  // be exported to geographic coordinates without a datum fallback.
  bool hasGeoReference() const {return has_geo_reference_;}

private:
  std::vector<SidescanPing> pings_;
  std::unique_ptr<tf2::BufferCore> tf_buffer_;
  double total_distance_m_ = 0.0;
  std::size_t poses_resolved_ = 0;
  std::size_t poses_skipped_ = 0;
  bool has_geo_reference_ = false;
};

}  // namespace marine_perception_tools

#endif  // SIDESCAN_BAG_SESSION_HPP_
