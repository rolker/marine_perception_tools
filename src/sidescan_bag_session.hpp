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
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mbes_geometry.hpp"
#include "sidescan_geometry.hpp"

// Forward-declared so the persistent TF cache stays out of TUs that only consume
// the ping index (mirrors bag_loader.hpp).
namespace tf2 {class BufferCore;}

namespace marine_perception_tools
{

// The three Garmin sidescan channels, in SidescanChannel order (Port, Starboard,
// Down). `kSidescanTopics` are the recorded ping topics. Verified against
// bizzyboat_sonar/2026-06-18T19-39-06+00-00. Pose comes from the boat (base_frame)
// heading + the channel's port/starboard sign, not a per-sensor-frame TF lookup.
inline constexpr int kNumSidescanChannels = 3;
inline constexpr std::array<const char *, kNumSidescanChannels> kSidescanTopics{
  "/bizzy/sensors/sidescan/garmin_sidescan/sonar_image_port",
  "/bizzy/sensors/sidescan/garmin_sidescan/sonar_image_starboard",
  "/bizzy/sensors/sidescan/garmin_sidescan/sonar_image_down"};

// The Garmin driver's bottom-tracked nadir depth (sensor_msgs/Range, height above
// bottom). This is the authoritative altitude source; the amplitude-based
// estimator (sidescan_geometry.hpp) is only a fallback for bags/sonars that do
// not publish it.
inline constexpr const char * kNadirDepthTopic =
  "/bizzy/sensors/sidescan/garmin_sidescan/nadir_depth";

// The M3 multibeam detections topic (marine_acoustic_msgs/SonarDetections, frame
// bizzy/m3). The bag carries detections only (no soundings cloud), so the viewer
// projects soundings itself via mbes_geometry.hpp. Optional: a bag without it
// still loads (sidescan-only).
inline constexpr const char * kMbesDetectionsTopic = "/bizzy/sensors/m3/detections";

// One ping's lightweight index entry: channel, time, the along-track distance of
// the boat when it was transmitted, and the world-plane geometry needed to
// project it (pose + acoustic scale + resolved altitude). The sample data is NOT
// held here — it is re-read per window by readWindow(), so the resident index
// stays small regardless of bag length. `has_pose` is false when TF could not
// resolve the sensor pose at the ping stamp (kept for the timeline, unpaintable).
struct SidescanPing
{
  SidescanChannel channel = SidescanChannel::Port;
  double stamp_s = 0.0;
  int64_t stamp_ns = 0;             // exact header stamp, for windowed re-read matching
  double cumulative_distance_m = 0.0;
  bool has_pose = false;
  PingGeometry geometry;            // altitude filled from the nearest nadir ping
  uint32_t samples_per_beam = 0;    // sample count (range extent without sample data)
  double sound_speed = 0.0;         // m/s (from ping_info; retained for display)
  double sample_rate = 0.0;         // Hz
};

// A ping together with its sample data, produced on demand by readWindow(): the
// heavy part (beam-0 amplitudes, normalized [0,1]) is read from the bag for the
// requested window only.
struct WindowPing
{
  SidescanChannel channel = SidescanChannel::Port;
  double cumulative_distance_m = 0.0;
  PingGeometry geometry;
  std::vector<float> amplitudes;
};

// One M3 detections ping's lightweight index entry: stamp, along-track distance
// (shared scrub axis), and the full world<-m3 transform captured at the ping
// stamp during the load pass (translation + quaternion, so soundings lift to the
// world frame at read time with no TF). Full 3D — unlike the planar sidescan
// pose, this carries the sensor roll/pitch/z that bathymetry needs. Sample data
// is NOT held; readMbesWindow() re-reads the window. `has_pose` is false when the
// transform or the distance mapping could not be resolved (unscrubable).
struct MbesPing
{
  double stamp_s = 0.0;
  int64_t stamp_ns = 0;
  double cumulative_distance_m = 0.0;
  bool has_pose = false;
  double tx = 0.0, ty = 0.0, tz = 0.0;            // world<-m3 translation (m)
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;  // world<-m3 rotation
};

// An M3 ping's window data: the raw per-beam backscatter (for the backscatter
// waterfall, across-track = beam index) and the valid-detection soundings already
// lifted to the WORLD frame (MbesSounding x/y/z in world metres, for the 3D
// cloud). Produced on demand by readMbesWindow().
struct MbesWindowPing
{
  double cumulative_distance_m = 0.0;
  std::vector<float> intensities;             // per-beam dB (waterfall row)
  std::vector<MbesSounding> world_soundings;  // valid beams, WORLD frame + dB
};

struct SidescanBagOptions
{
  std::string world_frame = "bizzy/map";     // local-tangent ENU render frame
  std::string base_frame = "bizzy/base_link";  // boat pose / heading reference
  std::string geo_frame = "earth";           // earth->world presence => geo export OK
  double altitude_threshold_frac = 0.5;      // nadir first-return detection level
  double altitude_max_dt_s = 2.0;            // max time gap to trust a nadir depth
  // Pose resolution is O(n_pings) only via a coarse boat-pose table sampled with a
  // BOUNDED TF cache, then interpolated per ping (no per-ping TF lookup). This is
  // what keeps load time linear on multi-hour surveys.
  double tf_cache_s = 30.0;                  // bounded tf2 cache window
  double pose_sample_dt_s = 0.2;             // boat-pose sampling cadence (~5 Hz)
  double pose_max_gap_s = 2.0;               // don't interpolate a ping across a bigger gap
};

// Opens a sidescan bag once and builds a lightweight in-memory index: the full TF
// cache, and per ping its channel, stamp, along-track distance (accumulated from
// base_link), resolved sensor pose, and height-above-bottom (from the driver's
// nadir_depth, or a transient amplitude estimate). The bulky sample data is NOT
// resident — `readWindow()` re-reads just the requested distance window's samples
// from the bag (via Reader::seek), so memory stays bounded by the window, not the
// survey length. `window()` returns the lightweight index slice.
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

  // Paintable pings (port + starboard by default) whose along-track distance lies
  // in [dist_lo, dist_hi], WITH their sample data read fresh from the bag. The
  // resident index holds no samples, so this is what bounds memory to the window.
  // `max_pings > 0` keeps only the most recent `max_pings` (stationary cap, so a
  // stopped boat re-reads only a bounded set). Returned in along-track order.
  std::vector<WindowPing> readWindow(
    double dist_lo, double dist_hi, int max_pings = 0, bool include_down = false) const;

  // All M3 detection pings, ordered by stamp, with cumulative distance assigned
  // (empty when the bag had no detections topic).
  const std::vector<MbesPing> & mbesPings() const {return mbes_pings_;}

  // M3 detection pings whose along-track distance lies in [dist_lo, dist_hi],
  // with sample data read fresh and projected: per ping the raw per-beam
  // backscatter (waterfall) + the valid soundings in the WORLD frame (3D cloud).
  // Shares the sidescan distance axis, so one scrub drives both. `max_pings > 0`
  // applies the same stationary cap as readWindow.
  std::vector<MbesWindowPing> readMbesWindow(
    double dist_lo, double dist_hi, int max_pings = 0) const;

  // Total along-track distance of the recording (metres).
  double totalDistance() const {return total_distance_m_;}

  // Bag time (stamp, seconds) of the ping nearest the given along-track distance —
  // used to stamp a marked contact with its real observation time. 0 if empty.
  double timeAtDistance(double dist_m) const;

  std::size_t channelCount(SidescanChannel ch) const;
  std::size_t posesResolved() const {return poses_resolved_;}
  std::size_t posesSkipped() const {return poses_skipped_;}

  // True when earth->world_frame resolved at least once, i.e. marked contacts can
  // be exported to geographic coordinates without a datum fallback.
  bool hasGeoReference() const {return has_geo_reference_;}

  // Convert a world (map-frame) point to WGS84 geodetic (lat/lon degrees, altitude
  // metres) using the captured earth->world transform. Returns false when no geo
  // reference resolved during load.
  bool mapToGeo(double x, double y, double & lat_deg, double & lon_deg, double & alt) const;

  // True when ping altitudes came from the driver's nadir_depth Range topic;
  // false means the amplitude-based estimator fallback was used (or no altitude
  // source was available at all).
  bool usedNadirDepth() const {return used_nadir_depth_;}

  // Count of bag messages that failed to deserialize and were skipped (one bad
  // message does not abort the load). A large value signals a corrupt bag.
  std::size_t decodeErrors() const {return decode_errors_;}

private:
  std::string bag_uri_;
  std::vector<SidescanPing> pings_;
  std::vector<MbesPing> mbes_pings_;
  std::unique_ptr<tf2::BufferCore> tf_buffer_;
  double total_distance_m_ = 0.0;
  std::size_t poses_resolved_ = 0;
  std::size_t poses_skipped_ = 0;
  std::size_t decode_errors_ = 0;
  bool has_geo_reference_ = false;
  bool used_nadir_depth_ = false;

  // Captured earth<-world transform (geo_frame <- world_frame): ECEF translation +
  // quaternion, used by mapToGeo(). Valid only when has_geo_reference_.
  double geo_tx_ = 0.0;
  double geo_ty_ = 0.0;
  double geo_tz_ = 0.0;
  double geo_qx_ = 0.0;
  double geo_qy_ = 0.0;
  double geo_qz_ = 0.0;
  double geo_qw_ = 1.0;
};

}  // namespace marine_perception_tools

#endif  // SIDESCAN_BAG_SESSION_HPP_
