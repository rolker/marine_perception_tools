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
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "marine_acoustic_msgs/msg/raw_sonar_image.hpp"

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
  bool tf_ok = false;     // world<-m3 transform captured (distance may still be pending)
  bool has_pose = false;  // fully resolved (tf_ok AND distance assigned) -> selectable
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
  // M3 sensor world pose for the across-track projection + pixel->map marking:
  // planar position + heading (ENU, CCW from +x/east). has_pose mirrors the source
  // MbesPing (always true for a windowed ping, which is pre-filtered on has_pose).
  double sensor_x = 0.0;
  double sensor_y = 0.0;
  double heading = 0.0;
  bool has_pose = false;
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

// An immutable snapshot of the resolved index, published periodically by the
// background indexer (buildIndex). Readers take a shared_ptr and use it lock-free;
// the indexer swaps in a newer (more-complete) snapshot as the bag resolves. `pings`
// / `mbes_pings` are stamp-sorted with cumulative distance assigned. Pings still
// unresolved in an in-progress build carry has_pose=false and are skipped by readers.
struct SessionIndex
{
  std::vector<SidescanPing> pings;
  std::vector<MbesPing> mbes_pings;
  double total_distance_m = 0.0;   // resolved-so-far distance (== final when complete)
  std::size_t poses_resolved = 0;
  std::size_t poses_skipped = 0;
  std::size_t decode_errors = 0;
  bool has_geo_reference = false;
  bool used_nadir_depth = false;
  bool complete = false;           // true once indexing has finished
  // Captured earth<-world transform (geo_frame <- world_frame), for mapToGeo().
  double geo_tx = 0.0;
  double geo_ty = 0.0;
  double geo_tz = 0.0;
  double geo_qx = 0.0;
  double geo_qy = 0.0;
  double geo_qz = 0.0;
  double geo_qw = 1.0;
};

class SidescanBagSession
{
public:
  explicit SidescanBagSession(
    const std::string & bag_uri, const SidescanBagOptions & opts = {});
  ~SidescanBagSession();

  SidescanBagSession(const SidescanBagSession &) = delete;
  SidescanBagSession & operator=(const SidescanBagSession &) = delete;

  // Build the in-memory index. Heavy (streams the whole bag); call once, on a worker
  // thread. Publishes a SessionIndex snapshot periodically so readers can use the
  // already-resolved part while the rest indexes; `progress(resolved_distance, done)`
  // fires on each publish (from the calling thread). After it returns, the index is
  // complete. Re-reads from the bag (readWindow etc.) and all accessors are safe to
  // call concurrently from other threads while this runs — they read a lock-free
  // immutable snapshot.
  using ProgressFn = std::function<void(double resolved_distance_m, bool complete)>;
  void buildIndex(const ProgressFn & progress = {});

  // The current immutable index snapshot (lock-free for the caller once held). Null
  // until buildIndex publishes its first snapshot.
  std::shared_ptr<const SessionIndex> snapshot() const;

  // Posed pings whose along-track distance lies in [dist_lo, dist_hi] (metres). A
  // negative dist_hi means "to the end of the track". Returned by value (snapshot).
  std::vector<SidescanPing> window(double dist_lo, double dist_hi) const;

  // Paintable pings (port + starboard by default) whose along-track distance lies
  // in [dist_lo, dist_hi], WITH their sample data read fresh from the bag. The
  // resident index holds no samples, so this is what bounds memory to the window.
  // `max_pings > 0` keeps only the most recent `max_pings` (stationary cap, so a
  // stopped boat re-reads only a bounded set). Returned in along-track order.
  std::vector<WindowPing> readWindow(
    double dist_lo, double dist_hi, int max_pings = 0, bool include_down = false) const;

  // M3 detection pings whose along-track distance lies in [dist_lo, dist_hi],
  // with sample data read fresh and projected: per ping the raw per-beam
  // backscatter (waterfall) + the valid soundings in the WORLD frame (3D cloud).
  // Shares the sidescan distance axis, so one scrub drives both. `max_pings > 0`
  // applies the same stationary cap as readWindow.
  std::vector<MbesWindowPing> readMbesWindow(
    double dist_lo, double dist_hi, int max_pings = 0) const;

  // The down-channel (water-column) pings in [dist_lo, dist_hi] as raw
  // RawSonarImage messages, in along-track order, for the echogram pane. Re-read
  // from the bag like readWindow; `max_pings > 0` applies the stationary cap.
  std::vector<marine_acoustic_msgs::msg::RawSonarImage> readDownImages(
    double dist_lo, double dist_hi, int max_pings = 0) const;

  // Total along-track distance resolved so far (metres); the final length once the
  // index is complete.
  double totalDistance() const;

  // Bag time (stamp, seconds) of the ping nearest the given along-track distance —
  // used to stamp a marked contact with its real observation time. 0 if empty.
  double timeAtDistance(double dist_m) const;

  // Cumulative along-track distance (m) of the posed ping nearest a map position —
  // maps a clicked/hovered world point onto the scrub axis. False if no posed pings.
  bool nearestTrackDistance(double map_x, double map_y, double & dist_m) const;

  // Map position (sensor x/y) of the posed ping nearest an along-track distance —
  // maps an along-track (echogram) position back to a world point. False if empty.
  bool positionAtDistance(double dist_m, double & map_x, double & map_y) const;

  std::size_t channelCount(SidescanChannel ch) const;
  std::size_t posesResolved() const;
  std::size_t posesSkipped() const;

  // Number of indexed pings so far (snapshot). For the status line.
  std::size_t pingCount() const;

  // The boat track (posed pings' sensor x/y) as of the current snapshot — for the
  // map track polyline; grows as indexing proceeds.
  std::vector<std::pair<double, double>> trackPoints() const;

  // True when earth->world_frame resolved at least once, i.e. marked contacts can
  // be exported to geographic coordinates without a datum fallback.
  bool hasGeoReference() const;

  // Convert a world (map-frame) point to WGS84 geodetic (lat/lon degrees, altitude
  // metres) using the captured earth->world transform. Returns false when no geo
  // reference resolved during load.
  bool mapToGeo(double x, double y, double & lat_deg, double & lon_deg, double & alt) const;

  // True when ping altitudes came from the driver's nadir_depth Range topic;
  // false means the amplitude-based estimator fallback was used (or no altitude
  // source was available at all).
  bool usedNadirDepth() const;

  // Count of bag messages that failed to deserialize and were skipped (one bad
  // message does not abort the load). A large value signals a corrupt bag.
  std::size_t decodeErrors() const;

  // True once buildIndex has finished (the snapshot is the complete index).
  bool indexComplete() const;

private:
  // Publish a copy of the current working index as a new immutable snapshot.
  void publishSnapshot(bool complete);

  std::string bag_uri_;
  SidescanBagOptions opts_;

  // --- Worker-thread-only working storage (mutated solely by buildIndex). ---
  std::vector<SidescanPing> pings_;
  std::vector<MbesPing> mbes_pings_;
  std::unique_ptr<tf2::BufferCore> tf_buffer_;
  double total_distance_m_ = 0.0;
  std::size_t poses_resolved_ = 0;
  std::size_t poses_skipped_ = 0;
  std::size_t decode_errors_ = 0;
  bool has_geo_reference_ = false;
  bool used_nadir_depth_ = false;
  double geo_tx_ = 0.0;
  double geo_ty_ = 0.0;
  double geo_tz_ = 0.0;
  double geo_qx_ = 0.0;
  double geo_qy_ = 0.0;
  double geo_qz_ = 0.0;
  double geo_qw_ = 1.0;

  // --- Published snapshot (the only state shared with reader threads). ---
  mutable std::mutex snap_mutex_;
  std::shared_ptr<const SessionIndex> snap_;
};

}  // namespace marine_perception_tools

#endif  // SIDESCAN_BAG_SESSION_HPP_
