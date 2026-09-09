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

#include "sidescan_bag_session.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "marine_acoustic_msgs/msg/raw_sonar_image.hpp"
#include "marine_acoustic_msgs/msg/sonar_detections.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "tf2/buffer_core.h"
#include "tf2/time.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "distance_buffer_policy.hpp"
#include "tf_lift.hpp"

namespace marine_perception_tools
{

std::optional<std::pair<double, double>> distance_interval(
  const SessionIndex & index, int64_t t_start_ns, int64_t t_end_ns)
{
  if (t_end_ns < t_start_ns) {std::swap(t_start_ns, t_end_ns);}

  bool found = false;
  double lo = 0.0;
  double hi = 0.0;
  // Both ping vectors are stamp-sorted: binary-search the window start, then
  // walk the in-window range folding posed pings' (monotonic) distances.
  const auto scan = [&](const auto & pings) {
      const auto first = std::lower_bound(
        pings.begin(), pings.end(), t_start_ns,
        [](const auto & ping, int64_t t) {return ping.stamp_ns < t;});
      for (auto it = first; it != pings.end() && it->stamp_ns <= t_end_ns; ++it) {
        if (!it->has_pose) {continue;}
        if (!found) {
          lo = hi = it->cumulative_distance_m;
          found = true;
        } else {
          lo = std::min(lo, it->cumulative_distance_m);
          hi = std::max(hi, it->cumulative_distance_m);
        }
      }
    };
  scan(index.pings);
  scan(index.mbes_pings);

  if (!found) {
    return std::nullopt;
  }
  return std::make_pair(lo, hi);
}

namespace
{

template<typename T>
T deserialize(const rosbag2_storage::SerializedBagMessageSharedPtr & msg)
{
  rclcpp::SerializedMessage serialized(*msg->serialized_data);
  T out;
  rclcpp::Serialization<T>().deserialize_message(&serialized, &out);
  return out;
}

// World-frame yaw (ENU, CCW from +x) from a transform quaternion.
double yaw_from_quaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// One sample of the boat pose (world<-base) plus along-track cumulative distance.
struct BasePoseSample
{
  double t = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double cumdist = 0.0;
};

// Interpolate the boat pose at time `t` from the coarse sample table. Returns false
// when `t` is outside the table's time span or the bracketing samples straddle a
// gap larger than `max_gap` (a TF dropout we shouldn't interpolate across). Yaw is
// interpolated by the shortest angle.
bool interp_base_pose(
  const std::vector<BasePoseSample> & tbl, double t, double max_gap, BasePoseSample & out)
{
  if (tbl.empty()) {return false;}
  if (tbl.size() == 1) {
    if (std::abs(tbl.front().t - t) > max_gap) {return false;}
    out = tbl.front();
    return true;
  }
  if (t < tbl.front().t || t > tbl.back().t) {return false;}
  const auto it = std::upper_bound(
    tbl.begin(), tbl.end(), t,
    [](double s, const BasePoseSample & e) {return s < e.t;});
  if (it == tbl.begin()) {out = tbl.front(); return true;}
  const BasePoseSample & b = *it;
  const BasePoseSample & a = *(it - 1);
  const double gap = b.t - a.t;
  if (gap > max_gap) {return false;}
  const double f = (gap > 1e-9) ? (t - a.t) / gap : 0.0;
  double dyaw = b.yaw - a.yaw;
  while (dyaw > M_PI) {dyaw -= 2.0 * M_PI;}
  while (dyaw < -M_PI) {dyaw += 2.0 * M_PI;}
  out.t = t;
  out.x = a.x + (b.x - a.x) * f;
  out.y = a.y + (b.y - a.y) * f;
  out.yaw = a.yaw + dyaw * f;
  out.cumdist = a.cumdist + (b.cumdist - a.cumdist) * f;
  return true;
}

// lookup_at_or_latest / rotate_by_quat moved to tf_lift.hpp (#21) — shared with
// read_mbes_window()'s one-shot windowed read.

// True when this host is big-endian (runtime check; std::endian is C++20).
bool host_is_big_endian()
{
  const uint16_t one = 1;
  return *reinterpret_cast<const uint8_t *>(&one) == 0;
}

// Decode beam 0 of a RawSonarImage into amplitudes normalized to [0, 1]. The data
// buffer holds `beam_count * samples_per_beam` elements of `dtype`, beam-major, so
// beam 0 is the leading `samples_per_beam` elements. Integer types normalize by
// their full-scale max; float types are passed through (already physical/relative)
// and clamped to [0, 1]. When the message's byte order differs from the host's,
// each multi-byte element is byte-swapped — otherwise a big-endian-recorded bag
// would decode to silent garbage. Unknown dtypes yield an empty vector (ping kept,
// unpaintable).
template<typename T>
std::vector<float> normalize_beam0(const uint8_t * bytes, std::size_t n, double scale, bool swap)
{
  std::vector<float> out(n);
  for (std::size_t i = 0; i < n; ++i) {
    T v;
    if (swap && sizeof(T) > 1) {
      unsigned char tmp[sizeof(T)];
      for (std::size_t k = 0; k < sizeof(T); ++k) {
        tmp[k] = bytes[i * sizeof(T) + (sizeof(T) - 1 - k)];
      }
      std::memcpy(&v, tmp, sizeof(T));
    } else {
      std::memcpy(&v, bytes + i * sizeof(T), sizeof(T));
    }
    out[i] = static_cast<float>(static_cast<double>(v) / scale);
  }
  return out;
}

std::vector<float> extract_beam0(
  const marine_acoustic_msgs::msg::SonarImageData & img,
  uint32_t samples_per_beam)
{
  using marine_acoustic_msgs::msg::SonarImageData;
  const std::size_t n = samples_per_beam;
  if (n == 0) {return {};}
  std::size_t elem_size = 0;
  switch (img.dtype) {
    case SonarImageData::DTYPE_UINT8: elem_size = 1; break;
    case SonarImageData::DTYPE_INT8: elem_size = 1; break;
    case SonarImageData::DTYPE_UINT16: elem_size = 2; break;
    case SonarImageData::DTYPE_INT16: elem_size = 2; break;
    case SonarImageData::DTYPE_UINT32: elem_size = 4; break;
    case SonarImageData::DTYPE_INT32: elem_size = 4; break;
    case SonarImageData::DTYPE_FLOAT32: elem_size = 4; break;
    case SonarImageData::DTYPE_FLOAT64: elem_size = 8; break;
    default: return {};  // unsupported (incl. 64-bit ints) — leave unpaintable
  }
  if (img.data.size() < n * elem_size) {return {};}
  const bool swap = img.is_bigendian != host_is_big_endian();
  const uint8_t * b = img.data.data();
  std::vector<float> out;
  switch (img.dtype) {
    case SonarImageData::DTYPE_UINT8: out = normalize_beam0<uint8_t>(b, n, 255.0, swap); break;
    case SonarImageData::DTYPE_INT8: out = normalize_beam0<int8_t>(b, n, 127.0, swap); break;
    case SonarImageData::DTYPE_UINT16: out = normalize_beam0<uint16_t>(b, n, 65535.0, swap); break;
    case SonarImageData::DTYPE_INT16: out = normalize_beam0<int16_t>(b, n, 32767.0, swap); break;
    case SonarImageData::DTYPE_UINT32:
      out = normalize_beam0<uint32_t>(b, n, 4294967295.0, swap); break;
    case SonarImageData::DTYPE_INT32:
      out = normalize_beam0<int32_t>(b, n, 2147483647.0, swap); break;
    case SonarImageData::DTYPE_FLOAT32: out = normalize_beam0<float>(b, n, 1.0, swap); break;
    case SonarImageData::DTYPE_FLOAT64: out = normalize_beam0<double>(b, n, 1.0, swap); break;
    default: return {};
  }
  for (float & v : out) {
    v = std::clamp(v, 0.0f, 1.0f);
  }
  return out;
}

}  // namespace

SidescanBagSession::SidescanBagSession(
  const std::string & bag_uri, const SidescanBagOptions & opts)
: bag_uri_(bag_uri), opts_(opts)
{
  // Cheap: open + verify the bag has sidescan ping topics, so the caller surfaces a
  // bad bag synchronously. The heavy index build is buildIndex() (run on a worker).
  rosbag2_cpp::Reader reader;
  reader.open(bag_uri);
  const auto & meta = reader.get_metadata();
  std::map<std::string, std::size_t> counts;
  for (const auto & t : meta.topics_with_message_count) {
    counts[t.topic_metadata.name] = t.message_count;
  }
  bool any_channel = false;
  for (int c = 0; c < kNumSidescanChannels; ++c) {
    auto it = counts.find(kSidescanTopics[c]);
    if (it != counts.end() && it->second > 0) {any_channel = true;}
  }
  if (!any_channel) {
    throw std::runtime_error(
      "bag has no Garmin sidescan ping topics (sonar_image_{port,starboard,down})");
  }

  // BOUNDED tf cache. A full-bag cache makes each lookupTransform pay for the whole
  // recording's transform depth, so per-ping pose resolution becomes ~O(n^2) on a
  // multi-hour survey. Instead keep the cache small and sample the boat pose at a
  // coarse cadence as we stream in time order, then interpolate per ping.
  tf_buffer_ = std::make_unique<tf2::BufferCore>(tf2::durationFromSec(opts_.tf_cache_s));
}

void SidescanBagSession::buildIndex(
  const ProgressFn & progress,
  const std::shared_ptr<std::atomic<bool>> & cancel)
{
  // Optional load profiling: set SIDESCAN_PROFILE=1 to print per-phase timings to
  // stderr. Quiet by default.
  const bool profile = std::getenv("SIDESCAN_PROFILE") != nullptr;
  const auto t_start = std::chrono::steady_clock::now();
  auto prof = [profile, t_start](const char * label) {
      if (!profile) {return;}
      const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_start).count();
      std::fprintf(stderr, "[profile] %-22s %8.0f ms\n", label, ms);
    };

  rosbag2_cpp::Reader reader;
  reader.open(bag_uri_);

  std::map<std::string, SidescanChannel> topic_to_channel;
  for (int c = 0; c < kNumSidescanChannels; ++c) {
    topic_to_channel[kSidescanTopics[c]] = static_cast<SidescanChannel>(c);
  }

  // The driver's bottom-tracked nadir depth (stamp_s, height-above-bottom m), the
  // authoritative altitude source. `down_estimates` is the amplitude-derived
  // fallback, computed transiently from the down channel (no samples retained).
  std::vector<std::pair<double, double>> nadir_depths;
  std::vector<std::pair<double, double>> down_estimates;

  // Coarse boat-pose table (world<-base), sampled ~1/pose_sample_dt_s as the TF
  // frontier advances. The lookup lags the frontier by `kMargin` so it always has
  // transforms bracketing the query time (no extrapolation throw).
  std::vector<BasePoseSample> base_table;
  double frontier_s = -std::numeric_limits<double>::infinity();
  double last_sample_s = -std::numeric_limits<double>::infinity();
  const double sample_dt = std::max(0.01, opts_.pose_sample_dt_s);
  constexpr double kMargin = 0.1;

  // Incremental resolve: accumulate the base-table cumulative distance and resolve
  // (pose + along-track distance) every sidescan ping whose stamp is already
  // bracketed by the cumdist-computed part of the table. Idempotent (skips already
  // resolved). Lets each published snapshot expose the part scrubbable so far.
  std::size_t resolved_base = 0;
  auto resolve_incremental = [&]() {
      for (std::size_t i = resolved_base; i < base_table.size(); ++i) {
        base_table[i].cumdist = (i == 0) ? 0.0 :
          base_table[i - 1].cumdist +
          std::hypot(
            base_table[i].x - base_table[i - 1].x, base_table[i].y - base_table[i - 1].y);
      }
      resolved_base = base_table.size();
      if (base_table.empty()) {return;}
      total_distance_m_ = base_table.back().cumdist;
      const double resolvable_t = base_table.back().t;
      for (auto & ping : pings_) {
        if (ping.has_pose || ping.stamp_s > resolvable_t) {continue;}
        BasePoseSample bp;
        if (interp_base_pose(base_table, ping.stamp_s, opts_.pose_max_gap_s, bp)) {
          ping.geometry.sensor_x = bp.x;
          ping.geometry.sensor_y = bp.y;
          ping.geometry.yaw = bp.yaw;
          ping.cumulative_distance_m = bp.cumdist;
          ping.has_pose = true;
          ++poses_resolved_;
        }
      }
      for (auto & mp : mbes_pings_) {
        if (mp.has_pose || !mp.tf_ok || mp.stamp_s > resolvable_t) {continue;}
        BasePoseSample bp;
        if (interp_base_pose(base_table, mp.stamp_s, opts_.pose_max_gap_s, bp)) {
          mp.cumulative_distance_m = bp.cumdist;
          mp.has_pose = true;
        }
      }
    };

  // Publish a snapshot at most every ~1.5 s of wall time (and at least every
  // kFlushMsgs messages, so a fast bag still grows). Intermediate snapshots carry
  // pose+distance only (altitude is assigned authoritatively in the final pass).
  auto last_flush = std::chrono::steady_clock::now();
  constexpr double kFlushS = 1.5;
  std::size_t since_flush = 0;
  constexpr std::size_t kFlushMsgs = 50000;
  auto maybe_flush = [&]() {
      const auto now = std::chrono::steady_clock::now();
      if (++since_flush < kFlushMsgs &&
        std::chrono::duration<double>(now - last_flush).count() < kFlushS)
      {
        return;
      }
      resolve_incremental();
      publishSnapshot(false);
      if (progress) {progress(total_distance_m_, false);}
      last_flush = now;
      since_flush = 0;
    };

  // ---- Single pass: stream the bag in order. ----
  while (reader.has_next()) {
    if (cancel && cancel->load(std::memory_order_relaxed)) {
      return;   // superseded: stop streaming, publish nothing more
    }
    auto bag_msg = reader.read_next();
    maybe_flush();   // grow the published snapshot as the bag streams in
    // One unreadable message (corrupt/truncated) is skipped + counted, not fatal.
    try {
      const std::string & topic = bag_msg->topic_name;
      if (topic == "/tf" || topic == "/tf_static") {
        auto tfm = deserialize<tf2_msgs::msg::TFMessage>(bag_msg);
        const bool is_static = (topic == "/tf_static");
        for (const auto & tr : tfm.transforms) {
          tf_buffer_->setTransform(tr, "bag", is_static);
          if (!is_static) {
            frontier_s = std::max(
              frontier_s, tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9);
          }
        }
        // Sample the boat pose at the coarse cadence (lagging the frontier).
        const double sample_t = frontier_s - kMargin;
        if (std::isfinite(sample_t) && sample_t - last_sample_s >= sample_dt) {
          const auto tp = tf2::TimePoint(
            std::chrono::nanoseconds(static_cast<int64_t>(sample_t * 1e9)));
          try {
            const auto tf =
              tf_buffer_->lookupTransform(opts_.world_frame, opts_.base_frame, tp);
            const auto & tr = tf.transform.translation;
            const auto & q = tf.transform.rotation;
            BasePoseSample s;
            s.t = sample_t;
            s.x = tr.x;
            s.y = tr.y;
            s.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w);
            base_table.push_back(s);
            last_sample_s = sample_t;
            if (!has_geo_reference_ &&
              tf_buffer_->canTransform(opts_.geo_frame, opts_.world_frame, tp))
            {
              // Capture earth<-world (ECEF translation + rotation) for mapToGeo().
              const auto g =
                tf_buffer_->lookupTransform(opts_.geo_frame, opts_.world_frame, tp);
              geo_tx_ = g.transform.translation.x;
              geo_ty_ = g.transform.translation.y;
              geo_tz_ = g.transform.translation.z;
              geo_qx_ = g.transform.rotation.x;
              geo_qy_ = g.transform.rotation.y;
              geo_qz_ = g.transform.rotation.z;
              geo_qw_ = g.transform.rotation.w;
              has_geo_reference_ = true;
            }
          } catch (const tf2::TransformException &) {
            // Chain not resolvable at this time yet; retry on the next cadence tick.
          }
        }
        continue;
      }
      if (topic == kNadirDepthTopic) {
        auto rng = deserialize<sensor_msgs::msg::Range>(bag_msg);
        if (std::isfinite(rng.range)) {
          const double t = rng.header.stamp.sec + rng.header.stamp.nanosec * 1e-9;
          nadir_depths.emplace_back(t, static_cast<double>(rng.range));
        }
        continue;
      }
      if (topic == kMbesDetectionsTopic) {
        // Index the ping (no samples) + capture world<-m3 now, while the bounded
        // TF cache still brackets this stamp. Cumulative distance is assigned in
        // the post-pass loop (needs the accumulated base-table distances).
        auto det = deserialize<marine_acoustic_msgs::msg::SonarDetections>(bag_msg);
        MbesPing mp;
        mp.stamp_s = det.header.stamp.sec + det.header.stamp.nanosec * 1e-9;
        mp.stamp_ns = static_cast<int64_t>(det.header.stamp.sec) * 1000000000LL +
          det.header.stamp.nanosec;
        const std::string src =
          det.header.frame_id.empty() ? std::string("bizzy/m3") : det.header.frame_id;
        const auto tp = tf2::TimePoint(std::chrono::nanoseconds(mp.stamp_ns));
        geometry_msgs::msg::TransformStamped tf;
        if (lookup_at_or_latest(*tf_buffer_, opts_.world_frame, src, tp, tf)) {
          mp.tx = tf.transform.translation.x;
          mp.ty = tf.transform.translation.y;
          mp.tz = tf.transform.translation.z;
          mp.qx = tf.transform.rotation.x;
          mp.qy = tf.transform.rotation.y;
          mp.qz = tf.transform.rotation.z;
          mp.qw = tf.transform.rotation.w;
          mp.tf_ok = true;  // transform captured; distance assigned in the resolve step
        }
        mbes_pings_.push_back(mp);
        continue;
      }
      auto ch_it = topic_to_channel.find(topic);
      if (ch_it == topic_to_channel.end()) {continue;}

      auto img = deserialize<marine_acoustic_msgs::msg::RawSonarImage>(bag_msg);
      SidescanPing ping;
      ping.channel = ch_it->second;
      ping.stamp_s = img.header.stamp.sec + img.header.stamp.nanosec * 1e-9;
      ping.stamp_ns = static_cast<int64_t>(img.header.stamp.sec) * 1000000000LL +
        img.header.stamp.nanosec;
      ping.sound_speed = img.ping_info.sound_speed;
      ping.sample_rate = img.sample_rate;
      ping.samples_per_beam = img.samples_per_beam;
      ping.geometry.sample0 = img.sample0;
      ping.geometry.metres_per_sample =
        slant_metres_per_sample(img.ping_info.sound_speed, img.sample_rate);
      ping.geometry.lateral_sign = channel_lateral_sign(ping.channel);

      // Down channel: derive a fallback altitude now (transient — no samples kept).
      if (ping.channel == SidescanChannel::Down) {
        const auto amps = extract_beam0(img.image, img.samples_per_beam);
        const double alt = estimate_altitude_from_nadir(
          amps, ping.geometry.sample0, ping.geometry.metres_per_sample,
          opts_.altitude_threshold_frac);
        if (std::isfinite(alt)) {down_estimates.emplace_back(ping.stamp_s, alt);}
      }
      pings_.push_back(std::move(ping));
    } catch (const std::exception &) {
      ++decode_errors_;
    }
  }

  std::stable_sort(
    pings_.begin(), pings_.end(),
    [](const SidescanPing & a, const SidescanPing & b) {return a.stamp_s < b.stamp_s;});

  prof("single pass read");

  // ---- Resolve per-ping pose by interpolating the coarse boat-pose table. ----
  // Heading is the boat heading; the channel's port/starboard sign places the
  // swath. No per-ping TF lookup (the expensive part is gone). First accumulate
  // along-track distance along the table.
  for (std::size_t i = 0; i < base_table.size(); ++i) {
    base_table[i].cumdist = (i == 0) ? 0.0 :
      base_table[i - 1].cumdist +
      std::hypot(base_table[i].x - base_table[i - 1].x, base_table[i].y - base_table[i - 1].y);
  }
  total_distance_m_ = base_table.empty() ? 0.0 : base_table.back().cumdist;

  poses_resolved_ = 0;
  poses_skipped_ = 0;
  for (auto & ping : pings_) {
    BasePoseSample bp;
    if (interp_base_pose(base_table, ping.stamp_s, opts_.pose_max_gap_s, bp)) {
      ping.geometry.sensor_x = bp.x;
      ping.geometry.sensor_y = bp.y;
      ping.geometry.yaw = bp.yaw;
      ping.cumulative_distance_m = bp.cumdist;
      ping.has_pose = true;
      ++poses_resolved_;
    } else {
      ping.has_pose = false;
      ++poses_skipped_;
    }
  }

  // M3 detection pings carry the world<-m3 transform (captured in-pass, tf_ok);
  // assign their along-track distance from the same base table so one scrub drives
  // both, and mark them selectable (has_pose). A ping with no transform or no
  // distance mapping is left unscrubable.
  for (auto & mp : mbes_pings_) {
    BasePoseSample bp;
    if (mp.tf_ok && interp_base_pose(base_table, mp.stamp_s, opts_.pose_max_gap_s, bp)) {
      mp.cumulative_distance_m = bp.cumdist;
      mp.has_pose = true;
    } else {
      mp.has_pose = false;
    }
  }
  std::stable_sort(
    mbes_pings_.begin(), mbes_pings_.end(),
    [](const MbesPing & a, const MbesPing & b) {return a.stamp_s < b.stamp_s;});

  prof("pose+distance resolve");

  // ---- Assign height-above-bottom from the nearest nadir sample in time. ----
  // Prefer the driver's bottom-tracked nadir_depth Range; only if that topic is
  // absent fall back to detecting the first bottom return in the down channel's
  // raw amplitudes (fragile near the transmit/near-field ringing — see
  // estimate_altitude_from_nadir).
  struct NadirAlt {double stamp_s; double altitude;};
  std::vector<NadirAlt> nadir;
  if (!nadir_depths.empty()) {
    used_nadir_depth_ = true;
    nadir.reserve(nadir_depths.size());
    for (const auto & [t, range] : nadir_depths) {
      nadir.push_back({t, range});
    }
  } else {
    nadir.reserve(down_estimates.size());
    for (const auto & [t, alt] : down_estimates) {
      nadir.push_back({t, alt});
    }
  }
  std::stable_sort(
    nadir.begin(), nadir.end(),
    [](const NadirAlt & a, const NadirAlt & b) {return a.stamp_s < b.stamp_s;});
  if (!nadir.empty()) {
    for (auto & ping : pings_) {
      // Nearest nadir ping in time (nadir is in stamp order — same source order).
      auto it = std::lower_bound(
        nadir.begin(), nadir.end(), ping.stamp_s,
        [](const NadirAlt & na, double s) {return na.stamp_s < s;});
      double best = 0.0;
      double best_dt = std::numeric_limits<double>::max();
      for (auto cand : {it == nadir.begin() ? it : std::prev(it),
          it == nadir.end() ? std::prev(it) : it})
      {
        const double dt = std::abs(cand->stamp_s - ping.stamp_s);
        if (dt < best_dt) {best_dt = dt; best = cand->altitude;}
      }
      // Only trust the nadir depth if it is close enough in time; across a long
      // nadir dropout, leave altitude unknown (0 -> flat slant≈ground) rather than
      // painting with a stale depth.
      ping.geometry.altitude = (best_dt <= opts_.altitude_max_dt_s) ? best : 0.0;
    }
  }

  // Geo-export readiness (earth -> world) was probed during the streaming pass,
  // while the relevant transforms were still in the bounded cache.
  prof("altitude assign + done");

  // Final authoritative snapshot (stamp-sorted, altitudes assigned).
  publishSnapshot(true);
  if (progress) {progress(total_distance_m_, true);}
}

void buildTrackLookup(SessionIndex & index)
{
  index.track_lookup.clear();
  const SidescanPing * last_posed = nullptr;
  for (const auto & p : index.pings) {
    if (!p.has_pose) {
      continue;
    }
    last_posed = &p;
    if (index.track_lookup.empty() ||
      p.cumulative_distance_m - index.track_lookup.back().cum_dist_m >=
      kTrackLookupStrideM)
    {
      index.track_lookup.push_back(
        {p.geometry.sensor_x, p.geometry.sensor_y, p.cumulative_distance_m});
    }
  }
  // The exact track end, so queries near it don't snap up to a stride short.
  if (last_posed &&
    last_posed->cumulative_distance_m > index.track_lookup.back().cum_dist_m)
  {
    index.track_lookup.push_back(
      {last_posed->geometry.sensor_x, last_posed->geometry.sensor_y,
        last_posed->cumulative_distance_m});
  }
}

void SidescanBagSession::publishSnapshot(bool complete)
{
  // Copy the worker's current working index into a fresh immutable snapshot and swap
  // it in. Readers hold a shared_ptr to the old one until they release it, so the
  // swap is safe without locking the index itself — only the pointer swap is guarded.
  auto idx = std::make_shared<SessionIndex>();
  idx->pings = pings_;
  idx->mbes_pings = mbes_pings_;
  idx->total_distance_m = total_distance_m_;
  idx->poses_resolved = poses_resolved_;
  idx->poses_skipped = poses_skipped_;
  idx->decode_errors = decode_errors_;
  idx->has_geo_reference = has_geo_reference_;
  idx->used_nadir_depth = used_nadir_depth_;
  idx->complete = complete;
  idx->geo_tx = geo_tx_;
  idx->geo_ty = geo_ty_;
  idx->geo_tz = geo_tz_;
  idx->geo_qx = geo_qx_;
  idx->geo_qy = geo_qy_;
  idx->geo_qz = geo_qz_;
  idx->geo_qw = geo_qw_;
  buildTrackLookup(*idx);
  std::lock_guard<std::mutex> lock(snap_mutex_);
  snap_ = std::move(idx);
}

void SidescanBagSession::adoptIndex(SessionIndex index)
{
  index.complete = true;
  buildTrackLookup(index);   // the cache stores pings only, not the lookup
  auto idx = std::make_shared<const SessionIndex>(std::move(index));
  std::lock_guard<std::mutex> lock(snap_mutex_);
  snap_ = std::move(idx);
}

std::shared_ptr<const SessionIndex> SidescanBagSession::snapshot() const
{
  std::lock_guard<std::mutex> lock(snap_mutex_);
  return snap_;
}

double SidescanBagSession::totalDistance() const
{
  const auto s = snapshot();
  return s ? s->total_distance_m : 0.0;
}

std::size_t SidescanBagSession::posesResolved() const
{
  const auto s = snapshot();
  return s ? s->poses_resolved : 0;
}

std::size_t SidescanBagSession::posesSkipped() const
{
  const auto s = snapshot();
  return s ? s->poses_skipped : 0;
}

std::size_t SidescanBagSession::pingCount() const
{
  const auto s = snapshot();
  return s ? s->pings.size() : 0;
}

std::vector<std::pair<double, double>> SidescanBagSession::trackPoints() const
{
  const auto s = snapshot();
  std::vector<std::pair<double, double>> out;
  if (!s) {return out;}
  out.reserve(s->pings.size());
  for (const auto & p : s->pings) {
    if (p.has_pose) {out.emplace_back(p.geometry.sensor_x, p.geometry.sensor_y);}
  }
  return out;
}

bool SidescanBagSession::hasGeoReference() const
{
  const auto s = snapshot();
  return s && s->has_geo_reference;
}

bool SidescanBagSession::usedNadirDepth() const
{
  const auto s = snapshot();
  return s && s->used_nadir_depth;
}

std::size_t SidescanBagSession::decodeErrors() const
{
  const auto s = snapshot();
  return s ? s->decode_errors : 0;
}

bool SidescanBagSession::indexComplete() const
{
  const auto s = snapshot();
  return s && s->complete;
}

SidescanBagSession::~SidescanBagSession() = default;

std::vector<SidescanPing> SidescanBagSession::window(
  double dist_lo, double dist_hi) const
{
  const auto snap = snapshot();
  if (!snap) {return {};}
  const double hi = (dist_hi < 0.0) ? snap->total_distance_m : dist_hi;
  std::vector<SidescanPing> out;
  for (const auto & ping : snap->pings) {
    if (ping.has_pose && ping.cumulative_distance_m >= dist_lo &&
      ping.cumulative_distance_m <= hi)
    {
      out.push_back(ping);
    }
  }
  return out;
}

std::size_t SidescanBagSession::channelCount(SidescanChannel ch) const
{
  const auto snap = snapshot();
  if (!snap) {return 0;}
  std::size_t n = 0;
  for (const auto & ping : snap->pings) {
    if (ping.channel == ch) {
      ++n;
    }
  }
  return n;
}

bool SidescanBagSession::mapToGeo(
  double x, double y, double & lat_deg, double & lon_deg, double & alt) const
{
  const auto snap = snapshot();
  if (!snap || !snap->has_geo_reference) {return false;}
  // Rotate the map point (x, y, 0) by the earth<-world quaternion and translate to
  // ECEF, then convert to geodetic.
  const double qx = snap->geo_qx;
  const double qy = snap->geo_qy;
  const double qz = snap->geo_qz;
  const double qw = snap->geo_qw;
  const double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double r01 = 2.0 * (qx * qy - qz * qw);
  const double r10 = 2.0 * (qx * qy + qz * qw);
  const double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
  const double r20 = 2.0 * (qx * qz - qy * qw);
  const double r21 = 2.0 * (qy * qz + qx * qw);
  const double ex = snap->geo_tx + r00 * x + r01 * y;
  const double ey = snap->geo_ty + r10 * x + r11 * y;
  const double ez = snap->geo_tz + r20 * x + r21 * y;
  ecef_to_geodetic(ex, ey, ez, lat_deg, lon_deg, alt);
  return true;
}

double SidescanBagSession::timeAtDistance(double dist_m) const
{
  const auto snap = snapshot();
  if (!snap || snap->pings.empty()) {return 0.0;}
  // pings are stamp-sorted with non-decreasing cumulative_distance_m (when complete;
  // an in-progress snapshot is approximately ordered).
  const auto it = std::lower_bound(
    snap->pings.begin(), snap->pings.end(), dist_m,
    [](const SidescanPing & p, double d) {return p.cumulative_distance_m < d;});
  return (it == snap->pings.end()) ? snap->pings.back().stamp_s : it->stamp_s;
}

bool SidescanBagSession::nearestTrackDistance(
  double map_x, double map_y, double & dist_m) const
{
  // Hover-rate query (#26): scan the ~1 m-decimated track_lookup, not every
  // ping — the answer is within a stride of exact, well inside the scrub's
  // 1 m granularity.
  const auto snap = snapshot();
  if (!snap || snap->track_lookup.empty()) {return false;}
  bool found = false;
  double best_d2 = 0.0;
  for (const auto & s : snap->track_lookup) {
    const double dx = s.x - map_x;
    const double dy = s.y - map_y;
    const double d2 = dx * dx + dy * dy;
    if (!found || d2 < best_d2) {
      best_d2 = d2;
      dist_m = s.cum_dist_m;
      found = true;
    }
  }
  return found;
}

bool SidescanBagSession::positionAtDistance(
  double dist_m, double & map_x, double & map_y) const
{
  // Hover-rate query (#26): the lookup is cumulative-distance-ordered by
  // construction, so binary-search it and take the nearer neighbour.
  const auto snap = snapshot();
  if (!snap || snap->track_lookup.empty()) {return false;}
  const auto & lut = snap->track_lookup;
  const auto after = std::lower_bound(
    lut.begin(), lut.end(), dist_m,
    [](const SessionIndex::TrackSample & s, double d) {return s.cum_dist_m < d;});
  const auto b = (after == lut.end()) ? after - 1 : after;
  const auto a = (b == lut.begin()) ? b : b - 1;
  const auto & best =
    (std::abs(a->cum_dist_m - dist_m) <= std::abs(b->cum_dist_m - dist_m)) ? *a : *b;
  map_x = best.x;
  map_y = best.y;
  return true;
}

std::vector<WindowPing> SidescanBagSession::readWindow(
  double dist_lo, double dist_hi, int max_pings, bool include_down,
  const std::shared_ptr<std::atomic<bool>> & cancel) const
{
  const auto snap = snapshot();
  if (!snap) {return {};}
  const double hi = (dist_hi < 0.0) ? snap->total_distance_m : dist_hi;

  // 1. Select paintable index entries in the distance window (channel-filtered).
  // The snapshot is held (snap) for the whole method, so pointers into it stay valid.
  std::vector<const SidescanPing *> sel;
  for (const auto & p : snap->pings) {
    if (p.cumulative_distance_m < dist_lo || p.cumulative_distance_m > hi) {continue;}
    if (!p.has_pose) {continue;}
    if (p.channel == SidescanChannel::Down && !include_down) {continue;}
    sel.push_back(&p);
  }
  if (sel.empty()) {return {};}

  // 2. Stationary cap: keep the most recent (the list is along-track ordered).
  const int from = stationary_keep_from(static_cast<int>(sel.size()), max_pings);
  sel.erase(sel.begin(), sel.begin() + from);

  // 3. Result slots + a (stamp_ns -> per-channel slot) lookup + the stamp span.
  std::vector<WindowPing> out(sel.size());
  std::unordered_map<int64_t, std::array<int, kNumSidescanChannels>> slot;
  int64_t t_lo = std::numeric_limits<int64_t>::max();
  int64_t t_hi = std::numeric_limits<int64_t>::min();
  for (std::size_t i = 0; i < sel.size(); ++i) {
    const SidescanPing * p = sel[i];
    out[i].channel = p->channel;
    out[i].geometry = p->geometry;
    out[i].cumulative_distance_m = p->cumulative_distance_m;
    auto res = slot.try_emplace(
      p->stamp_ns, std::array<int, kNumSidescanChannels>{-1, -1, -1});
    res.first->second[static_cast<int>(p->channel)] = static_cast<int>(i);
    t_lo = std::min(t_lo, p->stamp_ns);
    t_hi = std::max(t_hi, p->stamp_ns);
  }

  // 4. Re-read sample data over the stamp span (padded for recv-vs-header skew).
  std::map<std::string, SidescanChannel> topic_to_channel;
  for (int c = 0; c < kNumSidescanChannels; ++c) {
    topic_to_channel[kSidescanTopics[c]] = static_cast<SidescanChannel>(c);
  }
  constexpr int64_t kPadNs = 3000000000LL;  // 3 s
  rosbag2_cpp::Reader reader;
  reader.open(bag_uri_);
  try {
    reader.seek(static_cast<rcutils_time_point_value_t>(t_lo - kPadNs));
  } catch (const std::exception &) {
    // seek unsupported on this storage -> fall back to a sequential scan.
  }
  while (reader.has_next()) {
    if (cancel && cancel->load(std::memory_order_relaxed)) {
      return {};   // cancelled: an abandoned read publishes nothing (#44)
    }
    auto bag_msg = reader.read_next();
    if (bag_msg->recv_timestamp > t_hi + kPadNs) {break;}
    auto ch_it = topic_to_channel.find(bag_msg->topic_name);
    if (ch_it == topic_to_channel.end()) {continue;}
    if (ch_it->second == SidescanChannel::Down && !include_down) {continue;}
    try {
      auto img = deserialize<marine_acoustic_msgs::msg::RawSonarImage>(bag_msg);
      const int64_t sns = static_cast<int64_t>(img.header.stamp.sec) * 1000000000LL +
        img.header.stamp.nanosec;
      auto it = slot.find(sns);
      if (it == slot.end()) {continue;}
      const int idx = it->second[static_cast<int>(ch_it->second)];
      if (idx < 0) {continue;}
      out[idx].amplitudes = extract_beam0(img.image, img.samples_per_beam);
    } catch (const std::exception &) {
      // skip an unreadable message
    }
  }

  // 5. Drop entries whose samples weren't read (unreadable / outside the scan).
  out.erase(
    std::remove_if(
      out.begin(), out.end(),
      [](const WindowPing & w) {return w.amplitudes.empty();}),
    out.end());
  return out;
}

std::vector<MbesWindowPing> SidescanBagSession::readMbesWindow(
  double dist_lo, double dist_hi, int max_pings,
  const std::shared_ptr<std::atomic<bool>> & cancel) const
{
  const auto snap = snapshot();
  if (!snap) {return {};}
  const double hi = (dist_hi < 0.0) ? snap->total_distance_m : dist_hi;

  // 1. Select scrubbable detection pings in the distance window. The snapshot is
  // held (snap) for the whole method, so pointers into it stay valid.
  std::vector<const MbesPing *> sel;
  for (const auto & p : snap->mbes_pings) {
    if (!p.has_pose) {continue;}
    if (p.cumulative_distance_m < dist_lo || p.cumulative_distance_m > hi) {continue;}
    sel.push_back(&p);
  }
  if (sel.empty()) {return {};}

  // 2. Stationary cap (same policy as readWindow).
  const int from = stationary_keep_from(static_cast<int>(sel.size()), max_pings);
  sel.erase(sel.begin(), sel.begin() + from);

  // 3. Result slots keyed by stamp + the stamp span to re-read.
  std::vector<MbesWindowPing> out(sel.size());
  std::unordered_map<int64_t, int> slot;
  int64_t t_lo = std::numeric_limits<int64_t>::max();
  int64_t t_hi = std::numeric_limits<int64_t>::min();
  for (std::size_t i = 0; i < sel.size(); ++i) {
    out[i].cumulative_distance_m = sel[i]->cumulative_distance_m;
    out[i].sensor_x = sel[i]->tx;
    out[i].sensor_y = sel[i]->ty;
    out[i].heading =
      yaw_from_quaternion(sel[i]->qx, sel[i]->qy, sel[i]->qz, sel[i]->qw);
    out[i].has_pose = sel[i]->has_pose;
    slot.emplace(sel[i]->stamp_ns, static_cast<int>(i));
    t_lo = std::min(t_lo, sel[i]->stamp_ns);
    t_hi = std::max(t_hi, sel[i]->stamp_ns);
  }

  // 4. Re-read detections over the stamp span; project each to sensor-frame
  // soundings and lift to the world frame with the ping's captured transform.
  constexpr int64_t kPadNs = 3000000000LL;  // 3 s
  rosbag2_cpp::Reader reader;
  reader.open(bag_uri_);
  try {
    reader.seek(static_cast<rcutils_time_point_value_t>(t_lo - kPadNs));
  } catch (const std::exception &) {
    // seek unsupported -> sequential scan
  }
  while (reader.has_next()) {
    if (cancel && cancel->load(std::memory_order_relaxed)) {
      return {};   // cancelled: an abandoned read publishes nothing (#44)
    }
    auto bag_msg = reader.read_next();
    if (bag_msg->recv_timestamp > t_hi + kPadNs) {break;}
    if (bag_msg->topic_name != kMbesDetectionsTopic) {continue;}
    try {
      auto det = deserialize<marine_acoustic_msgs::msg::SonarDetections>(bag_msg);
      const int64_t sns = static_cast<int64_t>(det.header.stamp.sec) * 1000000000LL +
        det.header.stamp.nanosec;
      auto it = slot.find(sns);
      if (it == slot.end()) {continue;}
      const int idx = it->second;
      const MbesPing * p = sel[idx];
      out[idx].intensities.assign(det.intensities.begin(), det.intensities.end());
      const std::vector<MbesSounding> sensor = project_detections(det);
      out[idx].world_soundings.reserve(sensor.size());
      for (const auto & s : sensor) {
        double rx = 0.0;
        double ry = 0.0;
        double rz = 0.0;
        rotate_by_quat(p->qx, p->qy, p->qz, p->qw, s.x, s.y, s.z, rx, ry, rz);
        MbesSounding w;
        w.x = p->tx + rx;
        w.y = p->ty + ry;
        w.z = p->tz + rz;
        w.intensity = s.intensity;
        // The beam's own geometry rides along: angle and slant range are
        // measured in the SENSOR frame, so a rigid lift to world leaves them
        // unchanged — and both the ARA/TL correction (#27) and the angle-aware
        // per-sounding uncertainty (#49) read them off the world sounding.
        // Dropping them here left every bag-loaded sounding with a NaN angle.
        w.beam_angle = s.beam_angle;
        w.slant_range = s.slant_range;
        out[idx].world_soundings.push_back(w);
      }
    } catch (const std::exception &) {
      // skip an unreadable message
    }
  }

  // 5. Drop pings whose message wasn't found in the scan (no per-beam data read).
  out.erase(
    std::remove_if(
      out.begin(), out.end(),
      [](const MbesWindowPing & w) {return w.intensities.empty();}),
    out.end());
  return out;
}

std::vector<marine_acoustic_msgs::msg::RawSonarImage> SidescanBagSession::readDownImages(
  double dist_lo, double dist_hi, int max_pings,
  const std::shared_ptr<std::atomic<bool>> & cancel) const
{
  const auto snap = snapshot();
  if (!snap) {return {};}
  const double hi = (dist_hi < 0.0) ? snap->total_distance_m : dist_hi;

  // 1. Select scrubbable down-channel pings in the window (along-track order). The
  // snapshot is held (snap) for the whole method, so pointers into it stay valid.
  std::vector<const SidescanPing *> sel;
  for (const auto & p : snap->pings) {
    if (p.channel != SidescanChannel::Down || !p.has_pose) {continue;}
    if (p.cumulative_distance_m < dist_lo || p.cumulative_distance_m > hi) {continue;}
    sel.push_back(&p);
  }
  if (sel.empty()) {return {};}

  const int from = stationary_keep_from(static_cast<int>(sel.size()), max_pings);
  sel.erase(sel.begin(), sel.begin() + from);

  // 2. Result slots keyed by stamp + the stamp span to re-read.
  std::vector<marine_acoustic_msgs::msg::RawSonarImage> out(sel.size());
  std::vector<bool> filled(sel.size(), false);
  std::unordered_map<int64_t, int> slot;
  int64_t t_lo = std::numeric_limits<int64_t>::max();
  int64_t t_hi = std::numeric_limits<int64_t>::min();
  for (std::size_t i = 0; i < sel.size(); ++i) {
    slot.emplace(sel[i]->stamp_ns, static_cast<int>(i));
    t_lo = std::min(t_lo, sel[i]->stamp_ns);
    t_hi = std::max(t_hi, sel[i]->stamp_ns);
  }

  // 3. Re-read the down-channel images over the stamp span (raw, undecoded).
  constexpr int64_t kPadNs = 3000000000LL;  // 3 s
  rosbag2_cpp::Reader reader;
  reader.open(bag_uri_);
  try {
    reader.seek(static_cast<rcutils_time_point_value_t>(t_lo - kPadNs));
  } catch (const std::exception &) {
    // seek unsupported -> sequential scan
  }
  const std::string down_topic = kSidescanTopics[static_cast<int>(SidescanChannel::Down)];
  while (reader.has_next()) {
    if (cancel && cancel->load(std::memory_order_relaxed)) {
      return {};   // cancelled: an abandoned read publishes nothing (#44)
    }
    auto bag_msg = reader.read_next();
    if (bag_msg->recv_timestamp > t_hi + kPadNs) {break;}
    if (bag_msg->topic_name != down_topic) {continue;}
    try {
      auto img = deserialize<marine_acoustic_msgs::msg::RawSonarImage>(bag_msg);
      const int64_t sns = static_cast<int64_t>(img.header.stamp.sec) * 1000000000LL +
        img.header.stamp.nanosec;
      auto it = slot.find(sns);
      if (it == slot.end()) {continue;}
      out[it->second] = std::move(img);
      filled[it->second] = true;
    } catch (const std::exception &) {
      // skip an unreadable message
    }
  }

  // 4. Drop slots whose image wasn't read, preserving along-track order.
  std::vector<marine_acoustic_msgs::msg::RawSonarImage> result;
  result.reserve(out.size());
  for (std::size_t i = 0; i < out.size(); ++i) {
    if (filled[i]) {result.push_back(std::move(out[i]));}
  }
  return result;
}

}  // namespace marine_perception_tools
