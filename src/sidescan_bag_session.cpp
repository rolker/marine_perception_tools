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
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "marine_acoustic_msgs/msg/raw_sonar_image.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "tf2/buffer_core.h"
#include "tf2/time.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "distance_buffer_policy.hpp"

namespace marine_perception_tools
{
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
: bag_uri_(bag_uri)
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

  // ---- Open + bounds + channel presence. ----
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
  tf_buffer_ = std::make_unique<tf2::BufferCore>(tf2::durationFromSec(opts.tf_cache_s));

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
  const double sample_dt = std::max(0.01, opts.pose_sample_dt_s);
  constexpr double kMargin = 0.1;

  // ---- Single pass: stream the bag in order. ----
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
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
              tf_buffer_->lookupTransform(opts.world_frame, opts.base_frame, tp);
            const auto & tr = tf.transform.translation;
            const auto & q = tf.transform.rotation;
            BasePoseSample s;
            s.t = sample_t;
            s.x = tr.x;
            s.y = tr.y;
            s.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w);
            base_table.push_back(s);
            last_sample_s = sample_t;
            if (!has_geo_reference_) {
              has_geo_reference_ =
                tf_buffer_->canTransform(opts.geo_frame, opts.world_frame, tp);
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
          opts.altitude_threshold_frac);
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

  for (auto & ping : pings_) {
    BasePoseSample bp;
    if (interp_base_pose(base_table, ping.stamp_s, opts.pose_max_gap_s, bp)) {
      ping.geometry.sensor_x = bp.x;
      ping.geometry.sensor_y = bp.y;
      ping.geometry.yaw = bp.yaw;
      ping.cumulative_distance_m = bp.cumdist;
      ping.has_pose = true;
      ++poses_resolved_;
    } else {
      ++poses_skipped_;
    }
  }

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
      ping.geometry.altitude = (best_dt <= opts.altitude_max_dt_s) ? best : 0.0;
    }
  }

  // Geo-export readiness (earth -> world) was probed during the streaming pass,
  // while the relevant transforms were still in the bounded cache.
  prof("altitude assign + done");
}

SidescanBagSession::~SidescanBagSession() = default;

std::vector<const SidescanPing *> SidescanBagSession::window(
  double dist_lo, double dist_hi) const
{
  const double hi = (dist_hi < 0.0) ? total_distance_m_ : dist_hi;
  std::vector<const SidescanPing *> out;
  for (const auto & ping : pings_) {
    if (ping.cumulative_distance_m >= dist_lo && ping.cumulative_distance_m <= hi) {
      out.push_back(&ping);
    }
  }
  return out;
}

std::size_t SidescanBagSession::channelCount(SidescanChannel ch) const
{
  std::size_t n = 0;
  for (const auto & ping : pings_) {
    if (ping.channel == ch) {
      ++n;
    }
  }
  return n;
}

std::vector<WindowPing> SidescanBagSession::readWindow(
  double dist_lo, double dist_hi, int max_pings, bool include_down) const
{
  const double hi = (dist_hi < 0.0) ? total_distance_m_ : dist_hi;

  // 1. Select paintable index entries in the distance window (channel-filtered).
  std::vector<const SidescanPing *> sel;
  for (const auto & p : pings_) {
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

}  // namespace marine_perception_tools
