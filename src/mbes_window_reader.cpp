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

#include "mbes_window_reader.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "marine_acoustic_msgs/msg/sonar_detections.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/buffer_core.h"
#include "tf2/time.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "sidescan_bag_session.hpp"

namespace marine_perception_tools
{

namespace
{

// Same deserialize shim as sidescan_bag_session.cpp's (5 lines; the anonymous
// namespaces keep them from colliding).
template<typename T>
T deserialize(const rosbag2_storage::SerializedBagMessageSharedPtr & msg)
{
  T out;
  rclcpp::SerializedMessage serialized(*msg->serialized_data);
  rclcpp::Serialization<T>().deserialize_message(&serialized, &out);
  return out;
}

// Padding around the window: the recv-timestamp break test and the seek target
// tolerate recv-vs-header stamp skew (same value as readMbesWindow).
constexpr std::int64_t kPadNs = 3000000000LL;  // 3 s

}  // namespace

MbesWindowResult read_mbes_window(
  const std::string & bag_uri, std::int64_t t_start_ns, std::int64_t t_end_ns,
  const MbesWindowOptions & options,
  const std::shared_ptr<std::atomic<bool>> & cancel)
{
  MbesWindowResult result;
  result.world_frame = options.world_frame;
  const auto stop = [&cancel]() {
      return cancel && cancel->load(std::memory_order_relaxed);
    };
  // A cancelled read hands back nothing: the partial window it had gathered
  // is not a smaller window, it is an unfinished one (#44).
  const auto abandon = [&result]() {
      result.world_soundings.clear();
      result.used_pings = 0;
      result.cancelled = true;
      return result;
    };
  if (t_end_ns < t_start_ns) {
    std::swap(t_start_ns, t_end_ns);   // reversed window: the readMbesWindow precedent
  }
  const std::string det_topic = options.detections_topic.empty() ?
    std::string(kMbesDetectionsTopic) : options.detections_topic;

  // TF prepass, two topic-filtered reads. /tf_static is scanned from the bag
  // start (latched mounts live there; it is a handful of messages). /tf is
  // SEEKED to shortly before the window: every dynamic pair the lift needs —
  // including the earth<-map geo anchor — is republished continuously in these
  // recordings, so a bounded pre-window history brackets all window stamps.
  // (A once-published dynamic anchor would be missed by the seek; that
  // degrades to has_geo=false / skipped pings, both visible to the caller —
  // never silently misplaced soundings.) Scanning /tf from the start instead
  // costs time proportional to the window's position in the bag — measured
  // 50 s for a pass 75 min in, vs ~1 s seeked.
  const auto cache_span = std::chrono::nanoseconds(
    (t_end_ns - t_start_ns) + 2 * kPadNs + 60000000000LL);
  tf2::BufferCore tf_buffer(cache_span);
  const auto feed_tf =
    [&tf_buffer, t_end_ns, &stop](rosbag2_cpp::Reader & reader, bool is_static) {
      while (reader.has_next()) {
        if (stop()) {
          return;   // cancelled: stop consuming the bag between messages
        }
        auto bag_msg = reader.read_next();
        if (bag_msg->recv_timestamp > t_end_ns + kPadNs) {
          break;
        }
        try {
          auto tfm = deserialize<tf2_msgs::msg::TFMessage>(bag_msg);
          for (const auto & tr : tfm.transforms) {
            tf_buffer.setTransform(tr, "bag", is_static);
          }
        } catch (const std::exception &) {
          // skip an unreadable TF message
        }
      }
    };
  {
    rosbag2_cpp::Reader reader;
    reader.open(bag_uri);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {"/tf_static"};
    reader.set_filter(filter);
    feed_tf(reader, true);
  }
  if (stop()) {return abandon();}
  {
    // 15 s of pre-window /tf history: enough to bracket the earliest window
    // stamp for every continuously-published pair, with margin over kPadNs.
    constexpr std::int64_t kTfHistoryNs = 15000000000LL;
    rosbag2_cpp::Reader reader;
    reader.open(bag_uri);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {"/tf"};
    reader.set_filter(filter);
    try {
      reader.seek(static_cast<rcutils_time_point_value_t>(t_start_ns - kTfHistoryNs));
    } catch (const std::exception &) {
      // seek unsupported -> sequential scan
    }
    feed_tf(reader, false);
  }
  if (stop()) {return abandon();}

  // Geo anchor for cross-bag combination: earth<-world at the window midpoint.
  {
    const auto t_mid = tf2::TimePoint(
      std::chrono::nanoseconds(t_start_ns + (t_end_ns - t_start_ns) / 2));
    geometry_msgs::msg::TransformStamped ew;
    if (lookup_at_or_latest(
        tf_buffer, options.geo_frame, options.world_frame, t_mid, ew))
    {
      result.has_geo = true;
      result.earth_from_world = ew;
    }
  }

  // Detections pass: seek to the window (sequential-scan fallback when the
  // storage doesn't support seek — the readMbesWindow precedent), filtered to
  // the detections topic. Window membership tests the HEADER stamp (what the
  // index recorded); recv_timestamp only bounds the scan.
  rosbag2_cpp::Reader reader;
  reader.open(bag_uri);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {det_topic};
  reader.set_filter(filter);
  try {
    reader.seek(static_cast<rcutils_time_point_value_t>(t_start_ns - kPadNs));
  } catch (const std::exception &) {
    // seek unsupported -> sequential scan
  }
  while (reader.has_next()) {
    if (stop()) {
      return abandon();   // cancelled: per message, not per bag
    }
    auto bag_msg = reader.read_next();
    if (bag_msg->recv_timestamp > t_end_ns + kPadNs) {
      break;
    }
    try {
      auto det = deserialize<marine_acoustic_msgs::msg::SonarDetections>(bag_msg);
      const std::int64_t stamp_ns =
        static_cast<std::int64_t>(det.header.stamp.sec) * 1000000000LL +
        det.header.stamp.nanosec;
      if (stamp_ns < t_start_ns || stamp_ns > t_end_ns) {
        continue;
      }
      // Same source-frame fallback as the session's load pass.
      const std::string src =
        det.header.frame_id.empty() ? std::string("bizzy/m3") : det.header.frame_id;
      geometry_msgs::msg::TransformStamped tf;
      if (!lookup_at_or_latest(
          tf_buffer, options.world_frame, src,
          tf2::TimePoint(std::chrono::nanoseconds(stamp_ns)), tf))
      {
        ++result.skipped_pings;
        continue;
      }
      // No per-ping reserve: reserve(size+n) allocates EXACTLY, so calling it
      // each ping defeats amortized doubling and turns the append quadratic
      // (measured: 50 s for a ~1M-sounding window; ~1 s without).
      const std::vector<MbesSounding> sensor = project_detections(det);
      for (const auto & s : sensor) {
        result.world_soundings.push_back(
          lift_sounding_to_world(
            s,
            tf.transform.translation.x, tf.transform.translation.y,
            tf.transform.translation.z,
            tf.transform.rotation.x, tf.transform.rotation.y,
            tf.transform.rotation.z, tf.transform.rotation.w));
      }
      ++result.used_pings;
    } catch (const std::exception &) {
      // skip an unreadable message
    }
  }
  return result;
}

FrameReprojection make_reprojection(
  const geometry_msgs::msg::TransformStamped & earth_from_ref,
  const geometry_msgs::msg::TransformStamped & earth_from_src)
{
  const auto to_tf2 = [](const geometry_msgs::msg::TransformStamped & t) {
      return tf2::Transform(
        tf2::Quaternion(
          t.transform.rotation.x, t.transform.rotation.y,
          t.transform.rotation.z, t.transform.rotation.w),
        tf2::Vector3(
          t.transform.translation.x, t.transform.translation.y,
          t.transform.translation.z));
    };
  const tf2::Transform ref_from_src =
    to_tf2(earth_from_ref).inverse() * to_tf2(earth_from_src);
  // Equal anchors (bags sharing one geo alignment) compose to identity up to
  // double rounding — flag it so apply_reprojection() is a no-op per point.
  // Rounding on ECEF-sized translations is ~1e-9 m (6.4e6 m x 1e-16); 1e-6 m
  // covers it with margin while staying 1000x below any real mm-scale
  // inter-bag offset, so genuine survey signal can never be collapsed.
  constexpr double kTransEps = 1e-6;   // metres
  constexpr double kRotEps = 1e-12;    // quaternion component distance
  const auto & o = ref_from_src.getOrigin();
  const auto q0 = ref_from_src.getRotation();
  if (std::abs(o.x()) < kTransEps && std::abs(o.y()) < kTransEps &&
    std::abs(o.z()) < kTransEps &&
    std::abs(q0.x()) < kRotEps && std::abs(q0.y()) < kRotEps &&
    std::abs(q0.z()) < kRotEps && std::abs(std::abs(q0.w()) - 1.0) < kRotEps)
  {
    return FrameReprojection{};   // identity
  }
  FrameReprojection out;
  out.identity = false;
  out.tx = ref_from_src.getOrigin().x();
  out.ty = ref_from_src.getOrigin().y();
  out.tz = ref_from_src.getOrigin().z();
  const auto q = ref_from_src.getRotation();
  out.qx = q.x();
  out.qy = q.y();
  out.qz = q.z();
  out.qw = q.w();
  return out;
}

}  // namespace marine_perception_tools
