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

#include "bag_loader.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "builtin_interfaces/msg/time.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/buffer_core.h"
#include "tf2/time.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "sea_surface_segmentation/segments_projection.hpp"

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

tf2::TimePoint to_tf_time(const builtin_interfaces::msg::Time & t)
{
  return tf2::TimePoint(
    std::chrono::seconds(t.sec) + std::chrono::nanoseconds(t.nanosec));
}

// Per-camera topic names derived from the camera namespace name.
struct CamTopics
{
  std::string seg;
  std::string compressed;
  std::string info;
};

}  // namespace

LoadedBag load_bag(const std::string & bag_uri, const BagLoadOptions & opts)
{
  std::array<CamTopics, kNumCameras> topics;
  for (int i = 0; i < kNumCameras; ++i) {
    const std::string base = std::string("/bizzy/sensors/cameras/") + kCameraNames[i];
    topics[i].seg = base + "/segmentation";
    topics[i].compressed = topics[i].seg + "/compressed";
    topics[i].info = topics[i].seg + "/camera_info";
  }

  tf2::BufferCore tf_buffer(tf2::durationFromSec(7200.0));
  LoadedBag loaded;
  loaded.camera_models.resize(kNumCameras);
  std::array<std::string, kNumCameras> optical_frame;
  std::array<bool, kNumCameras> have_model{};

  // Seg-source selection per camera: prefer raw `Image`, else `CompressedImage`
  // (size-trimmed bags). An empty entry means the camera isn't in this bag.
  std::array<std::string, kNumCameras> seg_source;
  std::array<bool, kNumCameras> seg_compressed{};

  // ---- Pass 1: topic discovery + TF cache (whole bag) + per-camera models. ----
  {
    rosbag2_cpp::Reader reader;
    reader.open(bag_uri);

    std::set<std::string> present;
    for (const auto & t : reader.get_all_topics_and_types()) {
      present.insert(t.name);
    }
    for (int i = 0; i < kNumCameras; ++i) {
      if (present.count(topics[i].seg) != 0) {
        seg_source[i] = topics[i].seg;
        seg_compressed[i] = false;
      } else if (present.count(topics[i].compressed) != 0) {
        seg_source[i] = topics[i].compressed;
        seg_compressed[i] = true;
        loaded.used_compressed_segmentation = true;
      }
    }

    while (reader.has_next()) {
      auto bag_msg = reader.read_next();
      const std::string & topic = bag_msg->topic_name;
      if (topic == "/tf" || topic == "/tf_static") {
        auto tfm = deserialize<tf2_msgs::msg::TFMessage>(bag_msg);
        const bool is_static = (topic == "/tf_static");
        for (const auto & tr : tfm.transforms) {
          tf_buffer.setTransform(tr, "bag", is_static);
        }
        continue;
      }
      for (int i = 0; i < kNumCameras; ++i) {
        if (topic == topics[i].info) {
          auto info = deserialize<sensor_msgs::msg::CameraInfo>(bag_msg);
          loaded.camera_models[i].fromCameraInfo(info);
          optical_frame[i] = info.header.frame_id;
          have_model[i] = true;
          break;
        }
      }
    }
  }

  // Dispatch table for pass 2: chosen seg-source topic → camera index. Only
  // cameras that have both a model and a segmentation source contribute.
  std::map<std::string, int> seg_topic_to_cam;
  for (int i = 0; i < kNumCameras; ++i) {
    if (have_model[i] && !seg_source[i].empty()) {
      seg_topic_to_cam[seg_source[i]] = i;
    }
  }
  if (seg_topic_to_cam.empty()) {
    throw std::runtime_error(
      "bag has no usable camera (segmentation + camera_info) among "
      "oak_{port,forward,starboard,aft}");
  }

  // ---- Pass 2: replay every camera's segmentation into one merged timeline. ----
  const std::string costmap_topic = "/bizzy/local_costmap/costmap";
  rosbag2_cpp::Reader reader;
  reader.open(bag_uri);
  const int64_t bag_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    reader.get_metadata().starting_time.time_since_epoch()).count();
  const int64_t start_ns = bag_start_ns + static_cast<int64_t>(opts.start_s * 1e9);
  const int64_t end_ns = (opts.end_s < 0.0) ?
    std::numeric_limits<int64_t>::max() :
    bag_start_ns + static_cast<int64_t>(opts.end_s * 1e9);

  // Window gating uses bag receive time (recv_timestamp); the frame stamp and TF
  // lookup below use the image header stamp. These differ by recording latency —
  // faithful to the reference driver (bag_to_costmap_video.cpp); --start-s/--end-s
  // are documented as "from bag start" (receive time).
  std::size_t tf_skipped = 0;
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (bag_msg->recv_timestamp < start_ns) {continue;}
    if (bag_msg->recv_timestamp > end_ns) {break;}

    if (bag_msg->topic_name == costmap_topic) {
      auto grid = deserialize<nav_msgs::msg::OccupancyGrid>(bag_msg);
      RecordedCostmap rc;
      rc.stamp_s = grid.header.stamp.sec + grid.header.stamp.nanosec * 1e-9;
      rc.origin_x = grid.info.origin.position.x;
      rc.origin_y = grid.info.origin.position.y;
      rc.resolution = grid.info.resolution;
      rc.width = static_cast<int>(grid.info.width);
      rc.height = static_cast<int>(grid.info.height);
      rc.data.assign(grid.data.begin(), grid.data.end());
      loaded.costmaps.push_back(std::move(rc));
      continue;
    }

    auto cam_it = seg_topic_to_cam.find(bag_msg->topic_name);
    if (cam_it == seg_topic_to_cam.end()) {continue;}
    const int cam = cam_it->second;

    // Decode the mask from whichever source was selected for this camera (raw
    // Image or CompressedImage). The TF lookup uses the camera_info frame_id
    // (`optical_frame[cam]`), not the image's own frame_id, so a republished /
    // differing image frame_id won't silently drop every frame.
    cv::Mat mask;
    builtin_interfaces::msg::Time stamp;
    try {
      if (seg_compressed[cam]) {
        auto cmp = deserialize<sensor_msgs::msg::CompressedImage>(bag_msg);
        mask = cv_bridge::toCvCopy(cmp, "rgb8")->image;
        stamp = cmp.header.stamp;
      } else {
        auto img_msg = deserialize<sensor_msgs::msg::Image>(bag_msg);
        mask = cv_bridge::toCvCopy(img_msg, "rgb8")->image;
        stamp = img_msg.header.stamp;
      }
    } catch (const cv_bridge::Exception &) {
      continue;
    }

    const auto tf_time = to_tf_time(stamp);
    try {
      const auto cam_tf =
        tf_buffer.lookupTransform(opts.world_frame, optical_frame[cam], tf_time);
      const auto boat_tf =
        tf_buffer.lookupTransform(opts.world_frame, opts.boat_frame, tf_time);

      const auto & ct = cam_tf.transform.translation;
      const auto & cq = cam_tf.transform.rotation;

      PreparedFrame frame;
      frame.cam = cam;
      frame.stamp_s = stamp.sec + stamp.nanosec * 1e-9;
      frame.mask_rgb8 = mask;
      frame.camera_origin = cv::Vec3d(ct.x, ct.y, ct.z);
      frame.rotation_cam_to_target =
        sea_surface_segmentation::rotation_matrix_from_quaternion(cq.x, cq.y, cq.z, cq.w);
      frame.boat_x = boat_tf.transform.translation.x;
      frame.boat_y = boat_tf.transform.translation.y;
      loaded.frames.push_back(std::move(frame));
    } catch (const tf2::TransformException &) {
      ++tf_skipped;  // pose unavailable at this stamp — skip, don't fabricate geometry
      continue;
    }
  }

  // Interleave the cameras chronologically: accumulate_frame decays on a
  // monotonic stamp, so the merged stream must be ascending in time.
  std::stable_sort(
    loaded.frames.begin(), loaded.frames.end(),
    [](const PreparedFrame & a, const PreparedFrame & b) {return a.stamp_s < b.stamp_s;});
  std::stable_sort(
    loaded.costmaps.begin(), loaded.costmaps.end(),
    [](const RecordedCostmap & a, const RecordedCostmap & b) {return a.stamp_s < b.stamp_s;});

  if (loaded.frames.empty()) {
    throw std::runtime_error(
      "no usable segmentation frames in [" + std::to_string(opts.start_s) + ", " +
      std::to_string(opts.end_s) + "]s (" + std::to_string(tf_skipped) +
      " skipped for missing TF)");
  }

  return loaded;
}

}  // namespace marine_perception_tools
