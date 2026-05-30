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

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "builtin_interfaces/msg/time.hpp"
#include "cv_bridge/cv_bridge.hpp"
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

}  // namespace

LoadedBag load_forward_camera(const std::string & bag_uri, const BagLoadOptions & opts)
{
  const std::string seg_topic =
    "/bizzy/sensors/cameras/" + opts.camera + "/segmentation";
  const std::string compressed_topic = seg_topic + "/compressed";
  const std::string info_topic = seg_topic + "/camera_info";

  // ---- Pass 1: TF cache (whole bag) + the camera model. ----
  tf2::BufferCore tf_buffer(tf2::durationFromSec(7200.0));
  LoadedBag loaded;
  std::string optical_frame;  // the camera's optical frame id
  bool have_model = false;

  // Seg-source selection: prefer the raw `Image` topic; fall back to the
  // `.../compressed` `CompressedImage` topic when the raw one was not recorded
  // (size-trimmed bags). Reading both would double-count every frame.
  std::string seg_source_topic;
  bool seg_compressed = false;
  {
    rosbag2_cpp::Reader reader;
    reader.open(bag_uri);
    bool have_raw = false;
    bool have_cmp = false;
    for (const auto & t : reader.get_all_topics_and_types()) {
      if (t.name == seg_topic) {
        have_raw = true;
      } else if (t.name == compressed_topic) {
        have_cmp = true;
      }
    }
    if (have_raw) {
      seg_source_topic = seg_topic;
      seg_compressed = false;
    } else if (have_cmp) {
      seg_source_topic = compressed_topic;
      seg_compressed = true;
    } else {
      throw std::runtime_error(
        "bag has neither '" + seg_topic + "' nor '" + compressed_topic +
        "' — no segmentation for camera " + opts.camera);
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
      } else if (topic == info_topic) {
        auto info = deserialize<sensor_msgs::msg::CameraInfo>(bag_msg);
        loaded.camera_model.fromCameraInfo(info);
        optical_frame = info.header.frame_id;
        have_model = true;
      }
    }
  }

  if (!have_model) {
    throw std::runtime_error(
      "bag has no '" + info_topic + "' — cannot resolve the " + opts.camera +
      " camera model");
  }
  loaded.used_compressed_segmentation = seg_compressed;

  // ---- Pass 2: replay segmentation, resolve pose, build PreparedFrames. ----
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
    if (bag_msg->topic_name != seg_source_topic) {continue;}

    // Decode the mask from whichever source was selected (raw Image or
    // CompressedImage). The camera model is already pinned to this camera by
    // topic name; the TF lookup uses `optical_frame` (the camera_info frame_id),
    // not the image's own frame_id, so a republished/differing image frame_id
    // won't silently drop every frame.
    cv::Mat mask;
    builtin_interfaces::msg::Time stamp;
    try {
      if (seg_compressed) {
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
      const auto cam_tf = tf_buffer.lookupTransform(opts.world_frame, optical_frame, tf_time);
      const auto boat_tf = tf_buffer.lookupTransform(opts.world_frame, opts.boat_frame, tf_time);

      const auto & ct = cam_tf.transform.translation;
      const auto & cq = cam_tf.transform.rotation;

      PreparedFrame frame;
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

  if (loaded.frames.empty()) {
    throw std::runtime_error(
      "no usable " + opts.camera + " frames in [" + std::to_string(opts.start_s) + ", " +
      std::to_string(opts.end_s) + "]s (" + std::to_string(tf_skipped) +
      " skipped for missing TF)");
  }

  return loaded;
}

}  // namespace marine_perception_tools
