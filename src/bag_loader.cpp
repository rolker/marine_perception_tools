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

#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
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
  const std::string info_topic = seg_topic + "/camera_info";

  // ---- Pass 1: TF cache (whole bag) + the camera model. ----
  tf2::BufferCore tf_buffer(tf2::durationFromSec(7200.0));
  LoadedBag loaded;
  std::string optical_frame;  // the camera's optical frame id
  bool have_model = false;
  {
    rosbag2_cpp::Reader reader;
    reader.open(bag_uri);
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

  // ---- Pass 2: replay segmentation, resolve pose, build PreparedFrames. ----
  rosbag2_cpp::Reader reader;
  reader.open(bag_uri);
  const int64_t bag_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    reader.get_metadata().starting_time.time_since_epoch()).count();
  const int64_t start_ns = bag_start_ns + static_cast<int64_t>(opts.start_s * 1e9);
  const int64_t end_ns = (opts.end_s < 0.0) ?
    std::numeric_limits<int64_t>::max() :
    bag_start_ns + static_cast<int64_t>(opts.end_s * 1e9);

  std::size_t tf_skipped = 0;
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (bag_msg->recv_timestamp < start_ns) {continue;}
    if (bag_msg->recv_timestamp > end_ns) {break;}
    if (bag_msg->topic_name != seg_topic) {continue;}

    auto img_msg = deserialize<sensor_msgs::msg::Image>(bag_msg);
    if (img_msg.header.frame_id != optical_frame) {continue;}

    cv::Mat mask;
    try {
      mask = cv_bridge::toCvCopy(img_msg, "rgb8")->image;
    } catch (const cv_bridge::Exception &) {
      continue;
    }

    const auto tf_time = to_tf_time(img_msg.header.stamp);
    try {
      const auto cam_tf = tf_buffer.lookupTransform(opts.world_frame, optical_frame, tf_time);
      const auto boat_tf = tf_buffer.lookupTransform(opts.world_frame, opts.boat_frame, tf_time);

      const auto & ct = cam_tf.transform.translation;
      const auto & cq = cam_tf.transform.rotation;

      PreparedFrame frame;
      frame.stamp_s = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9;
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
