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
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "builtin_interfaces/msg/time.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "ffmpeg_encoder_decoder/decoder.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/time.hpp"
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

// Decode the H.265 camera RGB (display-only) into loaded.rgb_frames. One libav
// decoder per camera (codec state is per-stream); packets are fed in bag order
// and flushed at the end. This is the heaviest, most isolable piece: any failure
// (decoder unavailable, corrupt stream) leaves a camera's rgb_frames empty and
// the UI shows a "(no RGB)" placeholder — it never blocks the rest of the load.
void decode_camera_rgb(
  const std::string & bag_uri, int64_t start_ns, int64_t end_ns, LoadedBag & loaded)
{
  std::map<std::string, int> ffmpeg_to_cam;
  for (int i = 0; i < kNumCameras; ++i) {
    ffmpeg_to_cam[std::string("/bizzy/sensors/cameras/") + kCameraNames[i] +
      "/image_raw/ffmpeg"] = i;
  }

  std::array<std::unique_ptr<ffmpeg_encoder_decoder::Decoder>, kNumCameras> decoders;
  std::mutex rgb_mutex;  // decoded callbacks may fire off a worker thread

  // H.265 is a GOP stream: starting from a non-zero --start-s feeds the decoder
  // mid-GOP, so libav drops frames until the next keyframe. The first ~second of
  // RGB after a windowed start can therefore be absent (the UI shows a "(no RGB)"
  // tile until an IDR passes). Display-only — the projected segmentation, and so
  // the costmap, are unaffected.

  rosbag2_cpp::Reader reader;
  reader.open(bag_uri);
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (bag_msg->recv_timestamp < start_ns) {continue;}
    if (bag_msg->recv_timestamp > end_ns) {break;}
    auto it = ffmpeg_to_cam.find(bag_msg->topic_name);
    if (it == ffmpeg_to_cam.end()) {continue;}
    const int cam = it->second;

    auto pkt = deserialize<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(bag_msg);
    if (!decoders[cam]) {
      auto decoder = std::make_unique<ffmpeg_encoder_decoder::Decoder>();
      auto callback = [&loaded, &rgb_mutex, cam](
        const sensor_msgs::msg::Image::ConstSharedPtr & img, bool, const std::string &) {
          cv::Mat bgr;
          try {
            bgr = cv_bridge::toCvCopy(img, "bgr8")->image;
          } catch (const cv_bridge::Exception &) {
            return;
          }
          RgbFrame rf;
          rf.cam = cam;
          rf.stamp_s = img->header.stamp.sec + img->header.stamp.nanosec * 1e-9;
          rf.bgr = bgr;
          std::lock_guard<std::mutex> lock(rgb_mutex);
          loaded.rgb_frames.push_back(std::move(rf));
        };
      // The libav software decoder for the codec shares the codec's name
      // ("hevc"/"h264"), so pass the packet encoding as both codec and decoder.
      if (!decoder->initialize(pkt.encoding, callback, pkt.encoding)) {
        continue;  // leave decoders[cam] null → this camera stays RGB-less
      }
      decoders[cam] = std::move(decoder);
    }

    const rclcpp::Time stamp(pkt.header.stamp.sec, pkt.header.stamp.nanosec, RCL_ROS_TIME);
    decoders[cam]->decodePacket(
      pkt.encoding, pkt.data.data(), pkt.data.size(), pkt.pts, pkt.header.frame_id, stamp);
  }

  for (int i = 0; i < kNumCameras; ++i) {
    if (decoders[i]) {decoders[i]->flush();}
  }
  std::stable_sort(
    loaded.rgb_frames.begin(), loaded.rgb_frames.end(),
    [](const RgbFrame & a, const RgbFrame & b) {return a.stamp_s < b.stamp_s;});
}

}  // namespace

BagSession::BagSession(const std::string & bag_uri, const BagLoadOptions & opts)
: bag_uri_(bag_uri), opts_(opts), camera_models_(kNumCameras)
{
  std::array<CamTopics, kNumCameras> topics;
  for (int i = 0; i < kNumCameras; ++i) {
    const std::string base = std::string("/bizzy/sensors/cameras/") + kCameraNames[i];
    topics[i].seg = base + "/segmentation";
    topics[i].compressed = topics[i].seg + "/compressed";
    topics[i].info = topics[i].seg + "/camera_info";
  }

  // ---- Single full scan: bag bounds + topic discovery + TF cache + models. ----
  rosbag2_cpp::Reader reader;
  reader.open(bag_uri);
  const auto & meta = reader.get_metadata();
  bag_start_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    meta.starting_time.time_since_epoch()).count();
  duration_s_ = std::chrono::duration_cast<std::chrono::duration<double>>(
    meta.duration).count();

  // Size the TF cache to the bag's own duration (plus a pad) so a window read
  // anywhere in the recording resolves: BufferCore evicts transforms older than
  // its cache window relative to the newest inserted stamp, so a fixed cache
  // would drop the start of a long bag once the whole file is scanned in here.
  tf_buffer_ = std::make_unique<tf2::BufferCore>(tf2::durationFromSec(duration_s_ + 60.0));

  // Select by message count, not mere topic registration: a bag that registers
  // a camera's raw `segmentation` topic but recorded zero messages on it must
  // not shadow a populated `.../compressed` topic for the same camera.
  std::map<std::string, std::size_t> counts;
  for (const auto & t : meta.topics_with_message_count) {
    counts[t.topic_metadata.name] = t.message_count;
  }
  auto has_msgs = [&counts](const std::string & topic) {
      auto it = counts.find(topic);
      return it != counts.end() && it->second > 0;
    };
  for (int i = 0; i < kNumCameras; ++i) {
    if (has_msgs(topics[i].seg)) {
      seg_source_[i] = topics[i].seg;
      seg_compressed_[i] = false;
    } else if (has_msgs(topics[i].compressed)) {
      seg_source_[i] = topics[i].compressed;
      seg_compressed_[i] = true;
      used_compressed_segmentation_ = true;
    }
  }

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    const std::string & topic = bag_msg->topic_name;
    if (topic == "/tf" || topic == "/tf_static") {
      auto tfm = deserialize<tf2_msgs::msg::TFMessage>(bag_msg);
      const bool is_static = (topic == "/tf_static");
      for (const auto & tr : tfm.transforms) {
        tf_buffer_->setTransform(tr, "bag", is_static);
      }
      continue;
    }
    for (int i = 0; i < kNumCameras; ++i) {
      if (topic == topics[i].info) {
        auto info = deserialize<sensor_msgs::msg::CameraInfo>(bag_msg);
        camera_models_[i].fromCameraInfo(info);
        optical_frame_[i] = info.header.frame_id;
        have_model_[i] = true;
        break;
      }
    }
  }

  // A session with no usable camera (model + seg source) can never produce a
  // frame, so fail at open rather than on the first windowed read.
  bool any_usable = false;
  for (int i = 0; i < kNumCameras; ++i) {
    if (have_model_[i] && !seg_source_[i].empty()) {any_usable = true; break;}
  }
  if (!any_usable) {
    throw std::runtime_error(
      "bag has no usable camera (segmentation + camera_info) among "
      "oak_{port,forward,starboard,aft}");
  }
}

BagSession::~BagSession() = default;

LoadedBag BagSession::loadWindow(double start_s, double end_s) const
{
  // Dispatch table: chosen seg-source topic → camera index. Only cameras that
  // have both a model and a segmentation source contribute.
  std::map<std::string, int> seg_topic_to_cam;
  for (int i = 0; i < kNumCameras; ++i) {
    if (have_model_[i] && !seg_source_[i].empty()) {
      seg_topic_to_cam[seg_source_[i]] = i;
    }
  }

  // Intersect the requested window with the session's own [start_s, end_s] clamp
  // (from the ctor opts). A negative end means "to end of bag" on either side.
  double win_start = std::max(start_s, opts_.start_s);
  double win_end = end_s;
  if (opts_.end_s >= 0.0) {
    win_end = (win_end < 0.0) ? opts_.end_s : std::min(win_end, opts_.end_s);
  }

  LoadedBag loaded;
  loaded.camera_models = camera_models_;
  loaded.used_compressed_segmentation = used_compressed_segmentation_;

  const std::string costmap_topic = "/bizzy/local_costmap/costmap";
  const int64_t start_ns = bag_start_ns_ + static_cast<int64_t>(win_start * 1e9);
  const int64_t end_ns = (win_end < 0.0) ?
    std::numeric_limits<int64_t>::max() :
    bag_start_ns_ + static_cast<int64_t>(win_end * 1e9);

  // Window gating uses bag receive time (recv_timestamp); the frame stamp and TF
  // lookup below use the image header stamp. These differ by recording latency —
  // faithful to the reference driver (bag_to_costmap_video.cpp); the window
  // bounds are documented as "from bag start" (receive time).
  rosbag2_cpp::Reader reader;
  reader.open(bag_uri_);
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
    // (`optical_frame_[cam]`), not the image's own frame_id, so a republished /
    // differing image frame_id won't silently drop every frame.
    cv::Mat mask;
    builtin_interfaces::msg::Time stamp;
    try {
      if (seg_compressed_[cam]) {
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
        tf_buffer_->lookupTransform(opts_.world_frame, optical_frame_[cam], tf_time);
      const auto boat_tf =
        tf_buffer_->lookupTransform(opts_.world_frame, opts_.boat_frame, tf_time);

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
      // Heading (world-frame yaw) from the boat quaternion, for the costmap
      // boat marker. Standard ENU yaw: atan2(2(wz+xy), 1-2(yy+zz)).
      const auto & bq = boat_tf.transform.rotation;
      frame.boat_yaw = std::atan2(
        2.0 * (bq.w * bq.z + bq.x * bq.y),
        1.0 - 2.0 * (bq.y * bq.y + bq.z * bq.z));
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

  loaded.frames_skipped_no_tf = tf_skipped;

  // Display-only RGB: decode the H.265 camera streams over the same window.
  decode_camera_rgb(bag_uri_, start_ns, end_ns, loaded);

  // Resolve each RGB frame's camera→world rotation from TF at THAT image's own
  // stamp (the horizon overlay needs it). Per-image-stamp resolution is
  // deliberate: a TF/image desync then shows as a drawn horizon that doesn't
  // match the visual one. Missing model or TF leaves identity + has_pose=false.
  for (auto & rf : loaded.rgb_frames) {
    if (!have_model_[rf.cam] || optical_frame_[rf.cam].empty()) {continue;}
    const auto tf_time = tf2::TimePoint(
      std::chrono::nanoseconds(static_cast<int64_t>(rf.stamp_s * 1e9)));
    try {
      const auto tf =
        tf_buffer_->lookupTransform(opts_.world_frame, optical_frame_[rf.cam], tf_time);
      const auto & q = tf.transform.rotation;
      rf.rotation_cam_to_target =
        sea_surface_segmentation::rotation_matrix_from_quaternion(q.x, q.y, q.z, q.w);
      rf.has_pose = true;
    } catch (const tf2::TransformException &) {
      // leave identity + has_pose=false; the overlay skips this frame's camera
    }
  }

  if (loaded.frames.empty()) {
    // win_end < 0 means "to end of bag"; print "end" rather than a bare -1 that
    // reads as an inverted range.
    const std::string end_str =
      (win_end < 0.0) ? "end" : (std::to_string(win_end) + "s");
    throw std::runtime_error(
      "no usable segmentation frames in [" + std::to_string(win_start) + "s, " +
      end_str + "] (" + std::to_string(tf_skipped) + " skipped for missing TF)");
  }

  return loaded;
}

LoadedBag load_bag(const std::string & bag_uri, const BagLoadOptions & opts)
{
  // One-shot: open the session and read its full [start_s, end_s] clamp in one
  // call. Passing the session's own clamp as the window is a no-op intersection,
  // so this reproduces the original whole-or-clamped load exactly.
  return BagSession(bag_uri, opts).loadWindow(opts.start_s, opts.end_s);
}

}  // namespace marine_perception_tools
