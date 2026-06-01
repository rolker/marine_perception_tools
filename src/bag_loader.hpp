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

#ifndef BAG_LOADER_HPP_
#define BAG_LOADER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "image_geometry/pinhole_camera_model.hpp"

// Forward-declared so the persistent TF cache stays out of the UI / engine
// translation units that include this header (they never touch tf2 directly).
namespace tf2 {class BufferCore;}

namespace marine_perception_tools
{

// The four OAK cameras, in left→right display order: port (left), forward,
// starboard (right), aft. `kCameraNames` are the topic-namespace names
// (/bizzy/sensors/cameras/<name>/…); `kCameraLabels` are the short tile labels.
inline constexpr int kNumCameras = 4;
inline constexpr std::array<const char *, kNumCameras> kCameraNames{
  "oak_port", "oak_forward", "oak_starboard", "oak_aft"};
inline constexpr std::array<const char *, kNumCameras> kCameraLabels{
  "port", "fwd", "stbd", "aft"};

// One segmentation frame prepared for re-simulation: the decoded rgb8 mask plus
// the camera and boat poses resolved from TF at the frame stamp, all expressed in
// the world (bizzy/map_tide) frame. This is exactly the geometry
// `sea_surface_segmentation::accumulate_frame` consumes, captured once so a param
// sweep can replay the window from memory without re-reading the bag. `cam` is the
// source camera (index into kCameraNames / LoadedBag::camera_models) — all four
// cameras' frames share one merged, time-sorted timeline so they fuse into a
// single occupancy buffer, mirroring the deployed multi-camera SeaSurfaceLayer.
struct PreparedFrame
{
  int cam = 0;                          // index into kCameraNames / camera_models
  double stamp_s = 0.0;
  cv::Mat mask_rgb8;                    // rgb8 segmentation (R=obstacle, G=water, B=sky)
  cv::Vec3d camera_origin;             // optical centre in the world frame
  cv::Matx33d rotation_cam_to_target;  // camera-optical → world rotation
  double boat_x = 0.0;                 // boat (base_link) world XY — buffer window centre
  double boat_y = 0.0;
};

// A decoded display image (camera RGB) at a bag stamp. Display-only — never
// projected. Populated from the H.265 `image_raw/ffmpeg` stream.
struct RgbFrame
{
  int cam = 0;
  double stamp_s = 0.0;
  cv::Mat bgr;  // CV_8UC3, BGR (ready for cv_qt with bgr=true)
};

// A recorded nav2 costmap sample (the boat's own /…/local_costmap/costmap at a
// bag stamp), flattened to the fields the boat-centred render needs — kept here
// so the engine can sample it without depending on nav_msgs. Display-only.
struct RecordedCostmap
{
  double stamp_s = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double resolution = 0.0;  // <= 0 == invalid/empty
  int width = 0;
  int height = 0;
  std::vector<int8_t> data;  // row-major, nav2 convention (-1 unknown, 0..100 cost)
};

// Everything one bag yields for the tuner: a per-camera model table (indexed by
// camera index; an absent camera leaves a default-constructed entry), the merged
// time-sorted segmentation frames that drive accumulation, and two display-only
// side timelines — per-camera RGB and the recorded costmap — also time-sorted.
struct LoadedBag
{
  std::vector<image_geometry::PinholeCameraModel> camera_models;  // size kNumCameras
  std::vector<PreparedFrame> frames;     // merged across cameras, ordered by stamp
  std::vector<RgbFrame> rgb_frames;      // per-camera RGB, ordered by stamp
  std::vector<RecordedCostmap> costmaps;  // recorded costmap, ordered by stamp

  // True when any camera's segmentation came from the lossy-capable
  // `.../compressed` topic because its raw `Image` topic was absent. The UI
  // surfaces this: a JPEG-compressed mask corrupts the R-channel obstacle
  // probability the #22 softmax reads, so tuned values from such a bag are
  // suspect. Raw is preferred per-camera whenever present.
  bool used_compressed_segmentation = false;

  // Count of segmentation frames dropped because no TF was available at their
  // stamp (so no pose to project from). A large value relative to `frames` means
  // a degraded TF stream — surfaced by --probe so a partial load isn't silent.
  std::size_t frames_skipped_no_tf = 0;
};

// What/where to read. Defaults match the BizzyBoat recording convention.
struct BagLoadOptions
{
  double start_s = 0.0;
  double end_s = -1.0;  // < 0 == to end of bag
  std::string world_frame = "bizzy/map_tide";
  std::string boat_frame = "bizzy/base_link";
};

// A bag opened once for repeated windowed reads. The constructor performs the
// single full-file scan that builds the persistent TF cache + per-camera models
// + seg-source selection + bag time bounds; `loadWindow` then reads only the
// requested span, reusing that cached TF/model state (no second discovery scan).
// It does re-open a `rosbag2_cpp::Reader` and iterate, skipping by
// `recv_timestamp` until the window — a future optimization is `Reader::seek` to
// the window start instead of a linear skip. This is the stateful backbone of the
// windowed File->Open buffering (Milestone D): the UI keeps one BagSession and
// reloads spans as the timeline is scrubbed.
//
// Window gating uses bag receive time (recv_timestamp), matching the reference
// driver and the original load_bag — the caller (the buffer manager) pads the
// requested start to cover stamp-time warm-up.
class BagSession
{
public:
  // Full scan: build the TF cache (sized to the bag's own duration so lookups
  // anywhere in the recording succeed — a fixed cache would evict the start of a
  // long bag), resolve each present camera's model, and select its seg source
  // (raw Image preferred, else CompressedImage). `world_frame`/`boat_frame` come
  // from `opts`; `opts.start_s`/`end_s` are NOT applied here — they bound the
  // whole session and are intersected per `loadWindow` call. Throws
  // std::runtime_error if the bag opens with no usable camera (model + seg).
  BagSession(const std::string & bag_uri, const BagLoadOptions & opts);
  ~BagSession();

  BagSession(const BagSession &) = delete;
  BagSession & operator=(const BagSession &) = delete;

  // Decode + project every segmentation frame whose recv time lies in
  // [start_s, end_s] (seconds from bag start; end_s < 0 == to end), resolving
  // each frame's camera/boat pose from the cached TF, and decode the H.265 RGB +
  // recorded costmap over the same span. Frames with no TF at their stamp are
  // skipped (counted in LoadedBag::frames_skipped_no_tf). The session's own
  // [start_s, end_s] clamp (from the ctor opts) is intersected with the request.
  // Throws std::runtime_error if the resulting window has zero usable frames.
  LoadedBag loadWindow(double start_s, double end_s) const;

  // Bag time bounds, in seconds from bag start. `duration_s()` is the span the
  // whole-bag scrubber covers; both are known after construction without loading
  // any frames.
  double duration_s() const {return duration_s_;}

private:
  std::string bag_uri_;
  BagLoadOptions opts_;
  std::unique_ptr<tf2::BufferCore> tf_buffer_;
  std::vector<image_geometry::PinholeCameraModel> camera_models_;  // size kNumCameras
  std::array<std::string, kNumCameras> optical_frame_{};
  std::array<bool, kNumCameras> have_model_{};
  // Chosen seg-source topic per camera ("" == camera absent) + whether it is the
  // compressed variant. An empty session (no usable camera) never constructs.
  std::array<std::string, kNumCameras> seg_source_{};
  std::array<bool, kNumCameras> seg_compressed_{};
  bool used_compressed_segmentation_ = false;
  int64_t bag_start_ns_ = 0;
  double duration_s_ = 0.0;
};

// Two-pass load of all four cameras over [start_s, end_s]: equivalent to
// `BagSession(bag_uri, opts).loadWindow(opts.start_s, opts.end_s)`. Retained as
// the one-shot entry point for `--probe` and as a stable surface for the
// synthetic-frame tests. Throws std::runtime_error if no camera yields a usable
// model or zero frames result overall.
LoadedBag load_bag(const std::string & bag_uri, const BagLoadOptions & opts);

}  // namespace marine_perception_tools

#endif  // BAG_LOADER_HPP_
