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
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "image_geometry/pinhole_camera_model.hpp"

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

// Two-pass load of all four cameras: pass 1 fills a TF cache + each present
// camera's model; pass 2 decodes every segmentation frame in [start_s, end_s]
// (raw Image preferred, else CompressedImage), resolves its camera/boat pose from
// TF, and merges the frames into one time-sorted timeline. Frames whose TF is
// unavailable at their stamp are skipped (not emitted with stale geometry).
// Throws std::runtime_error if no camera yields a usable model or zero frames
// result overall.
LoadedBag load_bag(const std::string & bag_uri, const BagLoadOptions & opts);

}  // namespace marine_perception_tools

#endif  // BAG_LOADER_HPP_
