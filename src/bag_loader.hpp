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

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "image_geometry/pinhole_camera_model.hpp"

namespace marine_perception_tools
{

// One forward-camera segmentation frame prepared for re-simulation: the decoded
// rgb8 mask plus the camera and boat poses resolved from TF at the frame stamp,
// all expressed in the world (bizzy/map_tide) frame. This is exactly the geometry
// `sea_surface_segmentation::accumulate_frame` consumes, captured once so a
// param sweep can replay the window from memory without re-reading the bag.
struct PreparedFrame
{
  double stamp_s = 0.0;
  cv::Mat mask_rgb8;                    // rgb8 segmentation (R=obstacle, G=water, B=sky)
  cv::Vec3d camera_origin;             // optical centre in the world frame
  cv::Matx33d rotation_cam_to_target;  // camera-optical → world rotation
  double boat_x = 0.0;                 // boat (base_link) world XY — buffer window centre
  double boat_y = 0.0;
};

// All usable forward-camera frames in the requested window plus the single camera
// model they share (populated from the camera's CameraInfo).
struct LoadedBag
{
  image_geometry::PinholeCameraModel camera_model;
  std::vector<PreparedFrame> frames;  // ordered by bag time
};

// What/where to read. Defaults match the BizzyBoat recording convention.
struct BagLoadOptions
{
  double start_s = 0.0;
  double end_s = -1.0;  // < 0 == to end of bag
  std::string camera = "oak_forward";
  std::string world_frame = "bizzy/map_tide";
  std::string boat_frame = "bizzy/base_link";
};

// Two-pass load of the forward camera: pass 1 fills a TF cache + the camera model;
// pass 2 decodes each segmentation frame in [start_s, end_s] and resolves its
// camera/boat pose from TF. Frames whose TF is unavailable at their stamp are
// skipped (not emitted with stale geometry). Throws std::runtime_error if the bag
// lacks the camera's segmentation/camera_info topic or yields zero usable frames.
LoadedBag load_forward_camera(const std::string & bag_uri, const BagLoadOptions & opts);

}  // namespace marine_perception_tools

#endif  // BAG_LOADER_HPP_
