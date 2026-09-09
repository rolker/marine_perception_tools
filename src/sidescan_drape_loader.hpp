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

#ifndef SIDESCAN_DRAPE_LOADER_HPP_
#define SIDESCAN_DRAPE_LOADER_HPP_

// Loads one sidescan pass's pings — both channels of the interval — with
// sample data, reprojected into the CUBE lab's reference world frame (#29).
// Rides the hardened session machinery: SidescanBagSession + the bag-index
// cache (a cache hit replaces the whole-bag metadata scan), the
// time->distance interval mapping, and the earth-anchor reprojection the
// MBES cloud loader uses. The sidescan pose is planar, so the cross-bag
// reprojection is applied in the horizontal plane (ENU world frames differ
// by translation + yaw to display grade); altitude is height above bottom
// and rides through unchanged.

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sidescan_bag_session.hpp"

namespace marine_perception_tools
{

struct DrapePingsResult
{
  std::vector<WindowPing> pings;   // reference world frame
  std::vector<std::string> notes;  // human-readable problems
  bool ok = false;
  bool cancelled = false;   // abandoned mid-load (#44): ok stays false
};

// `cache_dir` empty disables the bag-index cache (full scan every time).
// `ref_bag`/`ref_has_geo`/`ref_earth_from_world` identify the reference
// frame (CloudLoadOutcome's ref_* fields). A pass from the reference bag
// needs no reprojection; another bag composes through the earth anchors,
// falling back to identity (with a note) when either anchor is missing.
//
// `cancel` (optional) rides into the whole-bag index scan and the window
// re-read, both of which poll it per bag message (#44) — on a cache miss this
// call is a full scan of a multi-GB recording, so a check only around the
// phases would bound nothing. A cancelled load returns `cancelled` with
// ok = false, and never writes its partial index to the cache.
DrapePingsResult load_drape_pings(
  const std::string & bag_path, std::int64_t t0_ns, std::int64_t t1_ns,
  const std::string & cache_dir,
  const std::string & ref_bag, bool ref_has_geo,
  const geometry_msgs::msg::TransformStamped & ref_earth_from_world,
  const std::shared_ptr<std::atomic<bool>> & cancel = {});

}  // namespace marine_perception_tools

#endif  // SIDESCAN_DRAPE_LOADER_HPP_
