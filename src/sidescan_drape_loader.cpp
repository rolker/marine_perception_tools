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

#include "sidescan_drape_loader.hpp"

#include <atomic>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "mbes_window_reader.hpp"   // make_reprojection / apply_reprojection
#include "session_index_io.hpp"

namespace marine_perception_tools
{

DrapePingsResult load_drape_pings(
  const std::string & bag_path, std::int64_t t0_ns, std::int64_t t1_ns,
  const std::string & cache_dir,
  const std::string & ref_bag, bool ref_has_geo,
  const geometry_msgs::msg::TransformStamped & ref_earth_from_world,
  const std::shared_ptr<std::atomic<bool>> & cancel)
{
  DrapePingsResult out;
  const auto stop = [&cancel]() {
      return cancel && cancel->load(std::memory_order_relaxed);
    };
  const auto abandon = []() {
      DrapePingsResult c;
      c.cancelled = true;   // ok stays false: nothing here may be drawn
      return c;
    };
  if (stop()) {return abandon();}
  try {
    SidescanBagSession session(bag_path);

    // Bag-index cache, same contract as the viewer's open path: a valid
    // cache replaces the whole-bag metadata scan; a miss scans and saves.
    const std::string cache_path =
      cache_dir.empty() ? std::string() : cachePathFor(cache_dir, bag_path);
    bool adopted = false;
    if (!cache_path.empty()) {
      const auto identity = bagIdentity(bag_path);
      if (auto cached = loadSessionIndex(cache_path, identity)) {
        session.adoptIndex(std::move(*cached));
        adopted = true;
      }
    }
    if (!adopted) {
      session.buildIndex({}, cancel);
      if (stop()) {
        return abandon();   // a partial index must never poison the cache
      }
      if (!cache_path.empty()) {
        if (const auto snap = session.snapshot()) {
          const auto identity = bagIdentity(bag_path);
          saveSessionIndex(cache_path, identity, *snap);   // best-effort
        }
      }
      out.notes.push_back("bag scanned (no index cache hit)");
    }

    const auto snap = session.snapshot();
    if (!snap) {
      out.notes.push_back("bag produced no index");
      return out;
    }
    const auto interval = distance_interval(*snap, t0_ns, t1_ns);
    if (!interval) {
      out.notes.push_back("pass window matched no posed pings");
      return out;
    }
    // Both channels of the interval, uncapped (the drape decimates onto the
    // grid itself; a cap here would thin the near-nadir coverage).
    out.pings = session.readWindow(interval->first, interval->second, 0, false, cancel);
    if (stop()) {
      return abandon();
    }
    if (out.pings.empty()) {
      out.notes.push_back("no paintable pings in the interval");
      return out;
    }

    // Cross-bag reprojection into the reference frame, composed through the
    // earth anchors (the cloud loader's pattern). The sidescan pose is
    // planar: positions transform at z = 0 and the yaw re-derives from a
    // transformed heading step — display-grade for ENU world frames, whose
    // relative rotation is yaw-dominant.
    if (bag_path != ref_bag) {
      if (ref_has_geo && snap->has_geo_reference) {
        geometry_msgs::msg::TransformStamped src;
        src.transform.translation.x = snap->geo_tx;
        src.transform.translation.y = snap->geo_ty;
        src.transform.translation.z = snap->geo_tz;
        src.transform.rotation.x = snap->geo_qx;
        src.transform.rotation.y = snap->geo_qy;
        src.transform.rotation.z = snap->geo_qz;
        src.transform.rotation.w = snap->geo_qw;
        const FrameReprojection reproject =
          make_reprojection(ref_earth_from_world, src);
        for (auto & ping : out.pings) {
          auto & g = ping.geometry;
          double hx = g.sensor_x + std::cos(g.yaw);
          double hy = g.sensor_y + std::sin(g.yaw);
          double hz = 0.0;
          double z = 0.0;
          apply_reprojection(reproject, g.sensor_x, g.sensor_y, z);
          apply_reprojection(reproject, hx, hy, hz);
          g.yaw = std::atan2(hy - g.sensor_y, hx - g.sensor_x);
        }
      } else {
        out.notes.push_back(
          "no geo anchor to relate the pass's frame to the reference — "
          "assuming a shared world frame");
      }
    }
    out.ok = true;
  } catch (const std::exception & e) {
    out.notes.push_back(std::string("drape load failed: ") + e.what());
  }
  return out;
}

}  // namespace marine_perception_tools
