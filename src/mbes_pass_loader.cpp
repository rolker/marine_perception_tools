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

#include "mbes_pass_loader.hpp"

#include <QString>

#include <algorithm>
#include <atomic>
#include <exception>
#include <memory>
#include <utility>

#include "mbes_window_reader.hpp"
#include "sidescan_geometry.hpp"

namespace marine_perception_tools
{

CloudLoadOutcome load_cloud_passes(
  const std::vector<CloudPassInfo> & passes, const std::optional<GeoClip> & clip,
  const std::shared_ptr<std::atomic<bool>> & cancel)
{
  CloudLoadOutcome out;
  const auto stop = [&cancel]() {
      return cancel && cancel->load(std::memory_order_relaxed);
    };
  // Cancelled: hand back an empty, explicitly-cancelled outcome. Nothing
  // downstream may paint a legend or a cloud out of half a load (#44).
  const auto abandon = []() {
      CloudLoadOutcome c;
      c.cancelled = true;
      return c;
    };
  out.pass_clouds.resize(passes.size());
  out.sounding_counts.assign(passes.size(), 0);

  // The FIRST pass's bag defines the reference world frame; other bags'
  // soundings reproject through the earth anchor (plan #21 design decision —
  // local map origins are not guaranteed identical across recordings).
  bool have_ref = false;
  std::string ref_bag;
  std::string ref_frame;
  bool ref_has_geo = false;
  geometry_msgs::msg::TransformStamped ref_earth_from_world;

  for (std::size_t i = 0; i < passes.size(); ++i) {
    if (stop()) {
      return abandon();
    }
    const auto & pass = passes[i];
    MbesWindowResult res;
    try {
      res = read_mbes_window(pass.bag_path, pass.t_start_ns, pass.t_end_ns, {}, cancel);
    } catch (const std::exception & e) {
      ++out.skipped_passes;
      out.notes << QString("%1: bag read failed (%2)")
        .arg(QString::fromStdString(pass.label)).arg(e.what());
      continue;
    }
    if (res.cancelled) {
      return abandon();   // the reader stopped mid-bag; so does the loader
    }
    out.skipped_pings += res.skipped_pings;

    // Contact clip: drop soundings outside the margin, in THIS bag's own
    // world frame (geo point -> ECEF -> inverse earth anchor). Without a geo
    // anchor the pass cannot be clipped — keep it whole, and say so.
    if (clip && !res.world_soundings.empty()) {
      if (res.has_geo) {
        double ex = 0.0;
        double ey = 0.0;
        double ez = 0.0;
        geodetic_to_ecef(clip->lat, clip->lon, clip->alt, ex, ey, ez);
        const auto & t = res.earth_from_world.transform;
        double cx = 0.0;
        double cy = 0.0;
        double cz = 0.0;
        rotate_by_quat(
          -t.rotation.x, -t.rotation.y, -t.rotation.z, t.rotation.w,
          ex - t.translation.x, ey - t.translation.y, ez - t.translation.z,
          cx, cy, cz);
        if (clip->isBox()) {
          // Box clip (#27): axis-aligned about the centre in this frame.
          const double he = clip->half_east_m;
          const double hn = clip->half_north_m;
          res.world_soundings.erase(
            std::remove_if(
              res.world_soundings.begin(), res.world_soundings.end(),
              [&](const MbesSounding & s) {
                return std::abs(s.x - cx) > he || std::abs(s.y - cy) > hn;
              }),
            res.world_soundings.end());
        } else {
          const double m2 = clip->margin_m * clip->margin_m;
          res.world_soundings.erase(
            std::remove_if(
              res.world_soundings.begin(), res.world_soundings.end(),
              [&](const MbesSounding & s) {
                const double dx = s.x - cx;
                const double dy = s.y - cy;
                return dx * dx + dy * dy > m2;
              }),
            res.world_soundings.end());
        }
      } else {
        out.notes << QString("%1: no geo anchor — not clipped")
          .arg(QString::fromStdString(pass.label));
      }
    }
    if (res.world_soundings.empty()) {
      out.notes << QString("%1: no soundings %2")
        .arg(QString::fromStdString(pass.label))
        .arg(clip ? "within the contact margin" : "in window");
      continue;
    }

    if (!have_ref) {
      have_ref = true;
      ref_bag = pass.bag_path;
      ref_frame = res.world_frame;
      ref_has_geo = res.has_geo;
      ref_earth_from_world = res.earth_from_world;
      // Surface the reference identity (#29): the sidescan drape reprojects
      // its pings into this same frame later.
      out.ref_bag = ref_bag;
      out.ref_frame = ref_frame;
      out.ref_has_geo = ref_has_geo;
      out.ref_earth_from_world = ref_earth_from_world;
    }

    FrameReprojection reproject;   // identity by default
    if (pass.bag_path != ref_bag) {
      if (ref_has_geo && res.has_geo) {
        reproject = make_reprojection(ref_earth_from_world, res.earth_from_world);
      } else if (res.world_frame != ref_frame) {
        // No geo anchor and a different frame: placement would be a guess —
        // skip visibly rather than mis-place soundings.
        ++out.skipped_passes;
        out.notes << QString("%1: no geo anchor to relate frame '%2' to '%3' — skipped")
          .arg(QString::fromStdString(pass.label))
          .arg(QString::fromStdString(res.world_frame))
          .arg(QString::fromStdString(ref_frame));
        continue;
      }
      // Same frame NAME without geo anchors: assume the shared local frame
      // (single-deployment recordings); identity.
    }

    auto & cloud = out.pass_clouds[i];
    cloud = std::move(res.world_soundings);
    for (auto & s : cloud) {
      apply_reprojection(reproject, s.x, s.y, s.z);
    }
    out.sounding_counts[i] = static_cast<int>(cloud.size());
  }
  return out;
}

}  // namespace marine_perception_tools
