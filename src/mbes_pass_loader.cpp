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
#include <sstream>
#include <string>
#include <utility>

#include "cube_bathymetry/projection_summary.h"

#include "mbes_projection.hpp"
#include "mbes_window_reader.hpp"
#include "sidescan_geometry.hpp"

namespace marine_perception_tools
{

namespace
{

// cube's warnings are written for its three COMMAND-LINE tools, so one of them
// tells the reader to check "--*-frame overrides" against a README section.
// The explorer has no such flags: its frames are MbesWindowOptions defaults
// compiled in. Forwarding that sentence into the GUI sends the operator
// looking for a switch that does not exist, so the sentence — and only the
// sentence, never the counts in front of it — is restated for this window.
QString gui_worded(const std::string & line, const MbesWindowOptions & frames)
{
  QString text = QString::fromStdString(line);
  const int cli = text.indexOf("check the --*-frame overrides");
  if (cli >= 0) {
    text.truncate(cli);
    text += QString(
      "the explorer projects through fixed frames (base_link '%1', level "
      "'%2', tide '%3') — check they match this bag's namespaced frames.")
      .arg(QString::fromStdString(frames.base_link_frame))
      .arg(QString::fromStdString(frames.level_frame))
      .arg(QString::fromStdString(frames.tide_frame));
  }
  return text;
}

}  // namespace

void append_projection_notes(
  QStringList & notes, const cube::ProjectionRunTotals & totals,
  int skipped_pings, int invalid_pings, int invalid_beams,
  const MbesWindowOptions & frames)
{
  // A load that projected nothing AND dropped nothing has nothing to report,
  // and a summary of zeros would read as a result. But "projected nothing" is
  // not the same as "nothing happened": pings refused for an unusable sound
  // speed never reach the projector and are not in `totals.pings`, so
  // returning early on that count alone hid the one line that said where they
  // went (#55 review).
  const bool projected = totals.pings > 0;
  const bool dropped = skipped_pings > 0 || invalid_pings > 0 || invalid_beams > 0;
  if (!projected && !dropped) {
    return;
  }
  if (projected) {
    std::ostringstream summary;
    std::ostringstream warnings;
    // cube_bathymetry's own formatter, not a local paraphrase: the counts and
    // the wording then cannot drift from the three offline tools'. The summary
    // line always carries the missing-attitude/heave counts, so a frame
    // mismatch shows even when it does not zero the sounding count.
    cube::report_projection_summary(totals, summary, warnings);
    const auto append_lines =
      [&notes, &frames](const std::string & text) {
        std::istringstream in(text);
        std::string line;
        while (std::getline(in, line)) {
          if (!line.empty()) {
            notes << gui_worded(line, frames);
          }
        }
      };
    append_lines(summary.str());
    // EVERY warning line, not just the summary. This is where the
    // default-beamwidth warning lives, and kongsberg_em_bridge currently
    // leaves rx_beamwidths empty on every M3 ping (marine_tools#85) — so it
    // fires on 100% of beams in exactly this deployment. Discarding it would
    // hide the one warning this deployment always produces.
    append_lines(warnings.str());
    // THE DROP CUBE WILL MAKE, said before the run makes it (#55 review).
    // cube's summary counts missing-attitude pings, but it emits no warning
    // for them and nothing else in the note says what the count costs: a ping
    // with no `level <- base_link` transform still gets finite POSITIONS, so
    // its soundings load, draw, and colour like any others — and then every
    // one of them is dropped by run_cube for a NaN uncertainty. Naming the
    // chain here is what lets the operator tell that apart from a bad
    // selection.
    if (totals.missing_attitude > 0) {
      notes << QString(
        "%1 ping(s) had no attitude transform ('%2' <- '%3'): their soundings "
        "carry NaN uncertainty, so they load and draw but a CUBE run will "
        "drop every one of them.")
        .arg(totals.missing_attitude)
        .arg(QString::fromStdString(frames.level_frame))
        .arg(QString::fromStdString(frames.base_link_frame));
    }
  }
  // The drop populations ProjectionRunTotals has no field for, so that every
  // place a sounding can vanish is visible in the same note.
  notes << QString(
    "Dropped before/around projection: %1 ping(s) with no world TF, "
    "%2 ping(s) with an unusable sound speed, %3 sounding(s) with an "
    "unusable slant range")
    .arg(skipped_pings).arg(invalid_pings).arg(invalid_beams);
  notes << QString::fromUtf8(offline_projection_caveat());
}

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
  // One options value for the whole load: the frames the passes are READ with
  // are the frames the note NAMES, so the two cannot drift apart.
  const MbesWindowOptions frames;
  bool have_ref = false;
  std::string ref_bag;
  std::string ref_frame;
  bool ref_has_geo = false;
  geometry_msgs::msg::TransformStamped ref_earth_from_world;

  // The whole load's projection accounting (#55), summed pass by pass so the
  // note describes the whole load's PROJECTION rather than one pass's. Note
  // what it does not describe: these totals are accumulated as each pass is
  // read, before the contact clip below and before a pass can be skipped for
  // want of a geo anchor, so `soundings` is what the projector produced, not
  // what survived into the cloud. The surviving count is per-pass, in
  // `sounding_counts`, and a clipped or skipped pass leaves its own note.
  cube::ProjectionRunTotals totals;
  totals.reports_georeferencing = false;
  int invalid_pings = 0;
  int invalid_beams = 0;

  for (std::size_t i = 0; i < passes.size(); ++i) {
    if (stop()) {
      return abandon();
    }
    const auto & pass = passes[i];
    MbesWindowResult res;
    try {
      res = read_mbes_window(
        pass.bag_path, pass.t_start_ns, pass.t_end_ns, frames, cancel);
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
    totals.pings += res.diagnostics.pings;
    totals.beams += res.diagnostics.beams;
    totals.soundings += res.diagnostics.soundings;
    totals.filtered_range += res.diagnostics.filtered_range;
    totals.missing_attitude += res.diagnostics.missing_attitude;
    totals.missing_heave += res.diagnostics.missing_heave;
    totals.default_beamwidth_beams += res.diagnostics.default_beamwidth_beams;
    totals.missing_rx_angle_beams += res.diagnostics.missing_rx_angle_beams;
    invalid_pings += res.invalid_pings;
    invalid_beams += res.invalid_beams;

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

  append_projection_notes(
    out.notes, totals, out.skipped_pings, invalid_pings, invalid_beams, frames);
  return out;
}

}  // namespace marine_perception_tools
