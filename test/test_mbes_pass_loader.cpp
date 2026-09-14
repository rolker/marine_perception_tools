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

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <optional>
#include <vector>

#include "mbes_pass_loader.hpp"
#include "mbes_projection.hpp"

namespace
{

using marine_perception_tools::CloudPassInfo;
using marine_perception_tools::cube_surface_shares_cloud_frame;
using marine_perception_tools::append_projection_notes;
using marine_perception_tools::load_cloud_passes;
using marine_perception_tools::MbesWindowOptions;
using marine_perception_tools::offline_projection_caveat;

TEST(MbesPassLoader, EmptyInputYieldsEmptyOutcome)
{
  const auto out = load_cloud_passes({});
  EXPECT_TRUE(out.pass_clouds.empty());
  EXPECT_TRUE(out.sounding_counts.empty());
  EXPECT_TRUE(out.notes.isEmpty());
  EXPECT_EQ(out.skipped_passes, 0);
  EXPECT_EQ(out.skipped_pings, 0);
}

TEST(MbesPassLoader, MissingBagCostsOnlyItsOwnPass)
{
  CloudPassInfo pass;
  pass.bag_path = "/nonexistent/bag_dir";
  pass.label = "2026-06-15 15:10:00  (bag_a)";
  pass.t_start_ns = 1000;
  pass.t_end_ns = 2000;
  const auto out = load_cloud_passes({pass, pass});

  // Both passes fail (same missing bag) — one slot, one note each; the loader
  // never throws and the per-pass structure stays index-aligned with the input.
  ASSERT_EQ(out.pass_clouds.size(), 2u);
  ASSERT_EQ(out.sounding_counts.size(), 2u);
  EXPECT_TRUE(out.pass_clouds[0].empty());
  EXPECT_TRUE(out.pass_clouds[1].empty());
  EXPECT_EQ(out.sounding_counts[0], 0);
  EXPECT_EQ(out.sounding_counts[1], 0);
  EXPECT_EQ(out.skipped_passes, 2);
  ASSERT_EQ(out.notes.size(), 2);
  EXPECT_TRUE(out.notes[0].contains("bag read failed"));
  EXPECT_TRUE(out.notes[0].contains("2026-06-15 15:10:00"));
}

// Cancellation (#44). The token is checked BEFORE the first bag is opened
// and inside read_mbes_window per message, so a cancelled load does no I/O
// and hands back nothing: no clouds, no notes, no counts a legend could be
// built from. (The bag path here does not exist — an uncancelled load would
// prove it opened it by leaving a "bag read failed" note per pass, which the
// companion test below pins.)
TEST(MbesPassLoader, CancelledLoadReadsNothingAndPublishesNothing)
{
  CloudPassInfo pass;
  pass.bag_path = "/nonexistent/bag_dir";
  pass.label = "2026-06-15 15:10:00  (bag_a)";
  pass.t_start_ns = 1000;
  pass.t_end_ns = 2000;

  const auto cancel = std::make_shared<std::atomic<bool>>(true);
  const auto out = load_cloud_passes({pass, pass}, std::nullopt, cancel);

  EXPECT_TRUE(out.cancelled);
  EXPECT_TRUE(out.pass_clouds.empty());
  EXPECT_TRUE(out.sounding_counts.empty());
  EXPECT_TRUE(out.notes.isEmpty());
  EXPECT_EQ(out.skipped_passes, 0);
  EXPECT_TRUE(out.ref_bag.empty());
}

// The token is a cancellation signal, not a switch that refuses work: an
// un-set token must load exactly as no token at all.
TEST(MbesPassLoader, UnsetCancelTokenLoadsNormally)
{
  CloudPassInfo pass;
  pass.bag_path = "/nonexistent/bag_dir";
  pass.label = "2026-06-15 15:10:00  (bag_a)";
  pass.t_start_ns = 1000;
  pass.t_end_ns = 2000;

  const auto cancel = std::make_shared<std::atomic<bool>>(false);
  const auto out = load_cloud_passes({pass, pass}, std::nullopt, cancel);

  EXPECT_FALSE(out.cancelled);
  ASSERT_EQ(out.pass_clouds.size(), 2u);
  EXPECT_EQ(out.skipped_passes, 2);
  ASSERT_EQ(out.notes.size(), 2);
  EXPECT_TRUE(out.notes[0].contains("bag read failed"));
}

// --- the load's projection note (#55) ---------------------------------------

// Totals shaped like a real M3 load: every beam on the default beamwidth
// (kongsberg_em_bridge reports no per-beam beamwidths), a few range-filtered,
// a few pings with no attitude.
cube::ProjectionRunTotals sampleTotals()
{
  cube::ProjectionRunTotals t;
  t.reports_georeferencing = false;
  t.pings = 120;
  t.beams = 3840;
  t.soundings = 3800;
  t.filtered_range = 40;
  t.missing_attitude = 3;
  t.missing_heave = 2;
  t.default_beamwidth_beams = 3840;
  t.missing_rx_angle_beams = 12;
  return t;
}

// One note per load, carrying all four parts: cube's own summary line, the
// warning lines it writes to the error stream, the drop populations that
// summary has no field for, and the offline-defaults caveat.
TEST(ProjectionNotes, CarryTheSummaryTheWarningsTheDropsAndTheCaveat)
{
  QStringList notes;
  append_projection_notes(notes, sampleTotals(), 5, 1, 7);
  const QString all = notes.join("\n");

  EXPECT_TRUE(all.contains("Offline projection: 120 pings")) << all.toStdString();
  EXPECT_TRUE(all.contains("3800 soundings")) << all.toStdString();
  // The counts that make a frame mismatch visible even when soundings survive.
  EXPECT_TRUE(all.contains("3 missing attitude")) << all.toStdString();
  EXPECT_TRUE(all.contains("2 missing heave")) << all.toStdString();
  // THE WARNING THIS DEPLOYMENT ALWAYS PRODUCES: every M3 beam falls back to
  // the generic device beamwidth. It lives on the error stream, so dropping
  // that stream would hide it entirely.
  EXPECT_TRUE(all.contains("WARNING")) << all.toStdString();
  EXPECT_TRUE(all.contains("generic device across-track beamwidth"))
    << all.toStdString();
  EXPECT_TRUE(all.contains("no usable receive angle")) << all.toStdString();
  // Every other way a sounding can vanish, in the same note.
  EXPECT_TRUE(all.contains("5 ping(s) with no world TF")) << all.toStdString();
  EXPECT_TRUE(all.contains("1 ping(s) with an unusable sound speed"))
    << all.toStdString();
  EXPECT_TRUE(all.contains("7 sounding(s) with an unusable slant range"))
    << all.toStdString();
  // And the caveat, ONCE — the same string the CUBE-tuning dialog shows.
  EXPECT_EQ(notes.filter(QString::fromUtf8(offline_projection_caveat())).size(), 1);
}

// The summary must not claim a georeferencing pass this path does not do: its
// earth-anchor reprojection relates one bag's world frame to another's, which
// is a different thing. Left true, the note would report "0 georeferenced into
// the grid" and warn about a localization chain that was never in question.
TEST(ProjectionNotes, DoNotReportPerSoundingGeoreferencing)
{
  QStringList notes;
  append_projection_notes(notes, sampleTotals(), 0, 0, 0);
  const QString all = notes.join("\n");
  EXPECT_FALSE(all.contains("georeferenced into the grid")) << all.toStdString();
  EXPECT_FALSE(all.contains("earth transform")) << all.toStdString();
}

// A load that projected nothing AND dropped nothing says nothing: a summary of
// zeros reads as a result, and the passes that failed have already left their
// own notes.
TEST(ProjectionNotes, AreSilentWhenNothingWasProjectedAndNothingDropped)
{
  QStringList notes;
  cube::ProjectionRunTotals empty;
  empty.reports_georeferencing = false;
  append_projection_notes(notes, empty, 0, 0, 0);
  EXPECT_TRUE(notes.isEmpty());
}

// ...but "projected nothing" is not "nothing happened". A bag whose every ping
// carries an unusable sound speed never reaches the projector, so `pings` is
// zero while the drop counters are not — and the note that accounts for those
// pings is the ONLY thing that can tell the operator where his data went.
TEST(ProjectionNotes, AccountForDropsEvenWhenNoPingWasProjected)
{
  QStringList notes;
  cube::ProjectionRunTotals empty;
  empty.reports_georeferencing = false;
  append_projection_notes(notes, empty, 2, 40, 0);
  const QString all = notes.join("\n");
  ASSERT_FALSE(notes.isEmpty());
  EXPECT_TRUE(all.contains("40 ping(s) with an unusable sound speed"))
    << all.toStdString();
  EXPECT_TRUE(all.contains("2 ping(s) with no world TF")) << all.toStdString();
  // No summary of zeros alongside it.
  EXPECT_FALSE(all.contains("Offline projection: 0 pings")) << all.toStdString();
}

// A missing-attitude ping is the quiet failure this whole note exists for: the
// position still resolves, so its soundings load, draw and colour like any
// others — and then run_cube drops every one for a NaN uncertainty. The count
// alone does not say that; cube emits no warning for it; so the note says it,
// in the frame names the projection actually used.
TEST(ProjectionNotes, SayWhatAMissingAttitudeTransformWillCostTheRun)
{
  QStringList notes;
  MbesWindowOptions frames;
  append_projection_notes(notes, sampleTotals(), 0, 0, 0, frames);
  const QString all = notes.join("\n");
  EXPECT_TRUE(all.contains("3 ping(s) had no attitude transform"))
    << all.toStdString();
  EXPECT_TRUE(all.contains(QString::fromStdString(frames.level_frame)))
    << all.toStdString();
  EXPECT_TRUE(all.contains(QString::fromStdString(frames.base_link_frame)))
    << all.toStdString();
  EXPECT_TRUE(all.contains("drop every one of them")) << all.toStdString();
}

// ...and nothing is said when there were none: an operator who reads a line
// about dropped pings on a clean load learns to ignore the line.
TEST(ProjectionNotes, SayNothingAboutAttitudeWhenEveryPingHadIt)
{
  QStringList notes;
  cube::ProjectionRunTotals t = sampleTotals();
  t.missing_attitude = 0;
  append_projection_notes(notes, t, 0, 0, 0);
  EXPECT_FALSE(notes.join("\n").contains("no attitude transform"))
    << notes.join("\n").toStdString();
}

// cube's warnings are written for its three command-line tools. One of them
// tells the reader to check "--*-frame overrides" against a README section;
// the explorer has no such flags, and an operator sent looking for one finds
// nothing. The counts in front of the sentence are cube's and stay verbatim;
// only the instruction is restated, naming the frames this window compiled in.
TEST(ProjectionNotes, RestateTheCommandLineFrameAdviceForAWindowWithNoFlags)
{
  cube::ProjectionRunTotals t;
  t.reports_georeferencing = false;
  t.pings = 12;
  t.beams = 384;
  t.soundings = 0;          // every sounding lost...
  t.filtered_range = 384;
  t.missing_attitude = 12;  // ...to a frame the bag does not carry
  QStringList notes;
  MbesWindowOptions frames;
  append_projection_notes(notes, t, 0, 0, 0, frames);
  const QString all = notes.join("\n");
  EXPECT_TRUE(all.contains("projected 0 soundings from 12 pings"))
    << all.toStdString();
  EXPECT_FALSE(all.contains("--*-frame")) << all.toStdString();
  EXPECT_FALSE(all.contains("README")) << all.toStdString();
  EXPECT_TRUE(all.contains(QString::fromStdString(frames.tide_frame)))
    << all.toStdString();
}

// The same, through the loader: every pass here fails to open, so no ping
// reaches the projector and the outcome carries only the per-pass failures.
TEST(ProjectionNotes, AreAbsentFromALoadThatOpenedNoBag)
{
  CloudPassInfo pass;
  pass.bag_path = "/nonexistent/bag_dir";
  pass.label = "2026-06-15 15:10:00  (bag_a)";
  pass.t_start_ns = 1000;
  pass.t_end_ns = 2000;
  const auto out = load_cloud_passes({pass});
  ASSERT_EQ(out.notes.size(), 1);
  EXPECT_TRUE(out.notes[0].contains("bag read failed"));
}

// --- a CUBE surface over the selection cloud (#36) ---------------------------
//
// A finished CUBE run lays its surface over the soundings already on screen
// instead of replacing them. The one thing that can stop it is the frame: the
// surface is a grid in the run's own reference world frame, placed by the
// displayed cloud's centroid, so it may only be drawn over soundings that live
// in that frame. Same reference bag AND frame name is that guarantee.
TEST(CubeSurfaceSharesCloudFrame, KeepsTheCloudWhenBothLoadsResolvedTheSameReference)
{
  EXPECT_TRUE(
    cube_surface_shares_cloud_frame(
      true, true, "/bags/a", "bizzy/map", "/bags/a", "bizzy/map"));
}

// A different reference bag — or the same bag under a different frame name —
// means the two point sets are related by a rigid transform the surface grid
// cannot be re-gridded through without re-running the estimate. Drawing it
// anyway would place bathymetry by luck, so the run falls back to its own
// soundings.
TEST(CubeSurfaceSharesCloudFrame, RefusesAcrossDifferentReferenceFrames)
{
  EXPECT_FALSE(
    cube_surface_shares_cloud_frame(
      true, true, "/bags/a", "bizzy/map", "/bags/b", "bizzy/map"));
  EXPECT_FALSE(
    cube_surface_shares_cloud_frame(
      true, true, "/bags/a", "bizzy/map", "/bags/a", "izzy/map"));
}

// Nothing to preserve: the pane is not showing a selection, or its load has
// not landed yet. An unresolved reference on either side is not a match
// either — a load that resolved no reference produced no soundings at all.
TEST(CubeSurfaceSharesCloudFrame, RefusesWithNothingLoadedOrNoReference)
{
  EXPECT_FALSE(
    cube_surface_shares_cloud_frame(
      false, true, "/bags/a", "bizzy/map", "/bags/a", "bizzy/map"));
  EXPECT_FALSE(
    cube_surface_shares_cloud_frame(
      true, false, "/bags/a", "bizzy/map", "/bags/a", "bizzy/map"));
  EXPECT_FALSE(
    cube_surface_shares_cloud_frame(true, true, "", "", "", ""));
  EXPECT_FALSE(
    cube_surface_shares_cloud_frame(
      true, true, "/bags/a", "bizzy/map", "", ""));
}

}  // namespace
