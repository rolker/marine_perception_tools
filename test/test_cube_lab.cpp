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
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cube_lab.hpp"

namespace
{

using marine_perception_tools::CubeShade;
using marine_perception_tools::CubeSurface;
using marine_perception_tools::MbesSounding;
using marine_perception_tools::build_cube_mesh;
using marine_perception_tools::run_cube;

// A dense flat patch at z = -10 m: 0.25 m sounding spacing over 5x5 m. Every
// beam is nadir over its own depth — geometry run_cube now requires to give a
// sounding an uncertainty at all (#49); a beam with none is dropped.
std::vector<MbesSounding> flatPatch(float z = -10.0f, float intensity = -30.0f)
{
  std::vector<MbesSounding> out;
  for (int i = 0; i <= 20; ++i) {
    for (int j = 0; j <= 20; ++j) {
      MbesSounding s;
      s.x = 100.0 + 0.25 * i;
      s.y = 200.0 + 0.25 * j;
      s.z = z;
      s.intensity = intensity;
      s.beam_angle = 0.0f;
      s.slant_range = std::abs(z);
      out.push_back(s);
    }
  }
  return out;
}

// A sounding with no beam geometry cannot be given an angle-aware uncertainty
// (#49), so it is dropped rather than inserted with a fabricated one — and the
// run says so instead of quietly estimating from fewer beams than it was given.
TEST(RunCube, SoundingsWithoutBeamGeometryAreSkippedAndNoted)
{
  auto soundings = flatPatch();
  soundings[0].beam_angle = std::numeric_limits<float>::quiet_NaN();
  soundings[1].slant_range = std::numeric_limits<float>::quiet_NaN();
  const auto surface = run_cube(soundings, 0.5);
  ASSERT_TRUE(surface.ok()) << surface.note;
  EXPECT_EQ(surface.soundings_in, soundings.size());   // what it was handed
  EXPECT_NE(surface.note.find("2 sounding(s) skipped"), std::string::npos)
    << surface.note;
}

TEST(RunCube, FlatPatchEstimatesThePlane)
{
  const auto surface = run_cube(flatPatch(), 0.5);
  ASSERT_TRUE(surface.ok()) << surface.note;
  EXPECT_EQ(surface.soundings_in, 441u);
  // Interior nodes carry estimates near the plane depth.
  int estimated = 0;
  for (const float d : surface.depth) {
    if (std::isfinite(d)) {
      ++estimated;
      EXPECT_NEAR(d, -10.0f, 0.2f);
    }
  }
  EXPECT_GT(estimated, 50);
}

TEST(RunCube, IntensityRidesTheHypothesis)
{
  const auto surface = run_cube(flatPatch(-10.0f, -27.5f), 0.5);
  ASSERT_TRUE(surface.ok());
  bool saw_intensity = false;
  for (std::size_t i = 0; i < surface.depth.size(); ++i) {
    if (std::isfinite(surface.depth[i]) && std::isfinite(surface.intensity[i])) {
      saw_intensity = true;
      EXPECT_NEAR(surface.intensity[i], -27.5f, 1.0f);
    }
  }
  EXPECT_TRUE(saw_intensity);
}

TEST(CubeSurfaceOk, RejectsSurfacesThatWouldBreakItsConsumers)
{
  // ok() is the guard every consumer checks before dividing by cell_m and
  // indexing all three arrays at y * nx + x; it must encode that invariant.
  CubeSurface s;
  s.nx = 2;
  s.ny = 2;
  s.cell_m = 0.5;
  s.depth.assign(4, 0.0f);
  s.uncertainty.assign(4, 0.0f);
  s.intensity.assign(4, 0.0f);
  EXPECT_TRUE(s.ok());

  CubeSurface no_cell = s;
  no_cell.cell_m = 0.0;
  EXPECT_FALSE(no_cell.ok());   // consumers divide by this

  CubeSurface short_unc = s;
  short_unc.uncertainty.pop_back();
  EXPECT_FALSE(short_unc.ok());   // indexed in lockstep with depth

  CubeSurface short_int = s;
  short_int.intensity.clear();
  EXPECT_FALSE(short_int.ok());

  CubeSurface empty;
  EXPECT_FALSE(empty.ok());
}

TEST(RunCube, EmptyAndDegenerateInputsFailLoud)
{
  EXPECT_FALSE(run_cube({}, 0.1).ok());
  EXPECT_FALSE(run_cube(flatPatch(), 0.0).ok());
  EXPECT_FALSE(run_cube(flatPatch(), -1.0).ok());
}

TEST(RunCube, AraCurveCorrectsSettledBackscatter)
{
  // All soundings at rx 30 deg; the curve says 30 deg sits -6 dB relative
  // to nadir, so the corrected (settled) intensity is raw + 6 dB.
  auto soundings = flatPatch(-10.0f, -27.5f);
  for (auto & s : soundings) {
    s.beam_angle = static_cast<float>(30.0 * M_PI / 180.0);
  }
  const char * dir = std::getenv("TMPDIR");
  const std::string csv =
    std::string(dir ? dir : "/tmp") + "/test_ara_curve.csv";
  {
    // The derive tool's format: >= 4 columns, angle in column 0 and
    // db_relative_to_nadir in column 3.
    std::ofstream out(csv);
    out << "# tl_removed: false\n";
    out << "abs_angle_deg_center,n,mean_db,db_relative_to_nadir\n";
    out << "0,1,0,0\n15,1,0,-3\n30,1,0,-6\n45,1,0,-9\n";
  }
  marine_perception_tools::CubeTuning tuning;
  tuning.ara_curve_path = csv;
  const auto surface = run_cube(flatPatch(-10.0f, -27.5f), 0.5,
      tuning);
  // (Baseline patch has NaN beam angles -> correction identity.)
  const auto corrected = run_cube(soundings, 0.5, tuning);
  std::remove(csv.c_str());
  ASSERT_TRUE(corrected.ok()) << corrected.note;
  EXPECT_NE(corrected.note.find("ARA corrected"), std::string::npos);
  bool saw = false;
  for (std::size_t i = 0; i < corrected.depth.size(); ++i) {
    if (std::isfinite(corrected.depth[i]) &&
      std::isfinite(corrected.intensity[i]))
    {
      saw = true;
      EXPECT_NEAR(corrected.intensity[i], -27.5f + 6.0f, 1.0f);
    }
  }
  EXPECT_TRUE(saw);
  // The NaN-angle baseline stays uncorrected (identity through the curve).
  for (std::size_t i = 0; i < surface.depth.size(); ++i) {
    if (std::isfinite(surface.depth[i]) && std::isfinite(surface.intensity[i])) {
      EXPECT_NEAR(surface.intensity[i], -27.5f, 1.0f);
    }
  }
}

TEST(RunCube, SelfCalCurveFlattensArbitraryGainBehaviour)
{
  // A sonar with made-up gain: raw = -30 - TL(R) - 0.2 deg quadratic-ish
  // angular droop. Derive the curve from the beams themselves, feed it back,
  // and the settled backscatter must come out flat at ~-30 dB.
  const double alpha = 0.05;
  std::vector<MbesSounding> soundings;
  for (int p = 0; p < 220; ++p) {   // 220 pings: >200 beams per 2-deg bin
    for (int a = 0; a <= 60; a += 2) {
      const double th = a * M_PI / 180.0;
      MbesSounding s;
      s.x = 100.0 + 0.25 * p;
      s.y = 200.0 + 10.0 * std::tan(th);
      s.z = -10.0;
      s.beam_angle = static_cast<float>(th);
      s.slant_range = static_cast<float>(10.0 / std::cos(th));
      const double tl = 40.0 * std::log10(s.slant_range) +
        2.0 * alpha * s.slant_range;
      s.intensity = static_cast<float>(-30.0 - tl - 0.005 * a * a);
      soundings.push_back(s);
    }
  }
  const char * dir = std::getenv("TMPDIR");
  const std::string csv =
    std::string(dir ? dir : "/tmp") + "/test_selfcal_curve.csv";
  ASSERT_EQ(
    marine_perception_tools::derive_box_curve(soundings, alpha, csv), "");
  marine_perception_tools::CubeTuning tuning;
  tuning.ara_curve_path = csv;
  const auto surface = run_cube(soundings, 0.5, tuning);
  std::remove(csv.c_str());
  ASSERT_TRUE(surface.ok()) << surface.note;
  EXPECT_NE(surface.note.find("tier-2 TL"), std::string::npos);
  int checked = 0;
  for (std::size_t i = 0; i < surface.depth.size(); ++i) {
    if (std::isfinite(surface.depth[i]) &&
      std::isfinite(surface.intensity[i]))
    {
      ++checked;
      EXPECT_NEAR(surface.intensity[i], -30.0f, 1.0f);
    }
  }
  EXPECT_GT(checked, 50);
}

TEST(RunCube, MaxNodesIsOperatorOwnedNotHidden)
{
  // The grid guard is the tuning's max_nodes — tightening it fails loud
  // with the limit named, and raising it lets the same input run.
  marine_perception_tools::CubeTuning tight;
  tight.max_nodes = 10;
  const auto refused = run_cube(flatPatch(), 0.5, tight);
  EXPECT_FALSE(refused.ok());
  EXPECT_NE(refused.note.find("max-nodes limit"), std::string::npos);
  EXPECT_TRUE(run_cube(flatPatch(), 0.5).ok());   // default limit admits it
}

// --- the uncertainty budget, set directly (#45) ------------------------------

TEST(IhoBudget, PresetsSeedTheTwoThresholds)
{
  using marine_perception_tools::iho_preset_limits;
  using marine_perception_tools::iho_preset_names;
  const auto names = iho_preset_names();
  // Every offered name is a preset, and no two presets carry the same pair:
  // that is the order1a/order1b collapse, enforced rather than remembered — a
  // dropdown entry that changes nothing is misleading.
  ASSERT_FALSE(names.empty());
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto a = iho_preset_limits(names[i]);
    ASSERT_TRUE(a.has_value()) << names[i];
    for (std::size_t j = i + 1; j < names.size(); ++j) {
      const auto b = iho_preset_limits(names[j]);
      ASSERT_TRUE(b.has_value()) << names[j];
      EXPECT_FALSE(a->first == b->first && a->second == b->second)
        << names[i] << " and " << names[j] << " are the same budget";
    }
  }
  // The S-44 values, from cube::Parameters::setIHOLimits.
  EXPECT_EQ(
    iho_preset_limits("exclusive"), (std::pair<float, float>{0.15f, 0.0075f}));
  EXPECT_EQ(
    iho_preset_limits("special"), (std::pair<float, float>{0.25f, 0.0075f}));
  EXPECT_EQ(
    iho_preset_limits("order1a/1b"), (std::pair<float, float>{0.5f, 0.013f}));
  EXPECT_EQ(
    iho_preset_limits("order2"), (std::pair<float, float>{1.0f, 0.023f}));
  // The library's own vocabulary still resolves to the budget it names.
  EXPECT_EQ(iho_preset_limits("order1a"), iho_preset_limits("order1a/1b"));
  EXPECT_EQ(iho_preset_limits("order1b"), iho_preset_limits("order1a/1b"));
  // "custom" is a selection state, never a preset to seed from.
  EXPECT_FALSE(
    iho_preset_limits(marine_perception_tools::kCustomIhoOrder).has_value());
  EXPECT_FALSE(iho_preset_limits("order3").has_value());
  EXPECT_FALSE(iho_preset_limits("").has_value());
}

TEST(IhoBudget, EditingEitherThresholdFallsOffThePresets)
{
  using marine_perception_tools::iho_order_for_limits;
  using marine_perception_tools::iho_preset_limits;
  using marine_perception_tools::iho_preset_names;
  const std::string custom = marine_perception_tools::kCustomIhoOrder;
  // A preset's own pair maps back to its name (the round trip the dropdown
  // rides when the dialog is accepted unchanged).
  for (const auto & name : iho_preset_names()) {
    const auto limits = iho_preset_limits(name);
    ASSERT_TRUE(limits.has_value());
    EXPECT_EQ(iho_order_for_limits(limits->first, limits->second), name);
  }
  // Editing either number alone takes the selection to custom.
  EXPECT_EQ(iho_order_for_limits(0.6f, 0.013f), custom);
  EXPECT_EQ(iho_order_for_limits(0.5f, 0.02f), custom);
  // Including looser than any order — the sparse geological-mapping case.
  EXPECT_EQ(iho_order_for_limits(3.0f, 0.1f), custom);
}

TEST(IhoBudget, DefaultTuningTakesTheBudgetFromTheLibrary)
{
  // default_cube_tuning() reads a real cube::Parameters, so the seeded budget
  // cannot drift from upstream; the library's default order is order1a.
  const auto d = marine_perception_tools::default_cube_tuning();
  const auto order1 = marine_perception_tools::iho_preset_limits("order1a/1b");
  ASSERT_TRUE(order1.has_value());
  EXPECT_NEAR(d.iho_fixed, order1->first, 1e-6f);
  EXPECT_NEAR(d.iho_percent, order1->second, 1e-6f);
  EXPECT_EQ(
    marine_perception_tools::iho_order_for_limits(d.iho_fixed, d.iho_percent),
    "order1a/1b");
}

TEST(RunCube, UsesTheBudgetInTheTuningNotAPresetName)
{
  // Sparse soundings at 1 m spacing on a 0.25 m grid: whether a node gets an
  // estimate depends on how far each sounding spreads, and the budget scales
  // that radius. A loose hand-set budget must fill more nodes than a tight
  // one — the property the operator is steering by when they set the numbers
  // themselves instead of picking an order.
  std::vector<MbesSounding> sparse;
  for (int i = 0; i <= 8; ++i) {
    for (int j = 0; j <= 8; ++j) {
      MbesSounding s;
      s.x = 100.0 + 1.0 * i;
      s.y = 200.0 + 1.0 * j;
      s.z = -5.0;
      s.intensity = -30.0f;
      s.beam_angle = 0.0f;      // nadir beams (#49: geometry or no sounding)
      s.slant_range = 5.0f;
      sparse.push_back(s);
    }
  }
  const auto estimated = [](const CubeSurface & c) {
      std::size_t n = 0;
      for (const auto d : c.depth) {
        if (std::isfinite(d)) {++n;}
      }
      return n;
    };
  marine_perception_tools::CubeTuning tight;
  tight.iho_fixed = 0.05f;
  tight.iho_percent = 0.001f;
  marine_perception_tools::CubeTuning loose;
  loose.iho_fixed = 5.0f;
  loose.iho_percent = 0.25f;
  const auto tight_surface = run_cube(sparse, 0.25, tight);
  const auto loose_surface = run_cube(sparse, 0.25, loose);
  ASSERT_TRUE(tight_surface.ok()) << tight_surface.note;
  ASSERT_TRUE(loose_surface.ok()) << loose_surface.note;
  EXPECT_GT(estimated(loose_surface), estimated(tight_surface));
  // The same pair reached through a preset gives the same run: the preset is
  // only a seed, the two numbers are the whole of what the run reads.
  const auto order2 = marine_perception_tools::iho_preset_limits("order2");
  ASSERT_TRUE(order2.has_value());
  marine_perception_tools::CubeTuning by_preset;
  by_preset.iho_fixed = order2->first;
  by_preset.iho_percent = order2->second;
  marine_perception_tools::CubeTuning by_hand;
  by_hand.iho_fixed = 1.0f;
  by_hand.iho_percent = 0.023f;
  EXPECT_EQ(
    estimated(run_cube(sparse, 0.25, by_preset)),
    estimated(run_cube(sparse, 0.25, by_hand)));
}

TEST(BuildCubeMesh, TriangulatesEstimatedCellsOnly)
{
  const auto surface = run_cube(flatPatch(), 0.5);
  ASSERT_TRUE(surface.ok());
  const auto lut = marine_colormap::bake_lut(
    marine_colormap::palette(0), marine_colormap::TransferParams{}, 256);
  const auto mesh = build_cube_mesh(surface, CubeShade::Depth, lut);
  ASSERT_FALSE(mesh.positions.empty());
  EXPECT_EQ(mesh.positions.size(), mesh.colors.size());
  EXPECT_EQ(mesh.indices.size() % 3, 0u);
  ASSERT_FALSE(mesh.indices.empty());
  // Every index points at a real vertex.
  const auto n_verts = mesh.positions.size() / 3;
  for (const auto idx : mesh.indices) {
    EXPECT_LT(idx, n_verts);
  }
  // Vertex heights are the estimated plane.
  for (std::size_t i = 2; i < mesh.positions.size(); i += 3) {
    EXPECT_NEAR(mesh.positions[i], -10.0f, 0.2f);
  }
}

TEST(BuildCubeMesh, FlatCellsRenderEachNodeAsOneQuad)
{
  const auto surface = run_cube(flatPatch(), 0.5);
  ASSERT_TRUE(surface.ok());
  const auto lut = marine_colormap::bake_lut(
    marine_colormap::palette(0), marine_colormap::TransferParams{}, 256);
  const auto mesh = build_cube_mesh(
    surface, CubeShade::Depth, lut,
    marine_perception_tools::CubeMeshStyle::CrispStepped);
  ASSERT_FALSE(mesh.positions.empty());
  // 4 vertices + 2 triangles per estimated node, all four sharing the
  // node's depth and colour (no cross-node blending).
  const std::size_t n_verts = mesh.positions.size() / 3;
  EXPECT_EQ(n_verts % 4, 0u);
  EXPECT_EQ(mesh.indices.size(), (n_verts / 4) * 6);
  for (std::size_t q = 0; q + 3 < n_verts; q += 4) {
    const float z0 = mesh.positions[q * 3 + 2];
    for (int k = 1; k < 4; ++k) {
      EXPECT_EQ(mesh.positions[(q + k) * 3 + 2], z0);
      EXPECT_EQ(mesh.colors[(q + k) * 3], mesh.colors[q * 3]);
    }
    // The quad spans exactly one cell.
    EXPECT_NEAR(
      mesh.positions[(q + 1) * 3] - mesh.positions[q * 3],
      surface.cell_m, 1e-5);
  }
}

TEST(BuildCubeMesh, CrispSmoothIsWatertightWithConstantQuadColour)
{
  // The default style: one constant-colour quad per node whose corners take
  // the neighbours' mean height. On a sloped synthetic surface, adjacent
  // quads must share corner heights exactly (watertight membrane) while each
  // quad's four vertices carry ONE colour (no texel blending).
  CubeSurface surface;
  surface.origin_x = 0.0;
  surface.origin_y = 0.0;
  surface.cell_m = 0.5;
  surface.nx = 4;
  surface.ny = 4;
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      surface.depth.push_back(-10.0f - 0.5f * x);   // sloping in x
      surface.uncertainty.push_back(0.1f);
      surface.intensity.push_back(std::nanf(""));
    }
  }
  const auto lut = marine_colormap::bake_lut(
    marine_colormap::palette(0), marine_colormap::TransferParams{}, 256);
  const auto mesh = build_cube_mesh(
    surface, CubeShade::Depth, lut,
    marine_perception_tools::CubeMeshStyle::CrispSmooth);
  const std::size_t n_verts = mesh.positions.size() / 3;
  ASSERT_EQ(n_verts, 4u * 16u);   // 4 verts per node
  // Constant colour per quad.
  for (std::size_t q = 0; q + 3 < n_verts; q += 4) {
    for (int k = 1; k < 4; ++k) {
      EXPECT_EQ(mesh.colors[(q + k) * 3], mesh.colors[q * 3]);
      EXPECT_EQ(mesh.colors[(q + k) * 3 + 1], mesh.colors[q * 3 + 1]);
    }
  }
  // Watertight: vertices at the same (x, y) position share the same z.
  for (std::size_t a = 0; a < n_verts; ++a) {
    for (std::size_t b = a + 1; b < n_verts; ++b) {
      if (mesh.positions[a * 3] == mesh.positions[b * 3] &&
        mesh.positions[a * 3 + 1] == mesh.positions[b * 3 + 1])
      {
        EXPECT_EQ(mesh.positions[a * 3 + 2], mesh.positions[b * 3 + 2]);
      }
    }
  }
  // The slope survives: a quad's east corners sit deeper than its west.
  EXPECT_LT(mesh.positions[1 * 3 + 2], mesh.positions[0 * 3 + 2]);
}

TEST(BuildCubeMesh, ManualRangeOverridesTheAutoRamp)
{
  const auto surface = run_cube(flatPatch(), 0.5);
  ASSERT_TRUE(surface.ok());
  const auto lut = marine_colormap::bake_lut(
    marine_colormap::palette(0), marine_colormap::TransferParams{}, 256);
  const auto mesh = build_cube_mesh(
    surface, CubeShade::Depth, lut,
    marine_perception_tools::CubeMeshStyle::Blended,
    std::pair<float, float>(-20.0f, -5.0f));
  ASSERT_FALSE(mesh.positions.empty());
  EXPECT_EQ(mesh.scalar_lo, -20.0f);
  EXPECT_EQ(mesh.scalar_hi, -5.0f);
}

// Cancellation (#44): a cancelled run must not hand back a grid. The same
// soundings that estimate a plane above produce no surface at all once the
// token is set, and the reason is legible in the note rather than looking
// like an empty box.
TEST(RunCube, CancelledRunYieldsNoSurface)
{
  const auto cancel = std::make_shared<std::atomic<bool>>(true);
  const auto surface = run_cube(flatPatch(), 0.5,
    marine_perception_tools::CubeTuning{}, cancel);
  EXPECT_FALSE(surface.ok());
  EXPECT_EQ(surface.note, "cancelled");
  EXPECT_TRUE(surface.depth.empty());
  EXPECT_EQ(surface.nx, 0);
  EXPECT_EQ(surface.ny, 0);
}

// The token is a cancellation signal, not a switch that refuses work: an
// un-set token must estimate exactly as no token at all.
TEST(RunCube, UnsetCancelTokenEstimatesNormally)
{
  const auto cancel = std::make_shared<std::atomic<bool>>(false);
  const auto surface = run_cube(flatPatch(), 0.5,
    marine_perception_tools::CubeTuning{}, cancel);
  ASSERT_TRUE(surface.ok()) << surface.note;
  EXPECT_EQ(surface.depth.size(), run_cube(flatPatch(), 0.5).depth.size());
}

TEST(BuildCubeMesh, EmptySurfaceYieldsEmptyMesh)
{
  const auto lut = marine_colormap::bake_lut(
    marine_colormap::palette(0), marine_colormap::TransferParams{}, 256);
  const auto mesh = build_cube_mesh(CubeSurface{}, CubeShade::Depth, lut);
  EXPECT_TRUE(mesh.positions.empty());
  EXPECT_TRUE(mesh.indices.empty());
}

}  // namespace


// The operator's report, as a CHARACTERISATION test (#49): a clean near-nadir
// pass and a noisy outer-beam pass over the same seabed. "The smile across a
// ping from the refraction is causing a ping's outer beams to ingest too much
// noise and uncertainty in an otherwise clean, smooth surface from another
// pass's near-nadir beams."
//
// This pins the two-pass behaviour and REPORTS the number rather than claiming
// the angle-aware model wins here. Measured over this case (2026-09-09):
//
//   depth-only placeholder : RMS 0.0079 m over 121 estimated nodes
//   angle-aware (#49)      : RMS 0.0414 m over 165 estimated nodes
//
// The angle-aware model reaches more nodes and tracks the noisy pass more
// closely, and the reason is worth knowing: it makes every sounding's absolute
// uncertainty SMALLER (5 cm at 10 m nadir, against 17 cm before), so CUBE's
// gate for "is this the same seabed?" tightens and a scattered pass splits into
// hypotheses instead of averaging into one. Node depth is then decided by
// Node::chooseHypothesis, which takes the hypothesis with the MOST SAMPLES —
// not the smallest variance. So on zero-mean scatter with equal sample counts,
// plain averaging (the old, looser model) is hard to beat, and the angular
// weighting shows up inside a hypothesis rather than in which one wins.
//
// The weighting itself is real and unit-tested (test_sounding_uncertainty):
// a 65 deg beam carries about twice the vertical variance of a nadir beam over
// the same depth. What this test records is that a variance ratio is not by
// itself a surface improvement on unbiased noise.
//
// It also does NOT correct a refraction SMILE: that bias is systematic, and
// only re-projection removes it (#28).
TEST(RunCube, TwoPassSurfaceOverACleanAndANoisyPassIsCharacterised)
{
  const double truth = -10.0;
  std::vector<MbesSounding> both;
  int k = 0;
  for (int i = 0; i <= 40; ++i) {
    for (int j = 0; j <= 40; ++j) {
      const double x = 100.0 + 0.125 * i;
      const double y = 200.0 + 0.125 * j;
      // Pass A: near-nadir beams, on the true depth.
      MbesSounding a;
      a.x = x;
      a.y = y;
      a.z = truth;
      a.intensity = -30.0f;
      a.beam_angle = static_cast<float>(5.0 * M_PI / 180.0);
      a.slant_range = static_cast<float>(-truth / std::cos(a.beam_angle));
      both.push_back(a);
      // Pass B: the same ground at the swath edge, scattered. Deterministic
      // (a fixed sawtooth), so the number this test reports is reproducible.
      const double noise = 0.4 * ((k % 7) - 3) / 3.0;
      ++k;
      MbesSounding b;
      b.x = x;
      b.y = y;
      b.z = truth + noise;
      b.intensity = -30.0f;
      b.beam_angle = static_cast<float>(65.0 * M_PI / 180.0);
      b.slant_range = static_cast<float>(-truth / std::cos(b.beam_angle));
      both.push_back(b);
    }
  }
  const auto surface = run_cube(both, 0.5);
  ASSERT_TRUE(surface.ok()) << surface.note;
  double sq = 0.0;
  std::size_t n = 0;
  for (const float d : surface.depth) {
    if (std::isfinite(d)) {
      sq += (d - truth) * (d - truth);
      ++n;
    }
  }
  ASSERT_GT(n, 0u);
  const double rms = std::sqrt(sq / static_cast<double>(n));
  // The bound both models clear, and the one that matters operationally: the
  // surface must stay far inside the noisy pass's own 0.23 m RMS scatter — the
  // estimator must never simply follow the outer beams.
  EXPECT_LT(rms, 0.10) << "the noisy outer-beam pass is carrying the surface";
  // Printed, not merely asserted: this is the number quoted above, so a future
  // change to the model shows up here in the open rather than silently.
  std::printf("[#49] two-pass surface RMS error %.4f m over %zu nodes\n", rms, n);
}


// The before/after this change is claimed on (#49), as a test. Two overlapping
// swaths over the same 12 m bottom with a 0.15 m ripple, each 80 pings x 64
// beams over +/-34 deg, with the scatter ANGLE-CORRELATED: near-nadir beams
// clean, outer beams noisy — the operator's case, where one pass's swath edge
// falls on ground another pass covered near nadir.
//
// Measured over this region (2026-09-09, cell 0.5 m):
//
//   depth-only placeholder : RMS 0.0407 m, 2542 of 2752 nodes estimated
//   angle-aware (#49)      : RMS 0.0347 m, 2685 of 2752 nodes estimated
//
// A 15% better surface against truth, over 143 more nodes. The bound below is
// set between the two, so the depth-only model would fail this test: it is the
// improvement, not a restatement of it.
TEST(RunCube, AngleCorrelatedNoiseIsSuppressedWhereSwathsOverlap)
{
  const auto truth_at = [](double y) {return -12.0 + 0.15 * std::sin(y * 0.4);};
  std::vector<MbesSounding> v;
  for (int pass = 0; pass < 2; ++pass) {
    const double track_y = (pass == 0) ? 0.0 : 14.0;   // swaths overlap mid-way
    for (int p = 0; p < 80; ++p) {
      for (int b = 0; b < 64; ++b) {
        const double angle = -0.6 + 1.2 * b / 63.0;
        const double range = 12.0 / std::cos(angle);
        MbesSounding s;
        s.x = 0.25 * p;
        s.y = track_y + range * std::sin(angle);
        // Deterministic sawtooth scatter, scaled by |sin(angle)|: zero at
        // nadir, worst at the swath edge.
        const double scatter = 0.35 * std::abs(std::sin(angle)) *
          (((p * 64 + b) % 7) - 3) / 3.0;
        s.z = truth_at(s.y) + scatter;
        s.intensity = -30.0f;
        s.beam_angle = static_cast<float>(angle);
        s.slant_range = static_cast<float>(range);
        v.push_back(s);
      }
    }
  }
  const auto surface = run_cube(v, 0.5);
  ASSERT_TRUE(surface.ok()) << surface.note;
  double sq = 0.0;
  std::size_t n = 0;
  for (int y = 0; y < surface.ny; ++y) {
    for (int x = 0; x < surface.nx; ++x) {
      const float d = surface.depth[static_cast<std::size_t>(y) * surface.nx + x];
      if (!std::isfinite(d)) {continue;}
      const double dy = d - truth_at(surface.origin_y + y * surface.cell_m);
      sq += dy * dy;
      ++n;
    }
  }
  ASSERT_GT(n, 2000u) << surface.note;
  const double rms = std::sqrt(sq / static_cast<double>(n));
  EXPECT_LT(rms, 0.038) << "no better than the depth-only model it replaced";
  std::printf("[#49] angle-correlated case: RMS %.4f m over %zu nodes\n", rms, n);
}
