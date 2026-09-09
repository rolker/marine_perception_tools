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
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "cube_lab.hpp"

namespace
{

using marine_perception_tools::CubeShade;
using marine_perception_tools::CubeSurface;
using marine_perception_tools::MbesSounding;
using marine_perception_tools::build_cube_mesh;
using marine_perception_tools::run_cube;

// A dense flat patch at z = -10 m: 0.25 m sounding spacing over 5x5 m.
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
      out.push_back(s);
    }
  }
  return out;
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
  const auto surface = run_cube(flatPatch(-10.0f, -27.5f), 0.5, "order1a",
      tuning);
  // (Baseline patch has NaN beam angles -> correction identity.)
  const auto corrected = run_cube(soundings, 0.5, "order1a", tuning);
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
  const auto surface = run_cube(soundings, 0.5, "order1a", tuning);
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
  const auto refused = run_cube(flatPatch(), 0.5, "order1a", tight);
  EXPECT_FALSE(refused.ok());
  EXPECT_NE(refused.note.find("max-nodes limit"), std::string::npos);
  EXPECT_TRUE(run_cube(flatPatch(), 0.5).ok());   // default limit admits it
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
  const auto surface = run_cube(flatPatch(), 0.5, "order1a",
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
  const auto surface = run_cube(flatPatch(), 0.5, "order1a",
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
