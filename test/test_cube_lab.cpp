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

#include <cmath>
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

TEST(RunCube, EmptyAndDegenerateInputsFailLoud)
{
  EXPECT_FALSE(run_cube({}, 0.1).ok());
  EXPECT_FALSE(run_cube(flatPatch(), 0.0).ok());
  EXPECT_FALSE(run_cube(flatPatch(), -1.0).ok());
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
  const auto mesh = build_cube_mesh(surface, CubeShade::Depth, lut, true);
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

TEST(BuildCubeMesh, ManualRangeOverridesTheAutoRamp)
{
  const auto surface = run_cube(flatPatch(), 0.5);
  ASSERT_TRUE(surface.ok());
  const auto lut = marine_colormap::bake_lut(
    marine_colormap::palette(0), marine_colormap::TransferParams{}, 256);
  const auto mesh = build_cube_mesh(
    surface, CubeShade::Depth, lut, false,
    std::pair<float, float>(-20.0f, -5.0f));
  ASSERT_FALSE(mesh.positions.empty());
  EXPECT_EQ(mesh.scalar_lo, -20.0f);
  EXPECT_EQ(mesh.scalar_hi, -5.0f);
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
