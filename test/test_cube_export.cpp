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

#include <gdal_priv.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "cube_export.hpp"

namespace
{

using marine_perception_tools::CubeSurface;
using marine_perception_tools::MapGeoAffine;

// A 3x2 surface at 0.5 m cells, origin (100, 200), sloping depth in x; one
// hole. The anchor is a plain equirectangular plane about 43N 71W.
CubeSurface tinySurface()
{
  CubeSurface s;
  s.origin_x = 100.0;
  s.origin_y = 200.0;
  s.cell_m = 0.5;
  s.nx = 3;
  s.ny = 2;
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 3; ++x) {
      s.depth.push_back(-10.0f - x);
      s.uncertainty.push_back(0.25f);
      s.intensity.push_back(-30.0f);
    }
  }
  s.depth[4] = std::nanf("");   // hole at (x=1, y=1)
  return s;
}

MapGeoAffine tinyAnchor()
{
  MapGeoAffine a;
  a.lat0 = 43.0;
  a.lon0 = -71.0;
  a.dlat_dx = 0.0;
  a.dlat_dy = 1.0 / 111320.0;
  a.dlon_dx = 1.0 / (111320.0 * 0.731);
  a.dlon_dy = 0.0;
  return a;
}

std::string tmpPath(const char * name)
{
  const char * dir = std::getenv("TMPDIR");
  return std::string(dir ? dir : "/tmp") + "/" + name;
}

TEST(CubeExport, FloatBandsRoundTrip)
{
  const auto surface = tinySurface();
  const auto path = tmpPath("cube_export_float.tif");
  const auto err = marine_perception_tools::write_cube_geotiff_float(
    surface, tinyAnchor(), path);
  ASSERT_TRUE(err.empty()) << err;

  GDALAllRegister();
  GDALDataset * ds = static_cast<GDALDataset *>(
    GDALOpen(path.c_str(), GA_ReadOnly));
  ASSERT_NE(ds, nullptr);
  EXPECT_EQ(ds->GetRasterXSize(), 3);
  EXPECT_EQ(ds->GetRasterYSize(), 2);
  EXPECT_EQ(ds->GetRasterCount(), 3);

  double gt[6];
  ASSERT_EQ(ds->GetGeoTransform(gt), CE_None);
  // Pixel (0,0) CENTRE = corner + half steps = node (x=0, y=ny-1): map
  // (100, 200.5) -> the anchor's lat/lon.
  const double cx = gt[0] + 0.5 * gt[1] + 0.5 * gt[2];
  const double cy = gt[3] + 0.5 * gt[4] + 0.5 * gt[5];
  EXPECT_NEAR(cx, -71.0 + 100.0 / (111320.0 * 0.731), 1e-9);
  EXPECT_NEAR(cy, 43.0 + 200.5 / 111320.0, 1e-9);

  // Band 1 depth: raster row 0 = grid row 1 (north-up flip); its middle
  // pixel is the hole.
  std::vector<float> row(3);
  GDALRasterBand * band = ds->GetRasterBand(1);
  ASSERT_EQ(band->RasterIO(
      GF_Read, 0, 0, 3, 1, row.data(), 3, 1, GDT_Float32, 0, 0), CE_None);
  EXPECT_EQ(row[0], -10.0f);
  EXPECT_TRUE(std::isnan(row[1]));
  EXPECT_EQ(row[2], -12.0f);
  GDALClose(ds);
  std::remove(path.c_str());
}

TEST(CubeExport, RgbaRoundTrip)
{
  const auto surface = tinySurface();
  std::vector<std::uint8_t> rgba(3 * 2 * 4, 0);
  for (std::size_t i = 0; i < 6; ++i) {
    rgba[i * 4] = static_cast<std::uint8_t>(10 * i);
    rgba[i * 4 + 3] = 255;
  }
  rgba[4 * 4 + 3] = 0;   // the hole is transparent
  const auto path = tmpPath("cube_export_rgba.tif");
  const auto err = marine_perception_tools::write_cube_geotiff_rgba(
    surface, rgba, tinyAnchor(), path);
  ASSERT_TRUE(err.empty()) << err;

  GDALAllRegister();
  GDALDataset * ds = static_cast<GDALDataset *>(
    GDALOpen(path.c_str(), GA_ReadOnly));
  ASSERT_NE(ds, nullptr);
  EXPECT_EQ(ds->GetRasterCount(), 4);
  EXPECT_EQ(ds->GetRasterBand(4)->GetColorInterpretation(), GCI_AlphaBand);
  // Raster row 1 = grid row 0: red channel = 0, 10, 20.
  std::vector<std::uint8_t> row(3);
  ASSERT_EQ(ds->GetRasterBand(1)->RasterIO(
      GF_Read, 0, 1, 3, 1, row.data(), 3, 1, GDT_Byte, 0, 0), CE_None);
  EXPECT_EQ(row[0], 0);
  EXPECT_EQ(row[1], 10);
  EXPECT_EQ(row[2], 20);
  // Alpha of the hole (grid node 4 = raster row 0, col 1) is 0.
  std::vector<std::uint8_t> alpha(3);
  ASSERT_EQ(ds->GetRasterBand(4)->RasterIO(
      GF_Read, 0, 0, 3, 1, alpha.data(), 3, 1, GDT_Byte, 0, 0), CE_None);
  EXPECT_EQ(alpha[1], 0);
  EXPECT_EQ(alpha[0], 255);
  GDALClose(ds);
  std::remove(path.c_str());
}

TEST(CubeExport, EmptySurfaceRefuses)
{
  EXPECT_FALSE(marine_perception_tools::write_cube_geotiff_float(
      CubeSurface{}, tinyAnchor(), tmpPath("never.tif")).empty());
}

}  // namespace
