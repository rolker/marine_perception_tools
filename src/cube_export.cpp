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

#include "cube_export.hpp"

#include <gdal_priv.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace marine_perception_tools
{

namespace
{

struct DatasetCloser
{
  void operator()(GDALDataset * ds) const
  {
    if (ds) {
      GDALClose(ds);
    }
  }
};
using DatasetPtr = std::unique_ptr<GDALDataset, DatasetCloser>;

// Create the GTiff with the full-affine WGS84 geotransform. Rows are
// written NORTH-UP: raster row L holds grid row (ny - 1 - L), so the
// geotransform maps pixel (P, L) centres onto node (P, ny - 1 - L).
DatasetPtr createDataset(
  const CubeSurface & surface, const MapGeoAffine & a, int bands,
  GDALDataType type, const std::string & path, std::string & error)
{
  GDALAllRegister();
  GDALDriver * driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  if (!driver) {
    error = "GDAL GTiff driver unavailable";
    return nullptr;
  }
  char ** options = nullptr;
  options = CSLSetNameValue(options, "COMPRESS", "DEFLATE");
  DatasetPtr ds(driver->Create(
      path.c_str(), surface.nx, surface.ny, bands, type, options));
  CSLDestroy(options);
  if (!ds) {
    error = "could not create " + path;
    return nullptr;
  }

  const double cell = surface.cell_m;
  const double ox = surface.origin_x;
  const double oy_top = surface.origin_y + (surface.ny - 1) * cell;
  double gt[6];
  gt[1] = a.dlon_dx * cell;          // dlon per column
  gt[2] = -a.dlon_dy * cell;         // dlon per (north-up) row
  gt[4] = a.dlat_dx * cell;
  gt[5] = -a.dlat_dy * cell;
  // Pixel (0,0) CORNER (GDAL convention: centre = corner + 0.5 px steps),
  // anchored so pixel centres land exactly on node positions.
  gt[0] = a.lon0 + a.dlon_dx * ox + a.dlon_dy * oy_top - 0.5 * (gt[1] + gt[2]);
  gt[3] = a.lat0 + a.dlat_dx * ox + a.dlat_dy * oy_top - 0.5 * (gt[4] + gt[5]);
  ds->SetGeoTransform(gt);

  OGRSpatialReference srs;
  srs.SetWellKnownGeogCS("WGS84");
  char * wkt = nullptr;
  srs.exportToWkt(&wkt);
  ds->SetProjection(wkt);
  CPLFree(wkt);
  return ds;
}

}  // namespace

std::string write_cube_geotiff_float(
  const CubeSurface & surface, const MapGeoAffine & anchor,
  const std::string & path)
{
  if (!surface.ok()) {
    return "no surface to export";
  }
  std::string error;
  auto ds = createDataset(surface, anchor, 3, GDT_Float32, path, error);
  if (!ds) {
    return error;
  }
  const std::vector<float> * bands[3] = {
    &surface.depth, &surface.uncertainty, &surface.intensity};
  const char * names[3] = {
    "depth_m_positive_up", "uncertainty_m", "cube_settled_backscatter_db"};
  std::vector<float> row(static_cast<std::size_t>(surface.nx));
  for (int b = 0; b < 3; ++b) {
    GDALRasterBand * band = ds->GetRasterBand(b + 1);
    band->SetNoDataValue(std::numeric_limits<double>::quiet_NaN());
    band->SetDescription(names[b]);
    for (int out_row = 0; out_row < surface.ny; ++out_row) {
      const int y = surface.ny - 1 - out_row;   // north-up flip
      const std::size_t base = static_cast<std::size_t>(y) * surface.nx;
      for (int x = 0; x < surface.nx; ++x) {
        row[static_cast<std::size_t>(x)] = (*bands[b])[base + x];
      }
      if (band->RasterIO(
          GF_Write, 0, out_row, surface.nx, 1, row.data(), surface.nx, 1,
          GDT_Float32, 0, 0) != CE_None)
      {
        return std::string("write failed: ") + CPLGetLastErrorMsg();
      }
    }
  }
  return {};
}

std::string write_cube_geotiff_rgba(
  const CubeSurface & surface, const std::vector<std::uint8_t> & rgba,
  const MapGeoAffine & anchor, const std::string & path)
{
  const std::size_t n_nodes =
    static_cast<std::size_t>(surface.nx) * static_cast<std::size_t>(surface.ny);
  if (!surface.ok() || rgba.size() < n_nodes * 4) {
    return "no rendered surface to export";
  }
  std::string error;
  auto ds = createDataset(surface, anchor, 4, GDT_Byte, path, error);
  if (!ds) {
    return error;
  }
  const GDALColorInterp interp[4] = {
    GCI_RedBand, GCI_GreenBand, GCI_BlueBand, GCI_AlphaBand};
  std::vector<std::uint8_t> row(static_cast<std::size_t>(surface.nx));
  for (int b = 0; b < 4; ++b) {
    GDALRasterBand * band = ds->GetRasterBand(b + 1);
    band->SetColorInterpretation(interp[b]);
    for (int out_row = 0; out_row < surface.ny; ++out_row) {
      const int y = surface.ny - 1 - out_row;   // north-up flip
      const std::size_t base = static_cast<std::size_t>(y) * surface.nx;
      for (int x = 0; x < surface.nx; ++x) {
        row[static_cast<std::size_t>(x)] =
          rgba[(base + static_cast<std::size_t>(x)) * 4 + b];
      }
      if (band->RasterIO(
          GF_Write, 0, out_row, surface.nx, 1, row.data(), surface.nx, 1,
          GDT_Byte, 0, 0) != CE_None)
      {
        return std::string("write failed: ") + CPLGetLastErrorMsg();
      }
    }
  }
  return {};
}

}  // namespace marine_perception_tools
