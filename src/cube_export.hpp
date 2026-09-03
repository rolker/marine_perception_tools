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

#ifndef CUBE_EXPORT_HPP_
#define CUBE_EXPORT_HPP_

// GeoTIFF export of the box-CUBE surface (#27): either the data itself
// (Float32 bands depth / uncertainty / CUBE-settled backscatter, NaN
// NoData) or the rendered view (RGBA with the active colormap applied,
// alpha 0 on holes). Georeferencing comes from the reference frame's
// earth anchor as a full affine geotransform on WGS84 (EPSG:4326) —
// rows are written north-up. Qt-free; GDAL only.

#include <cstdint>
#include <string>
#include <vector>

#include "cube_lab.hpp"
#include "map_geo_anchor.hpp"

namespace marine_perception_tools
{

// Float32 data bands: 1 = depth (m, +up), 2 = uncertainty (m),
// 3 = CUBE-settled backscatter (dB). NaN = no estimate.
// Returns an empty string on success, else the error.
std::string write_cube_geotiff_float(
  const CubeSurface & surface, const MapGeoAffine & anchor,
  const std::string & path);

// RGBA render: `rgba` is 4 bytes per node (row-major like the surface),
// alpha 0 = hole. Returns an empty string on success, else the error.
std::string write_cube_geotiff_rgba(
  const CubeSurface & surface, const std::vector<std::uint8_t> & rgba,
  const MapGeoAffine & anchor, const std::string & path);

}  // namespace marine_perception_tools

#endif  // CUBE_EXPORT_HPP_
