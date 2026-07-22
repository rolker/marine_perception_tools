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

#ifndef MAP_GEO_ANCHOR_HPP_
#define MAP_GEO_ANCHOR_HPP_

// The map-ENU -> geographic anchor that places a bag's layers on the survey
// map (#24), plus the probe that derives it from a session's mapToGeo. Pure
// (no Qt), so the probe is unit-testable against a synthetic mapping.

#include <cmath>
#include <optional>

namespace marine_perception_tools
{

// First-order (affine) expansion of a bag's mapToGeo about the map origin —
// exact to millimetres over survey scales, and invertible, which mapToGeo
// itself is not.
struct MapGeoAffine
{
  double lat0 = 0.0;      // geo of map (0, 0)
  double lon0 = 0.0;
  double dlat_dx = 0.0;   // degrees per map metre
  double dlat_dy = 0.0;
  double dlon_dx = 0.0;
  double dlon_dy = 0.0;
};

// Derive the anchor by probing `map_to_geo(x, y, lat, lon, alt) -> bool` at
// the origin and one step along each map axis. The step is survey-scale
// (100 m) so the finite differences sit well above double noise while the
// linearisation error stays negligible. nullopt when any probe fails (the bag
// has no earth reference) or returns non-finite coordinates.
template<typename MapToGeoFn>
std::optional<MapGeoAffine> probe_map_anchor(MapToGeoFn && map_to_geo)
{
  constexpr double kStepM = 100.0;
  double alt = 0.0;
  MapGeoAffine a;
  double lat_x = 0.0;
  double lon_x = 0.0;
  double lat_y = 0.0;
  double lon_y = 0.0;
  if (!map_to_geo(0.0, 0.0, a.lat0, a.lon0, alt) ||
    !map_to_geo(kStepM, 0.0, lat_x, lon_x, alt) ||
    !map_to_geo(0.0, kStepM, lat_y, lon_y, alt))
  {
    return std::nullopt;
  }
  a.dlat_dx = (lat_x - a.lat0) / kStepM;
  a.dlon_dx = (lon_x - a.lon0) / kStepM;
  a.dlat_dy = (lat_y - a.lat0) / kStepM;
  a.dlon_dy = (lon_y - a.lon0) / kStepM;
  if (!std::isfinite(a.lat0) || !std::isfinite(a.lon0) ||
    !std::isfinite(a.dlat_dx) || !std::isfinite(a.dlat_dy) ||
    !std::isfinite(a.dlon_dx) || !std::isfinite(a.dlon_dy))
  {
    return std::nullopt;
  }
  return a;
}

}  // namespace marine_perception_tools

#endif  // MAP_GEO_ANCHOR_HPP_
