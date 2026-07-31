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
#include <limits>

#include "map_geo_anchor.hpp"

namespace
{

using marine_perception_tools::probe_map_anchor;

// A synthetic exactly-affine mapToGeo: the probe must recover its
// coefficients to double precision (finite differences of a linear map are
// exact up to rounding).
constexpr double kLat0 = 43.02;
constexpr double kLon0 = -71.36;
constexpr double kDLatDx = 1.1e-7;
constexpr double kDLatDy = 9.0e-6;
constexpr double kDLonDx = 1.23e-5;
constexpr double kDLonDy = -2.2e-7;

bool affine_map_to_geo(double x, double y, double & lat, double & lon, double & alt)
{
  lat = kLat0 + kDLatDx * x + kDLatDy * y;
  lon = kLon0 + kDLonDx * x + kDLonDy * y;
  alt = 0.0;
  return true;
}

TEST(MapGeoAnchor, RecoversAnAffineMappingExactly)
{
  const auto anchor = probe_map_anchor(affine_map_to_geo);
  ASSERT_TRUE(anchor.has_value());
  EXPECT_DOUBLE_EQ(anchor->lat0, kLat0);
  EXPECT_DOUBLE_EQ(anchor->lon0, kLon0);
  // The finite-difference step is 100 m; a linear map's differences are exact
  // to rounding in the degree values (~1e-16 of a degree).
  EXPECT_NEAR(anchor->dlat_dx, kDLatDx, 1e-15);
  EXPECT_NEAR(anchor->dlat_dy, kDLatDy, 1e-15);
  EXPECT_NEAR(anchor->dlon_dx, kDLonDx, 1e-15);
  EXPECT_NEAR(anchor->dlon_dy, kDLonDy, 1e-15);
}

TEST(MapGeoAnchor, ForwardEvaluationMatchesTheSourceMapping)
{
  const auto anchor = probe_map_anchor(affine_map_to_geo);
  ASSERT_TRUE(anchor.has_value());
  // A survey-scale point well away from the probe locations.
  const double x = -732.5;
  const double y = 418.0;
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
  ASSERT_TRUE(affine_map_to_geo(x, y, lat, lon, alt));
  const double lat_a = anchor->lat0 + anchor->dlat_dx * x + anchor->dlat_dy * y;
  const double lon_a = anchor->lon0 + anchor->dlon_dx * x + anchor->dlon_dy * y;
  EXPECT_NEAR(lat_a, lat, 1e-12);
  EXPECT_NEAR(lon_a, lon, 1e-12);
}

TEST(MapGeoAnchor, FailingMappingYieldsNoAnchor)
{
  const auto anchor = probe_map_anchor(
    [](double, double, double &, double &, double &) {return false;});
  EXPECT_FALSE(anchor.has_value());
}

TEST(MapGeoAnchor, MappingFailingAwayFromTheOriginYieldsNoAnchor)
{
  // Succeeds at the origin, fails at the axis probes: the anchor must not be
  // built from a partial probe.
  const auto anchor = probe_map_anchor(
    [](double x, double y, double & lat, double & lon, double & alt) {
      if (x != 0.0 || y != 0.0) {
        return false;
      }
      return affine_map_to_geo(x, y, lat, lon, alt);
    });
  EXPECT_FALSE(anchor.has_value());
}

TEST(MapGeoAnchor, NonFiniteCoordinatesYieldNoAnchor)
{
  const auto anchor = probe_map_anchor(
    [](double, double, double & lat, double & lon, double & alt) {
      lat = std::numeric_limits<double>::quiet_NaN();
      lon = 0.0;
      alt = 0.0;
      return true;
    });
  EXPECT_FALSE(anchor.has_value());
}

}  // namespace
