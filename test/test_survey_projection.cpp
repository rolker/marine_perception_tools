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

#include "survey_overview_projection.hpp"

namespace
{

using marine_perception_tools::GeoView;
using marine_perception_tools::fitView;
using marine_perception_tools::geoToPixel;
using marine_perception_tools::lonScale;
using marine_perception_tools::pixelToGeo;

// Massabesic latitude — where the aspect correction actually matters.
constexpr double kLat = 43.02;
constexpr double kLon = -71.36;
constexpr double kW = 800.0;
constexpr double kH = 600.0;

GeoView makeView()
{
  GeoView view;
  view.center_lat = kLat;
  view.center_lon = kLon;
  view.px_per_deg_lat = 100000.0;
  return view;
}

TEST(SurveyProjection, CenterMapsToWidgetCenter)
{
  const auto view = makeView();
  const auto p = geoToPixel(view, kLat, kLon, kW, kH);
  EXPECT_DOUBLE_EQ(p.first, kW / 2.0);
  EXPECT_DOUBLE_EQ(p.second, kH / 2.0);
}

TEST(SurveyProjection, RoundTripIsExact)
{
  const auto view = makeView();
  const auto p = geoToPixel(view, kLat + 0.003, kLon - 0.007, kW, kH);
  const auto geo = pixelToGeo(view, p.first, p.second, kW, kH);
  EXPECT_NEAR(geo.first, kLat + 0.003, 1e-12);
  EXPECT_NEAR(geo.second, kLon - 0.007, 1e-12);
}

TEST(SurveyProjection, NorthUpSouthDown)
{
  // Larger latitude (further north) must land at SMALLER pixel y.
  const auto view = makeView();
  const auto north = geoToPixel(view, kLat + 0.001, kLon, kW, kH);
  const auto south = geoToPixel(view, kLat - 0.001, kLon, kW, kH);
  EXPECT_LT(north.second, south.second);
  EXPECT_DOUBLE_EQ(north.first, south.first);
}

TEST(SurveyProjection, LongitudeCompressedByCosLat)
{
  // At 43°N one degree of longitude must be cos(43°) times as wide as one
  // degree of latitude is tall — the aspect-correctness core of the pane.
  const auto view = makeView();
  const auto lat_step = geoToPixel(view, kLat + 0.01, kLon, kW, kH);
  const auto lon_step = geoToPixel(view, kLat, kLon + 0.01, kW, kH);
  const double dy = std::abs(lat_step.second - kH / 2.0);
  const double dx = std::abs(lon_step.first - kW / 2.0);
  EXPECT_NEAR(dx / dy, std::cos(kLat * M_PI / 180.0), 1e-9);
}

TEST(SurveyProjection, FitViewCentersAndContainsBox)
{
  const double south = kLat - 0.02, north = kLat + 0.02;
  const double west = kLon - 0.05, east = kLon + 0.05;
  const auto view = fitView(south, west, north, east, kW, kH);
  EXPECT_DOUBLE_EQ(view.center_lat, kLat);
  EXPECT_DOUBLE_EQ(view.center_lon, kLon);
  // Every box corner lands inside the widget.
  for (const double lat : {south, north}) {
    for (const double lon : {west, east}) {
      const auto p = geoToPixel(view, lat, lon, kW, kH);
      EXPECT_GE(p.first, 0.0);
      EXPECT_LE(p.first, kW);
      EXPECT_GE(p.second, 0.0);
      EXPECT_LE(p.second, kH);
    }
  }
}

TEST(SurveyProjection, FitViewDegenerateBoxStaysFinite)
{
  const auto view = fitView(kLat, kLon, kLat, kLon, kW, kH);
  EXPECT_TRUE(std::isfinite(view.px_per_deg_lat));
  EXPECT_GT(view.px_per_deg_lat, 0.0);
}

TEST(SurveyProjection, LonScaleFlooredNearPoles)
{
  GeoView view;
  view.center_lat = 89.9999;
  EXPECT_GE(lonScale(view), 0.01);
}

}  // namespace
