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

#include <limits>
#include <optional>

#include "hover_geo_readout.hpp"

namespace
{

using marine_perception_tools::GeoPoint;
using marine_perception_tools::HoverGeoReadout;
using marine_perception_tools::HoverPane;
using marine_perception_tools::MapGeoAffine;
using marine_perception_tools::format_hover_geo;
using marine_perception_tools::geo_from_map;
using marine_perception_tools::hover_pane_label;

// A survey-scale anchor: ~1e-5 deg per metre east, ~9e-6 deg per metre north.
MapGeoAffine anchor()
{
  MapGeoAffine a;
  a.lat0 = 43.02;
  a.lon0 = -71.36;
  a.dlat_dx = 0.0;
  a.dlat_dy = 9.0e-6;
  a.dlon_dx = 1.23e-5;
  a.dlon_dy = 0.0;
  return a;
}

TEST(HoverGeoConversion, PlacesAMapPositionThroughTheAnchor)
{
  const auto p = geo_from_map(anchor(), 100.0, 50.0);
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR(p->lat, 43.02 + 50.0 * 9.0e-6, 1e-12);
  EXPECT_NEAR(p->lon, -71.36 + 100.0 * 1.23e-5, 1e-12);
}

// The unplaceable bag: a load with no earth reference has no anchor, and the
// readout must be silent rather than reading the map origin, or (0, 0).
TEST(HoverGeoConversion, ReportsNothingWithoutAnAnchor)
{
  EXPECT_FALSE(geo_from_map(std::nullopt, 100.0, 50.0).has_value());
  EXPECT_FALSE(geo_from_map(std::nullopt, 0.0, 0.0).has_value());
}

TEST(HoverGeoConversion, ReportsNothingForANonFinitePosition)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(geo_from_map(anchor(), nan, 0.0).has_value());
  EXPECT_FALSE(geo_from_map(anchor(), 0.0, inf).has_value());
}

TEST(HoverGeoConversion, ReportsNothingWhenTheAnchorItselfIsNotFinite)
{
  MapGeoAffine a = anchor();
  a.dlat_dy = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(geo_from_map(a, 0.0, 50.0).has_value());
}

TEST(HoverGeoFormat, NamesThePaneAndKeepsSixDecimals)
{
  EXPECT_EQ(
    format_hover_geo(HoverPane::Map, GeoPoint{43.0203045, -71.3600001}),
    "Map  43.020305, -71.360000");
  EXPECT_EQ(
    format_hover_geo(HoverPane::Cloud, GeoPoint{43.0, -71.0}),
    "MBES 3D  43.000000, -71.000000");
}

TEST(HoverGeoFormat, EveryPaneHasAName)
{
  for (const auto pane : {HoverPane::Map, HoverPane::Cloud, HoverPane::Sidescan,
      HoverPane::Mbes, HoverPane::Echogram})
  {
    EXPECT_STRNE(hover_pane_label(pane), "");
  }
}

TEST(HoverGeoReadoutRule, StartsEmpty)
{
  HoverGeoReadout r;
  EXPECT_TRUE(r.text().empty());
  EXPECT_FALSE(r.source().has_value());
}

TEST(HoverGeoReadoutRule, AHoverShowsThePositionAndItsPane)
{
  HoverGeoReadout r;
  r.hover(HoverPane::Sidescan, GeoPoint{43.0, -71.0});
  EXPECT_EQ(r.text(), "Sidescan  43.000000, -71.000000");
  ASSERT_TRUE(r.source().has_value());
  EXPECT_EQ(*r.source(), HoverPane::Sidescan);
}

// The stale-value bug this feature exists to kill: moving onto a pane that
// cannot place the cursor must blank the previous pane's number, not leave it
// standing where it reads as live.
TEST(HoverGeoReadoutRule, AnUnplaceableHoverClearsTheOtherPanesPosition)
{
  HoverGeoReadout r;
  r.hover(HoverPane::Map, GeoPoint{43.0, -71.0});
  r.hover(HoverPane::Cloud, std::nullopt);
  EXPECT_TRUE(r.text().empty());
  ASSERT_TRUE(r.source().has_value());
  EXPECT_EQ(*r.source(), HoverPane::Cloud) << "the cursor is over the cloud";
}

TEST(HoverGeoReadoutRule, LeavingTheOwningPaneClears)
{
  HoverGeoReadout r;
  r.hover(HoverPane::Echogram, GeoPoint{43.0, -71.0});
  r.leave(HoverPane::Echogram);
  EXPECT_TRUE(r.text().empty());
  EXPECT_FALSE(r.source().has_value());
}

// Qt sends the old widget's Leave as the pointer enters the new one, so a
// leave can arrive after the next pane has already reported. It must not
// blank the live position.
TEST(HoverGeoReadoutRule, ALateLeaveFromAnotherPaneLeavesTheLivePositionAlone)
{
  HoverGeoReadout r;
  r.hover(HoverPane::Map, GeoPoint{43.0, -71.0});
  r.hover(HoverPane::Cloud, GeoPoint{43.5, -71.5});
  r.leave(HoverPane::Map);
  EXPECT_EQ(r.text(), "MBES 3D  43.500000, -71.500000");
  ASSERT_TRUE(r.source().has_value());
  EXPECT_EQ(*r.source(), HoverPane::Cloud);
}

TEST(HoverGeoReadoutRule, LeavingAPaneThatCouldNotPlaceTheCursorStillReleasesIt)
{
  HoverGeoReadout r;
  r.hover(HoverPane::Cloud, std::nullopt);
  r.leave(HoverPane::Cloud);
  EXPECT_FALSE(r.source().has_value());
}

}  // namespace
