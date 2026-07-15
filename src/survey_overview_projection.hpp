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

#ifndef SURVEY_OVERVIEW_PROJECTION_HPP_
#define SURVEY_OVERVIEW_PROJECTION_HPP_

#include <algorithm>
#include <cmath>
#include <utility>

namespace marine_perception_tools
{

// The survey-overview view state and its geo↔pixel mapping, kept pure and
// Qt-free so the correctness-critical projection is unit-testable (#19 plan
// decision). North-up equirectangular about the view centre: one degree of
// longitude is drawn cos(center_lat) times as wide as one degree of latitude,
// which is aspect-correct over a survey-sized extent (kilometres, not
// hemispheres). Pixel y grows DOWN (Qt widget coordinates), so north = -y.
struct GeoView
{
  double center_lat = 0.0;      // deg, at the widget centre
  double center_lon = 0.0;      // deg, at the widget centre
  double px_per_deg_lat = 1.0;  // zoom: pixels per degree of latitude
};

// cos(center_lat), floored so a (nonsensical) polar view cannot collapse the
// longitude axis to zero width and make the mapping non-invertible.
inline double lonScale(const GeoView & view)
{
  constexpr double kMinScale = 0.01;
  const double s = std::cos(view.center_lat * M_PI / 180.0);
  return (s > kMinScale) ? s : kMinScale;
}

// Geographic → widget pixel, for a widget of the given size.
inline std::pair<double, double> geoToPixel(
  const GeoView & view, double lat, double lon, double width, double height)
{
  const double x = width / 2.0 +
    (lon - view.center_lon) * view.px_per_deg_lat * lonScale(view);
  const double y = height / 2.0 -
    (lat - view.center_lat) * view.px_per_deg_lat;
  return {x, y};
}

// Widget pixel → geographic; exact inverse of geoToPixel for the same view
// and widget size.
inline std::pair<double, double> pixelToGeo(
  const GeoView & view, double x, double y, double width, double height)
{
  const double lat = view.center_lat -
    (y - height / 2.0) / view.px_per_deg_lat;
  const double lon = view.center_lon +
    (x - width / 2.0) / (view.px_per_deg_lat * lonScale(view));
  return {lat, lon};
}

// View that fits the geographic box [south..north]×[west..east] in a widget
// of the given size with a small margin, centred. Degenerate boxes (a single
// tile row, a point) get a sane minimum span so the zoom stays finite.
inline GeoView fitView(
  double south, double west, double north, double east,
  double width, double height)
{
  constexpr double kMargin = 0.95;   // fraction of the widget used by the box
  constexpr double kMinSpanDeg = 1e-5;
  GeoView view;
  view.center_lat = (south + north) / 2.0;
  view.center_lon = (west + east) / 2.0;
  const double lat_span = std::max(north - south, kMinSpanDeg);
  const double lon_span = std::max(east - west, kMinSpanDeg) * lonScale(view);
  const double fit_lat = kMargin * height / lat_span;
  const double fit_lon = kMargin * width / lon_span;
  view.px_per_deg_lat = std::min(fit_lat, fit_lon);
  return view;
}

}  // namespace marine_perception_tools

#endif  // SURVEY_OVERVIEW_PROJECTION_HPP_
