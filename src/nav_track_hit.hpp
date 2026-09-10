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

#ifndef NAV_TRACK_HIT_HPP_
#define NAV_TRACK_HIT_HPP_

// Position -> time over the campaign nav track (#46): the inverse of
// nav_track_lookup.hpp's fixAtTime. Given the survey index's decimated
// nav_track rows and where the cursor is on the map, which fix is the
// operator pointing at, and when was the boat there?
//
// The radius is in SCREEN PIXELS, not ground metres, and that is the whole
// point of this unit. "Close enough to the trackline" is a statement about
// the operator's hand and the size of the marker under it, so it has to mean
// the same thing at every zoom: a ground radius that felt right over one
// survey line would swallow a whole day's passes zoomed out to the campaign,
// and would be unreachable zoomed in on a single ping. The caller supplies
// the view's ground metres per pixel and the search converts each candidate's
// ground separation into pixels before comparing.
//
// Linear over the track (46.5k campaign points is well under a millisecond)
// and pure, so it runs at mouse-move rate on the UI thread and is tested
// without a display.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "marine_survey_index/query.hpp"

namespace marine_perception_tools
{

/// Screen pixels within which the cursor counts as being on a nav-track fix.
/// Wide enough to catch the line without hunting for it, narrow enough that
/// two passes a boat-width apart are still separately reachable at survey
/// zoom. Inclusive at the boundary, like the click-versus-drag slop.
constexpr double kFixHitRadiusPx = 8.0;

/// Metres per degree of latitude — the same figure the canvas's
/// equirectangular plane uses, so a pixel here is the pixel drawn there.
constexpr double kHitMetresPerDegLat = 111320.0;

/// The nav-track fix under the cursor, and how far off it the cursor was.
struct TrackHit
{
  std::size_t index = 0;          // into the track passed in
  std::int64_t t_ns = 0;          // when the boat was here
  std::int64_t bag_id = 0;        // which recording it came from
  double latitude = 0.0;
  double longitude = 0.0;
  double distance_px = 0.0;       // cursor -> fix, on screen
};

/// The nearest fix within `radius_px` of (`lat`, `lon`), or nullopt when the
/// track is empty, the view scale is unusable, or nothing is close enough.
///
/// `metres_per_pixel` is the view's true ground scale (SidescanCanvas::
/// groundMetresPerPixel). Ties go to the earlier fix, so a cursor exactly
/// between two passes resolves the same way every frame instead of flickering
/// between them.
inline std::optional<TrackHit> nearestTrackFix(
  const std::vector<marine_survey_index::NavPoint> & track,
  double lat, double lon, double metres_per_pixel,
  double radius_px = kFixHitRadiusPx)
{
  if (track.empty() || !(metres_per_pixel > 0.0) ||
    !std::isfinite(metres_per_pixel) || !std::isfinite(lat) ||
    !std::isfinite(lon) || !(radius_px >= 0.0))
  {
    return std::nullopt;
  }
  // Local flat-earth scaling about the cursor: over the few hundred metres a
  // hit radius can span, the error in cos(lat) is far below a pixel.
  const double lon_scale = std::cos(lat * M_PI / 180.0);
  std::optional<TrackHit> best;
  for (std::size_t i = 0; i < track.size(); ++i) {
    const auto & p = track[i];
    // A non-finite fix is not a candidate, and must be skipped BEFORE it can
    // become `best`: its `px` is NaN, `px > radius_px` is false so the radius
    // gate does not reject it, and once it is `best` the `px < best` test is
    // false for every real candidate afterwards — so one bad row would swallow
    // the whole track and return itself. The index writer filters these, so
    // this guards a database written by some other path (#42 review, and
    // independently flagged by Copilot).
    if (!std::isfinite(p.latitude) || !std::isfinite(p.longitude)) {
      continue;
    }
    const double north_m = (p.latitude - lat) * kHitMetresPerDegLat;
    const double east_m = (p.longitude - lon) * kHitMetresPerDegLat * lon_scale;
    const double px = std::hypot(north_m, east_m) / metres_per_pixel;
    if (px > radius_px) {
      continue;
    }
    // Strictly nearer, so the first of equally-near fixes keeps the hit.
    if (best && !(px < best->distance_px)) {
      continue;
    }
    best = TrackHit{i, p.t_ns, p.bag_id, p.latitude, p.longitude, px};
  }
  return best;
}

}  // namespace marine_perception_tools

#endif  // NAV_TRACK_HIT_HPP_
