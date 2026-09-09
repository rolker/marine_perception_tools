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

#ifndef HOVER_GEO_READOUT_HPP_
#define HOVER_GEO_READOUT_HPP_

// The geographic cursor readout's rules (#47), Qt-free so they unit-test
// without a display:
//
//  - converting a pane's own map-ENU position to geographic through the
//    frame's anchor, answering NOTHING when the frame has no earth reference
//    (an unplaceable bag) rather than a plausible-looking zero;
//  - the one-line format, which names the pane the position came from,
//    because four panes feed one label and an unattributed number is a
//    position the operator cannot act on;
//  - which pane currently owns the readout, so a leave from the pane the
//    cursor has already left cannot wipe the pane it moved to.

#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

#include "map_geo_anchor.hpp"

namespace marine_perception_tools
{

// The spatial panes that can say where the cursor is. Each converts in its
// own frame; the readout only ever sees geographic degrees.
enum class HoverPane
{
  Map,        // the index map (canvas metres, or the bag's map-ENU)
  Cloud,      // the MBES 3D view (its load's reference world frame)
  Sidescan,   // the sidescan waterfall (open bag's map-ENU)
  Mbes,       // the MBES backscatter waterfall (open bag's map-ENU)
  Echogram,   // the water-column echogram (via along-track distance)
};

// The pane's name as the readout shows it: the pane headers' own words, so
// the operator matches the tag to a pane by reading, not by learning a code.
inline const char * hover_pane_label(HoverPane pane)
{
  switch (pane) {
    case HoverPane::Map: return "Map";
    case HoverPane::Cloud: return "MBES 3D";
    case HoverPane::Sidescan: return "Sidescan";
    case HoverPane::Mbes: return "MBES Backscatter";
    case HoverPane::Echogram: return "Water Column";
  }
  return "";
}

struct GeoPoint
{
  double lat = 0.0;
  double lon = 0.0;
};

// A pane's map-ENU metres -> geographic, through the affine anchor of the
// frame those metres live in. nullopt when there is no anchor (the frame has
// no earth reference) or the position itself is not finite: the readout must
// say nothing rather than place the cursor somewhere it is not.
inline std::optional<GeoPoint> geo_from_map(
  const std::optional<MapGeoAffine> & anchor, double x, double y)
{
  if (!anchor || !std::isfinite(x) || !std::isfinite(y)) {
    return std::nullopt;
  }
  const auto & a = *anchor;
  const GeoPoint p{
    a.lat0 + a.dlat_dx * x + a.dlat_dy * y,
    a.lon0 + a.dlon_dx * x + a.dlon_dy * y};
  if (!std::isfinite(p.lat) || !std::isfinite(p.lon)) {
    return std::nullopt;
  }
  return p;
}

// "Map  43.123456, -70.123456" — the six-decimal degrees the map readout has
// always shown (~0.1 m, finer than any position here is good to), with the
// source pane in front of them.
inline std::string format_hover_geo(HoverPane pane, const GeoPoint & p)
{
  char buf[96];
  std::snprintf(
    buf, sizeof(buf), "%s  %.6f, %.6f", hover_pane_label(pane), p.lat, p.lon);
  return std::string(buf);
}

// Which pane owns the readout, and what it reads.
//
// Every hover — resolvable or not — hands ownership to the pane the cursor is
// over, because the cursor being there is proof that any other pane's value is
// stale. A leave only clears if that pane still owns the readout: Qt sends the
// old widget's Leave as the pointer enters the new one, and a leave that
// arrived late must not blank a live position from the pane it moved to.
class HoverGeoReadout
{
public:
  void hover(HoverPane pane, const std::optional<GeoPoint> & pos)
  {
    source_ = pane;
    text_ = pos ? format_hover_geo(pane, *pos) : std::string();
  }

  void leave(HoverPane pane)
  {
    if (source_ && *source_ == pane) {
      source_.reset();
      text_.clear();
    }
  }

  // Empty whenever there is no live position to show.
  const std::string & text() const {return text_;}
  const std::optional<HoverPane> & source() const {return source_;}

private:
  std::optional<HoverPane> source_;
  std::string text_;
};

}  // namespace marine_perception_tools

#endif  // HOVER_GEO_READOUT_HPP_
