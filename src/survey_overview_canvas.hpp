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

#ifndef SURVEY_OVERVIEW_CANVAS_HPP_
#define SURVEY_OVERVIEW_CANVAS_HPP_

#include <QImage>
#include <QPoint>
#include <QWidget>

#include <vector>

#include "survey_overview_projection.hpp"

namespace marine_perception_tools
{

// One store tile rendered on the overview: a colormapped image plus its
// geographic bounds (GGGS grid corners, degrees).
struct OverviewTile
{
  QImage image;
  double south = 0.0;
  double west = 0.0;
  double north = 0.0;
  double east = 0.0;
};

// Survey-wide, north-up map canvas: draws store tiles in lat/lon space via the
// pure GeoView projection (survey_overview_projection.hpp — cos(lat) aspect
// correction), with wheel-zoom about the cursor, drag-pan, and a clicked(lat,
// lon) signal for the pass query. Sibling of SidescanCanvas, which stays in
// its per-bag map-ENU frame; this one spans the whole survey in geographic
// coordinates.
class SurveyOverviewCanvas : public QWidget
{
  Q_OBJECT

public:
  explicit SurveyOverviewCanvas(QWidget * parent = nullptr);

  // Replaces the tile set and refits the view to its bounds.
  void setTiles(std::vector<OverviewTile> tiles);

  // Marks the last-queried point (drawn as a crosshair); invalidated by setTiles.
  void setQueryMark(double lat, double lon);

Q_SIGNALS:
  // A left click that wasn't a drag, in geographic coordinates.
  void clicked(double lat, double lon);
  // Cursor geographic position, for a status readout.
  void hoverGeo(double lat, double lon);

protected:
  void paintEvent(QPaintEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;

private:
  void fitToTiles();

  std::vector<OverviewTile> tiles_;
  GeoView view_;
  bool have_fit_ = false;
  // Whether the user has zoomed/panned. Until they do, a resize refits to the
  // tile bounds — the ctor-time widget size is pre-layout, so the first real
  // fit must wait for a resize/first paint at the laid-out size.
  bool user_adjusted_ = false;

  bool panning_ = false;
  bool moved_since_press_ = false;
  QPoint last_mouse_;
  QPoint press_pos_;

  bool have_mark_ = false;
  double mark_lat_ = 0.0;
  double mark_lon_ = 0.0;
};

}  // namespace marine_perception_tools

#endif  // SURVEY_OVERVIEW_CANVAS_HPP_
