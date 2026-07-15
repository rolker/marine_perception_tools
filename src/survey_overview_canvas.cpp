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

#include "survey_overview_canvas.hpp"

#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QWheelEvent>

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

namespace marine_perception_tools
{

SurveyOverviewCanvas::SurveyOverviewCanvas(QWidget * parent)
: QWidget(parent)
{
  setMouseTracking(true);
  setMinimumSize(320, 240);
}

void SurveyOverviewCanvas::setTiles(std::vector<OverviewTile> tiles)
{
  tiles_ = std::move(tiles);
  have_mark_ = false;
  have_fit_ = false;
  user_adjusted_ = false;
  // Defer the fit to the first paint/resize: at ctor time (setTiles runs from
  // the window ctor) the widget still has its pre-layout size, so fitting now
  // would mis-scale the basemap. paintEvent refits lazily when !have_fit_.
  update();
}

void SurveyOverviewCanvas::setFallbackBounds(
  double south, double west, double north, double east)
{
  have_fallback_ = true;
  fallback_south_ = south;
  fallback_west_ = west;
  fallback_north_ = north;
  fallback_east_ = east;
  if (!user_adjusted_) {
    have_fit_ = false;   // let the next paint refit against the new bounds
  }
  update();
}

void SurveyOverviewCanvas::setQueryMark(double lat, double lon)
{
  have_mark_ = true;
  mark_lat_ = lat;
  mark_lon_ = lon;
  update();
}

void SurveyOverviewCanvas::fitToTiles()
{
  if (tiles_.empty()) {
    if (have_fallback_) {
      view_ = fitView(
        fallback_south_, fallback_west_, fallback_north_, fallback_east_,
        width(), height());
      have_fit_ = true;
    }
    return;   // no tiles and no fallback: no geo frame — stay unfit
  }
  double south = std::numeric_limits<double>::max();
  double west = std::numeric_limits<double>::max();
  double north = std::numeric_limits<double>::lowest();
  double east = std::numeric_limits<double>::lowest();
  for (const auto & tile : tiles_) {
    south = std::min(south, tile.south);
    west = std::min(west, tile.west);
    north = std::max(north, tile.north);
    east = std::max(east, tile.east);
  }
  view_ = fitView(south, west, north, east, width(), height());
  have_fit_ = true;
}

void SurveyOverviewCanvas::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);
  QPainter painter(this);
  painter.fillRect(rect(), QColor(24, 26, 30));
  if (!have_fit_) {
    fitToTiles();
  }
  if (tiles_.empty()) {
    painter.setPen(Qt::gray);
    painter.drawText(
      rect(), Qt::AlignCenter,
      have_fit_ ?
      "No store tiles loaded — click anywhere to query passes" :
      "No store tiles loaded");
    if (!have_fit_) {
      return;   // no geo frame: nothing further can be drawn meaningfully
    }
  }
  painter.setRenderHint(QPainter::SmoothPixmapTransform);
  for (const auto & tile : tiles_) {
    const auto nw = geoToPixel(view_, tile.north, tile.west, width(), height());
    const auto se = geoToPixel(view_, tile.south, tile.east, width(), height());
    const QRectF target(QPointF(nw.first, nw.second), QPointF(se.first, se.second));
    if (!target.intersects(rect())) {
      continue;
    }
    painter.drawImage(target, tile.image);
  }
  if (have_mark_) {
    const auto p = geoToPixel(view_, mark_lat_, mark_lon_, width(), height());
    painter.setPen(QPen(QColor(0, 255, 255), 2));
    painter.drawLine(QPointF(p.first - 8, p.second), QPointF(p.first + 8, p.second));
    painter.drawLine(QPointF(p.first, p.second - 8), QPointF(p.first, p.second + 8));
  }
}

void SurveyOverviewCanvas::resizeEvent(QResizeEvent * event)
{
  QWidget::resizeEvent(event);
  // Refit to the tile bounds whenever the widget is (re)sized, until the user
  // takes control of the view. This is what makes the initial fit correct: the
  // real laid-out size only exists after the first resize/show.
  if (!user_adjusted_) {
    have_fit_ = false;
    update();
  }
}

void SurveyOverviewCanvas::wheelEvent(QWheelEvent * event)
{
  if (!have_fit_) {
    return;   // no geo frame yet: zooming a default view would lock in garbage
  }
  // Zoom about the cursor: keep the geographic point under it fixed.
  user_adjusted_ = true;
  const double factor = (event->angleDelta().y() > 0) ? 1.25 : (1.0 / 1.25);
  const auto pos = event->position();
  const auto anchor = pixelToGeo(view_, pos.x(), pos.y(), width(), height());
  view_.px_per_deg_lat = std::clamp(view_.px_per_deg_lat * factor, 1.0, 1e9);
  const auto moved = pixelToGeo(view_, pos.x(), pos.y(), width(), height());
  view_.center_lat += anchor.first - moved.first;
  view_.center_lon += anchor.second - moved.second;
  update();
}

void SurveyOverviewCanvas::mousePressEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    panning_ = true;
    moved_since_press_ = false;
    last_mouse_ = event->pos();
    press_pos_ = event->pos();
  }
}

void SurveyOverviewCanvas::mouseMoveEvent(QMouseEvent * event)
{
  if (!have_fit_) {
    return;   // no geo frame yet: hover/pan through the default view is noise
  }
  const auto geo = pixelToGeo(view_, event->pos().x(), event->pos().y(), width(), height());
  Q_EMIT hoverGeo(geo.first, geo.second);
  if (!panning_) {
    return;
  }
  const QPoint delta = event->pos() - last_mouse_;
  if ((event->pos() - press_pos_).manhattanLength() > 4) {
    moved_since_press_ = true;
  }
  last_mouse_ = event->pos();
  user_adjusted_ = true;
  view_.center_lat += delta.y() / view_.px_per_deg_lat;
  view_.center_lon -= delta.x() / (view_.px_per_deg_lat * lonScale(view_));
  update();
}

void SurveyOverviewCanvas::mouseReleaseEvent(QMouseEvent * event)
{
  if (event->button() != Qt::LeftButton) {
    return;
  }
  panning_ = false;
  if (!moved_since_press_ && have_fit_) {   // a click, not a pan
    const auto geo = pixelToGeo(view_, event->pos().x(), event->pos().y(), width(), height());
    Q_EMIT clicked(geo.first, geo.second);
  }
}

}  // namespace marine_perception_tools
