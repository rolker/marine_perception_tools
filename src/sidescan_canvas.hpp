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

#ifndef SIDESCAN_CANVAS_HPP_
#define SIDESCAN_CANVAS_HPP_

#include <QImage>
#include <QPoint>
#include <QPointF>
#include <QPolygonF>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QWidget>

#include <cstddef>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "map_geo_anchor.hpp"
#include "tile_selection.hpp"

namespace marine_perception_tools
{

// A contact to draw on the map: centre + box extent (map metres) + label.
struct ContactMarker
{
  double x = 0.0;
  double y = 0.0;
  double w = 0.0;
  double h = 0.0;
  QString id;
};

// One store tile rendered on the map: a colormapped image plus its geographic
// bounds (GGGS grid corners, degrees). (Moved here from the retired
// SurveyOverviewCanvas, #24.)
struct OverviewTile
{
  QImage image;
  double south = 0.0;
  double west = 0.0;
  double north = 0.0;
  double east = 0.0;
};

// A geographic rect for a selectable index tile (degrees).
struct GeoRect
{
  double south = 0.0;
  double west = 0.0;
  double north = 0.0;
  double east = 0.0;
};

// (MapGeoAffine — the map-ENU -> geographic anchor for a bag with an earth
// reference — lives in map_geo_anchor.hpp with the probe that derives it.)

// North-up 2D map canvas (#24: the explorer's index map). Internally it works
// in "canvas metres":
//  - Without a geographic origin (bag-only, no survey index) canvas metres ARE
//    the bag's map-ENU metres — the historical per-bag behaviour, unchanged.
//  - With a geographic origin (survey index loaded) canvas metres are a local
//    equirectangular plane about that origin; geographic layers (store-tile
//    basemap, nav track, selectable index tiles) place directly, and the
//    per-bag layers (coverage image, boat track, contacts, linked cursor)
//    place through the bag's MapGeoAffine anchor. A bag without a geo anchor
//    cannot be placed on the survey map: its layers are hidden and the
//    map-frame signals are suppressed rather than emitted with garbage.
// Wheel zoom (about the cursor) and drag pan; ctrl-click toggles the index
// tile under the cursor and ctrl-drag rubber-bands tiles.
class SidescanCanvas : public QWidget
{
  Q_OBJECT

public:
  explicit SidescanCanvas(QWidget * parent = nullptr);

  // --- geographic frame (survey/index mode) -------------------------------
  // Establish the geographic origin of the canvas-metre plane. Set BEFORE any
  // geographic layer; changing it re-derives all stored layer geometry.
  void setGeoOrigin(double lat_deg, double lon_deg);
  bool hasGeoOrigin() const {return geo_mode_;}

  // The bag's map-ENU -> geo anchor (nullopt: bag has no earth reference).
  void setMapAnchor(const std::optional<MapGeoAffine> & anchor);
  bool mapPlaceable() const {return !geo_mode_ || map_anchor_.has_value();}

  // Store-tile basemap (geo bounds per tile).
  void setStoreTiles(std::vector<OverviewTile> tiles);

  // Decimated nav track, pre-segmented per bag, as (lat, lon) polylines.
  void setNavTrack(std::vector<std::vector<std::pair<double, double>>> segments);

  // Overlay visibility (#24 desk-verify follow-up): a whole campaign's track
  // and tile grid blanket the surveyed area at overview zoom, hiding the
  // basemap under them — let the operator switch them off. Hiding the tile
  // grid keeps SELECTED tiles visible (the selection state must stay
  // answerable at a glance).
  void setNavTrackVisible(bool on);
  void setIndexTilesVisible(bool on);

  // Selectable index tiles. Replaces the set and clears the selection.
  void setIndexTiles(const std::vector<GeoRect> & tiles);
  const std::set<std::size_t> & selectedTiles() const {return selected_tiles_;}
  void clearTileSelection();

  // Programmatic selection (same contract as ctrl-click): indices into the
  // setIndexTiles order. Out-of-range indices are dropped; emits
  // tileSelectionChanged only when the selection actually changes.
  void selectTiles(const std::set<std::size_t> & indices);

  // Fit the view to a geographic box (applied at the next laid-out paint, and
  // re-applied on resize until the user pans/zooms — the ctor-time widget
  // size is pre-layout, so an immediate fit would mis-scale).
  void fitGeo(double south, double west, double north, double east);
  // Whether the user has taken the view over (pan/zoom/explicit bag fit) —
  // callers use this to avoid re-fitting out from under the operator when a
  // better fit box becomes available (e.g. the async basemap arriving).
  bool viewAdjustedByUser() const {return user_adjusted_;}

  // --- per-bag layers (map-ENU metres, unchanged contracts) ---------------
  // Set the rendered coverage image and the map extent it covers. `origin_x/y` is
  // the south-west corner (min east, min north); the image is north-up (row 0 =
  // north edge). `res_m` is metres per pixel.
  void setCoverage(const QImage & image, double origin_x, double origin_y, double res_m);

  // Boat track as a polyline in map metres.
  void setTrack(const std::vector<QPointF> & track_map);

  void setGridSpacing(double metres);

  // Recenter the view on a map point (metres) without changing zoom — used to keep
  // the scrubbed window centred ("follow the playhead"). No-op when the bag is
  // not placeable on the survey map.
  void setCenter(double map_x, double map_y);

  // Fit the current bag content (coverage or track); with a geographic fit
  // pending and no bag content, refits the survey bounds instead.
  void resetView();

  // In mark mode, left-drag draws a contact box (instead of panning) and emits
  // boxMarked() on release.
  void setMarkMode(bool on);

  // Contacts to overlay (map metres). Drawn wherever they fall in the current view.
  void setContacts(const QVector<ContactMarker> & contacts);

  // Transient cross-pane linked cursor at a map point (metres); nullopt clears it.
  void setCursorWorld(const std::optional<QPointF> & map_point);

signals:
  // A contact box was drawn, in map coordinates (metres).
  void boxMarked(const QRectF & map_rect);

  // Hovered map position (metres), for a cross-pane linked cursor (always valid).
  void hoverWorld(double map_x, double map_y, bool valid);

  // Hovered geographic position (only in geo mode), for a status readout.
  void hoverGeo(double lat, double lon);

  // Middle-click map position (metres), for click-to-seek.
  void seekWorld(double map_x, double map_y);

  // The set of selected index tiles changed (ctrl-click / rubber band).
  void tileSelectionChanged();

protected:
  void paintEvent(QPaintEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;

private:
  // canvas metres <-> screen pixels (view transform).
  QPointF mapToScreen(double mx, double my) const;
  QPointF screenToMap(double sx, double sy) const;
  // geographic <-> canvas metres (local equirectangular about the geo origin).
  QPointF geoToCanvas(double lat, double lon) const;
  std::pair<double, double> canvasToGeo(double mx, double my) const;
  // bag map-ENU <-> canvas metres (through the anchor; identity without geo mode).
  QPointF bagToCanvas(double x, double y) const;
  std::optional<QPointF> canvasToBag(double mx, double my) const;

  void rebuildGeoLayerGeometry();   // re-derive canvas-metre rects/polylines
  void applyPendingFit();
  void drawGrid(QPainter & painter) const;
  void drawNavTrack(QPainter & painter) const;
  void drawIndexTiles(QPainter & painter) const;

  // --- geographic frame ---
  bool geo_mode_ = false;
  double geo_lat0_ = 0.0;
  double geo_lon0_ = 0.0;
  double lon_scale_ = 1.0;   // cos(geo_lat0_), floored
  std::optional<MapGeoAffine> map_anchor_;

  std::vector<OverviewTile> store_tiles_;
  std::vector<QRectF> store_tile_rects_;   // canvas metres (x0..x1, y0..y1 via QRectF)

  std::vector<std::vector<std::pair<double, double>>> nav_segments_geo_;
  std::vector<QPolygonF> nav_segments_;    // canvas metres

  std::vector<GeoRect> index_tiles_geo_;
  std::vector<SelectableRect> index_tiles_;   // canvas metres
  std::set<std::size_t> selected_tiles_;
  bool show_nav_track_ = true;
  bool show_index_tiles_ = true;

  bool fit_pending_ = false;
  bool user_adjusted_ = false;
  double fit_south_ = 0.0, fit_west_ = 0.0, fit_north_ = 0.0, fit_east_ = 0.0;

  bool band_selecting_ = false;   // mid ctrl-drag rubber band
  QPoint band_start_;
  QPoint band_cur_;

  // --- per-bag layers ---
  QImage coverage_;
  double cov_origin_x_ = 0.0;   // SW corner east (m, map frame)
  double cov_origin_y_ = 0.0;   // SW corner north (m, map frame)
  double cov_res_m_ = 1.0;      // metres per coverage pixel
  bool have_coverage_ = false;

  std::vector<QPointF> track_;  // map frame

  double grid_spacing_m_ = 10.0;
  double px_per_m_ = 4.0;       // zoom
  QPointF center_map_{0.0, 0.0};  // canvas point shown at the widget centre

  QPoint last_drag_pos_;

  bool mark_mode_ = false;
  bool marking_ = false;        // mid contact box-drag
  QPoint mark_start_;           // screen px
  QPoint mark_cur_;             // screen px
  QVector<ContactMarker> contacts_;              // map frame
  std::optional<QPointF> cursor_world_;   // cross-pane linked cursor (map metres)
};

}  // namespace marine_perception_tools

#endif  // SIDESCAN_CANVAS_HPP_
