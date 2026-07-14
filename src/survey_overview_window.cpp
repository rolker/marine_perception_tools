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

#include "survey_overview_window.hpp"

#include <QDateTime>
#include <QHeaderView>
#include <QSplitter>
#include <QStatusBar>
#include <QString>
#include <QStringList>
#include <QTimeZone>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "marine_autonomy/gggs.h"
#include "marine_colormap/colormap.hpp"
#include "marine_colormap/palette.hpp"
#include "marine_colormap/transfer.hpp"
#include "marine_tiled_raster_store/tile_io.hpp"
#include "sidescan_viewer_window.hpp"

namespace marine_perception_tools
{

namespace
{

QString isoUtc(std::int64_t t_ns)
{
  return QDateTime::fromMSecsSinceEpoch(t_ns / 1000000LL, QTimeZone::utc())
         .toString("yyyy-MM-dd HH:mm:ss");
}

// Colormap one store tile's band 0 (depth) into an RGBA image: finite values
// span the given range, NaN (store NoData) stays transparent so gaps read as
// gaps instead of painting as the deepest colour.
QImage tileToImage(
  const marine_tiled_raster_store::TiledRasterTile<double> & tile,
  double lo, double hi, const std::vector<marine_colormap::Rgba8> & lut)
{
  const auto & band = tile.band(0);
  const int rows = marine_tiled_raster_store::TiledRasterTile<double>::edge;
  const int cols = marine_tiled_raster_store::TiledRasterTile<double>::edge;
  QImage image(cols, rows, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  const double span = (hi > lo) ? (hi - lo) : 1.0;
  for (int r = 0; r < rows; ++r) {
    // Store row 0 is the SOUTH edge (GGGS rows grow northward); QImage row 0
    // is drawn at the TOP (north) of the target rect, so flip vertically.
    QRgb * out = reinterpret_cast<QRgb *>(image.scanLine(rows - 1 - r));
    for (int c = 0; c < cols; ++c) {
      const double v = band[static_cast<std::size_t>(r) * cols + c];
      if (std::isnan(v)) {
        continue;
      }
      const double t = std::clamp((v - lo) / span, 0.0, 1.0);
      const auto & rgba =
        lut[static_cast<std::size_t>(t * static_cast<double>(lut.size() - 1) + 0.5)];
      out[c] = qRgba(rgba.r, rgba.g, rgba.b, 255);
    }
  }
  return image;
}

// One physical pass over the queried spot. queryPasses returns one PassRow per
// (pass, tile), so a transit that crosses several tiles inside the query box
// comes back as several per-tile segments.
struct CoalescedPass
{
  std::string bag_path;
  std::string sensor_type;
  std::string topic;
  std::int64_t t_start_ns = 0;
  std::int64_t t_end_ns = 0;
  std::int64_t ping_count = 0;
};

// Merge the per-tile segments so the pass list shows one row per physical pass.
// Segments sharing (bag, sensor, topic) whose time windows overlap or sit
// within kSegmentGapNs are one transit across adjacent tiles (their windows are
// back-to-back); a genuine revisit of the same spot is separated by far more,
// so it stays a distinct entry. Ping counts of merged segments are summed.
// Result is ordered by bag then start, matching the queryPasses contract the
// list build relies on.
std::vector<CoalescedPass> coalescePasses(
  const std::vector<marine_survey_index::PassRow> & rows)
{
  // 5 s comfortably spans the inter-tile ping gap within one transit without
  // bridging two separate visits (survey revisits are minutes apart).
  constexpr std::int64_t kSegmentGapNs = 5LL * 1000000000LL;

  // rows arrive ordered by bag then t_start; bucket by (bag, sensor, topic) —
  // filtering that order per bucket keeps each bucket sorted by t_start, so a
  // running interval-merge against the last segment is correct.
  std::map<std::tuple<std::string, std::string, std::string>,
    std::vector<CoalescedPass>> buckets;
  for (const auto & row : rows) {
    auto & merged = buckets[{row.bag_path, row.sensor_type, row.topic}];
    if (!merged.empty() && row.t_start_ns <= merged.back().t_end_ns + kSegmentGapNs) {
      merged.back().t_end_ns = std::max(merged.back().t_end_ns, row.t_end_ns);
      merged.back().ping_count += row.ping_count;
    } else {
      merged.push_back(CoalescedPass{
            row.bag_path, row.sensor_type, row.topic,
            row.t_start_ns, row.t_end_ns, row.ping_count});
    }
  }

  std::vector<CoalescedPass> passes;
  for (auto & [key, merged] : buckets) {
    passes.insert(passes.end(), merged.begin(), merged.end());
  }
  std::sort(passes.begin(), passes.end(),
    [](const CoalescedPass & a, const CoalescedPass & b) {
      if (a.bag_path != b.bag_path) {
        return a.bag_path < b.bag_path;
      }
      return a.t_start_ns < b.t_start_ns;
    });
  return passes;
}

}  // namespace

SurveyOverviewWindow::SurveyOverviewWindow(
  const std::string & index_path, const std::string & stores_dir, QWidget * parent)
: QMainWindow(parent),
  bridge_(std::make_unique<SurveyIndexBridge>(index_path))
{
  setWindowTitle(QString("Survey overview — %1").arg(QString::fromStdString(index_path)));

  canvas_ = new SurveyOverviewCanvas(this);
  pass_list_ = new QTreeWidget(this);
  pass_list_->setColumnCount(4);
  pass_list_->setHeaderLabels({"Start (UTC)", "Duration", "Pings", "Sensor"});
  pass_list_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  pass_list_->setRootIsDecorated(true);

  auto * splitter = new QSplitter(this);
  splitter->addWidget(canvas_);
  splitter->addWidget(pass_list_);
  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 2);
  setCentralWidget(splitter);

  status_ = new QLabel(this);
  statusBar()->addWidget(status_, 1);
  status_->setText("Click the map to list the passes that saw that spot.");

  // The hover read-out is a permanent widget so it doesn't disturb (or get
  // hidden by) the main status_ label the way a transient statusBar message
  // would on every mouse-move.
  hover_ = new QLabel(this);
  statusBar()->addPermanentWidget(hover_);

  connect(canvas_, &SurveyOverviewCanvas::clicked,
    this, &SurveyOverviewWindow::onMapClicked);
  connect(canvas_, &SurveyOverviewCanvas::hoverGeo,
    this, &SurveyOverviewWindow::onHoverGeo);
  connect(pass_list_, &QTreeWidget::itemActivated,
    this, &SurveyOverviewWindow::onPassActivated);

  loadStoreTiles(stores_dir);
  resize(1280, 800);
}

void SurveyOverviewWindow::loadStoreTiles(const std::string & stores_dir)
{
  // The tile level is encoded in the filenames (<level>_<row>_<col>.tif). A
  // store should hold a single level; scan every tile so we render one level
  // deterministically (the lowest) and can warn when the directory mixes
  // levels — otherwise the other levels vanish silently. A missing or empty
  // directory degrades to an empty map — the pass query still works.
  std::error_code ec;
  std::map<int, std::vector<std::string>> by_level;   // level -> tile paths
  for (const auto & entry : std::filesystem::directory_iterator(stores_dir, ec)) {
    const auto name = entry.path().filename().string();
    if (entry.path().extension() != ".tif" || name.find('_') == std::string::npos) {
      continue;
    }
    try {
      by_level[std::stoi(name.substr(0, name.find('_')))].push_back(entry.path().string());
    } catch (const std::exception &) {
      continue;
    }
  }
  if (by_level.empty()) {
    status_->setText(QString("No store tiles found under %1 — map is empty; "
      "pass queries still work.").arg(QString::fromStdString(stores_dir)));
    return;
  }

  const int level = by_level.begin()->first;   // render the lowest level
  const auto & level_paths = by_level.begin()->second;

  // Warn when the store mixes levels: only `level` is rendered, so name the
  // ignored ones instead of dropping them without a trace. (loadTiles() would
  // throw on the off-level tiles, so this loads the chosen level tile-by-tile.)
  QString level_warning;
  if (by_level.size() > 1) {
    QStringList others;
    for (const auto & [lvl, paths] : by_level) {
      if (lvl != level) {
        others << QString::number(lvl);
      }
    }
    level_warning = QString(" (mixed store: ignoring levels %1)").arg(others.join(", "));
  }

  // Load per-tile with a per-tile guard: tileRasterCount/loadTile @throw on a
  // corrupt or unreadable tile, and one bad tile must cost only itself — not
  // the whole map, and certainly not the window (graceful degradation; the
  // pass query works regardless).
  std::map<gggs::GridIndex, marine_tiled_raster_store::TiledRasterTile<double>> tiles;
  int failed_tiles = 0;
  for (const auto & path : level_paths) {
    try {
      const int band_count = marine_tiled_raster_store::tileRasterCount(path);
      if (band_count < 1) {
        ++failed_tiles;
        continue;
      }
      auto tile = marine_tiled_raster_store::loadTile<double>(
        path, gggs::Level(static_cast<std::uint8_t>(level)),
        static_cast<std::size_t>(band_count));
      tiles.emplace(tile.index(), std::move(tile));
    } catch (const std::exception &) {
      ++failed_tiles;
    }
  }
  if (failed_tiles > 0) {
    level_warning += QString(" (%1 unreadable tile%2 skipped)")
      .arg(failed_tiles).arg(failed_tiles == 1 ? "" : "s");
  }
  if (tiles.empty()) {
    status_->setText(QString("No loadable store tiles under %1 — map is empty; "
      "pass queries still work.%2")
      .arg(QString::fromStdString(stores_dir)).arg(level_warning));
    return;
  }

  // One shared depth range across the survey so colours are comparable
  // between tiles.
  double lo = std::numeric_limits<double>::max();
  double hi = std::numeric_limits<double>::lowest();
  for (const auto & [index, tile] : tiles) {
    for (const double v : tile.band(0)) {
      if (!std::isnan(v)) {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
      }
    }
  }
  if (lo > hi) {
    status_->setText("Store tiles hold no finite depths — map is empty.");
    return;
  }
  const auto lut = marine_colormap::bake_lut(
    marine_colormap::palette(0), marine_colormap::TransferParams{}, 256);

  std::vector<OverviewTile> overview;
  overview.reserve(tiles.size());
  for (const auto & [index, tile] : tiles) {
    OverviewTile out;
    out.image = tileToImage(tile, lo, hi, lut);
    out.south = index.southLatitude();
    out.west = index.westLongitude();
    out.north = index.northLatitude();
    out.east = index.eastLongitude();
    overview.push_back(std::move(out));
  }
  canvas_->setTiles(std::move(overview));
  status_->setText(
    QString("%1 store tiles (L%2), depth %3–%4 m — click to query passes.%5")
    .arg(tiles.size()).arg(level).arg(lo, 0, 'f', 1).arg(hi, 0, 'f', 1)
    .arg(level_warning));
}

void SurveyOverviewWindow::onMapClicked(double lat, double lon)
{
  canvas_->setQueryMark(lat, lon);
  pass_list_->clear();
  std::vector<marine_survey_index::PassRow> rows;
  try {
    rows = bridge_->queryPoint(lat, lon);
  } catch (const std::exception & e) {
    status_->setText(QString("Pass query failed: %1").arg(e.what()));
    return;
  }
  if (rows.empty()) {
    status_->setText(QString("No indexed passes at %1, %2.")
      .arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6));
    return;
  }

  const auto passes = coalescePasses(rows);

  // Group by bag (passes are ordered by bag path, then time).
  QTreeWidgetItem * bag_item = nullptr;
  QString current_bag;
  for (const auto & pass : passes) {
    const QString bag = QString::fromStdString(pass.bag_path);
    if (bag_item == nullptr || bag != current_bag) {
      current_bag = bag;
      bag_item = new QTreeWidgetItem(pass_list_, {bag});
      bag_item->setFirstColumnSpanned(true);
      bag_item->setExpanded(true);
    }
    const double duration_s =
      static_cast<double>(pass.t_end_ns - pass.t_start_ns) / 1e9;
    auto * item = new QTreeWidgetItem(bag_item, {
        isoUtc(pass.t_start_ns),
        QString("%1 s").arg(duration_s, 0, 'f', 1),
        QString::number(pass.ping_count),
        QString::fromStdString(pass.sensor_type)});
    item->setData(0, Qt::UserRole, bag);
    item->setData(1, Qt::UserRole, QVariant::fromValue<qlonglong>(pass.t_start_ns));
    item->setData(2, Qt::UserRole, QVariant::fromValue<qlonglong>(pass.t_end_ns));
  }
  status_->setText(QString("%1 passes at %2, %3 — double-click one to open it.")
    .arg(passes.size()).arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6));
}

void SurveyOverviewWindow::onPassActivated(QTreeWidgetItem * item, int)
{
  const QString bag = item->data(0, Qt::UserRole).toString();
  if (bag.isEmpty()) {
    return;   // a bag header row, not a pass
  }
  const auto t_start = static_cast<std::int64_t>(item->data(1, Qt::UserRole).toLongLong());
  const auto t_end = static_cast<std::int64_t>(item->data(2, Qt::UserRole).toLongLong());
  // One viewer per activation; closing it frees it. The overview stays up as
  // the navigation hub.
  auto * viewer = new SidescanViewerWindow();
  viewer->setAttribute(Qt::WA_DeleteOnClose);
  viewer->show();
  viewer->openBag(bag.toStdString(), t_start, t_end);
}

void SurveyOverviewWindow::onHoverGeo(double lat, double lon)
{
  hover_->setText(QString("%1, %2").arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6));
}

}  // namespace marine_perception_tools
