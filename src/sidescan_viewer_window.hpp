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

#ifndef SIDESCAN_VIEWER_WINDOW_HPP_
#define SIDESCAN_VIEWER_WINDOW_HPP_

#include <QFutureWatcher>
#include <QImage>
#include <QMainWindow>
#include <QString>

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "marine_contacts/contact_store.hpp"
#include "marine_sonar_widgets/waterfall_model.hpp"
#include "mbes_pass_loader.hpp"
#include "sidescan_bag_session.hpp"
#include "sidescan_canvas.hpp"   // OverviewTile/GeoRect (index-map layer types)
#include "survey_index_bridge.hpp"
#include "time_bar_widget.hpp"   // TimelinePassInfo (pass bars on the time bar)

class QLabel;
class QSlider;
class QDoubleSpinBox;
class QSpinBox;
class QProgressBar;
class QPushButton;
class QListWidget;
class QCheckBox;
class QComboBox;
class QRectF;
class QCloseEvent;
class QSplitter;
class QTreeWidget;

namespace marine_perception_tools
{

class SidescanCanvas;
class PointCloudView;
class TimeBarWidget;
}  // namespace marine_perception_tools

namespace marine_sonar_widgets {class WaterfallWidget; class EchogramWidget;}

namespace marine_perception_tools
{

// Result of an off-thread window render: the painted coverage image + its map
// extent. Built on a worker thread (readWindow + paint + rasterize) so scrubbing
// never blocks the UI; only the final setCoverage runs on the UI thread.
struct SidescanRenderResult
{
  bool ok = false;
  uint64_t epoch = 0;    // session epoch this render was computed for (stale-drop)
  QImage image;          // georeferenced coverage (map frame)
  // Uncorrected slant-range sidescan rows (shared-lib WaterfallWidget) for the same
  // window: port/starboard combined, newest drawn at top, each carrying its map pose
  // so the widget inverts a marked pixel back to map coordinates.
  std::vector<marine_sonar_widgets::WaterfallRow> sidescan_rows;
  std::vector<MbesSounding> mbes_soundings;  // window's M3 soundings, world frame
  // Boat pose at the scrub head (world frame) for the 3D context arrow.
  bool boat_valid = false;
  double boat_x = 0.0;
  double boat_y = 0.0;
  double boat_z = 0.0;
  double boat_heading = 0.0;
  std::vector<marine_sonar_widgets::WaterfallRow> mbes_backscatter_rows;  // per-ping dB
  std::vector<marine_acoustic_msgs::msg::RawSonarImage> down_images;  // water-column pings
  double origin_x = 0.0;
  double origin_y = 0.0;
  double res_m = 0.25;
  double center_x = 0.0;  // map centre of the window's painted swath
  double center_y = 0.0;
  bool has_center = false;
  std::size_t npings = 0;
  double head_m = 0.0;
  double total_m = 0.0;
  double win_lo = 0.0;
  double win_hi = 0.0;
};

// A multi-pass cloud load in flight: the generation ties the result to the
// selection that requested it, so a superseded load can never clobber a newer
// selection's cloud.
struct CloudLoadTicket
{
  std::uint64_t generation = 0;
  CloudLoadOutcome outcome;
};

// A basemap (store-tile) load in flight: layer/palette changes supersede via
// the generation, same pattern as the cloud load.
struct BasemapLoadTicket
{
  std::uint64_t generation = 0;
  std::vector<OverviewTile> tiles;
  QString note;
};

// Offline sidescan viewer main window: File->Open a bag, then scrub along
// distance travelled. A rolling ~window of pings is painted (quality-wins) into a
// coverage raster at true map position and shown north-up on the canvas, with the
// boat track and a measuring grid. With a survey index (#24) the map becomes the
// explorer's index map — store-tile basemap, nav track, selectable index tiles —
// and selecting tiles auto-loads every mbes-bathy pass of the selection into the
// 3D cloud pane (per-pass colours + legend).
class SidescanViewerWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit SidescanViewerWindow(QWidget * parent = nullptr);
  ~SidescanViewerWindow() override;

  // Enter survey-explorer mode (#24): open `survey_index.db`, place the store
  // tiles / nav track / selectable index tiles on the map, and drive the cloud
  // pane from the tile selection. Throws std::runtime_error on a missing or
  // incompatible index (the bridge's regenerate hint propagates).
  void openSurveyIndex(const std::string & index_path, const std::string & stores_dir);

  // Open a bag directly (e.g. from a CLI argument). Non-zero cue bounds
  // (UNIX ns) jump the scrub to the along-track window those stamps cover once
  // indexing completes — the survey-index (jump-to-pass) bridge. The whole bag
  // is still metadata-indexed first (TF + cumulative distance need the full
  // recording); only sample data is window-read, so the cue costs one normal
  // index pass, not a whole-bag sample read.
  void openBag(const std::string & bag_uri, int64_t cue_start_ns = 0, int64_t cue_end_ns = 0);

protected:
  // Persist window geometry + splitter sizes on close (QSettings).
  void closeEvent(QCloseEvent * event) override;

  // Route scrub keys (Left/Right/PageUp/PageDown/Home/End) to the scrub slider from
  // anywhere in the window, so scrubbing works without the slider holding focus —
  // except while editing a control (spin box / combo / list / the slider itself).
  bool eventFilter(QObject * obj, QEvent * event) override;

signals:
  // Emitted from the background indexer (worker thread) as the bag resolves; the
  // queued connection marshals it to the UI thread. `epoch` guards against a stale
  // bag's worker updating after a newer bag was opened.
  void indexProgress(quint64 epoch, double resolved_distance_m, bool done);

private slots:
  void onOpenBag();
  void onIndexProgress(quint64 epoch, double resolved_distance_m, bool done);
  void onRenderFinished();
  void onScrubChanged();
  void onGridSpacingChanged(double metres);
  void onWindowLengthChanged(double metres);
  void onMaxPingsChanged(int max_pings);
  void onContactMarked(const QRectF & map_rect);
  void onSaveContacts();
  void onLoadContacts();
  void onExportGeoJson();
  void onTileSelectionChanged();
  void onCloudPassesLoaded();
  void onBasemapLoaded();
  void onTimelinePassActivated(const QString & bag_path, qlonglong t_start_ns, qlonglong t_end_ns);
  void onTimeSelected(qlonglong t_ns);   // time-bar centre committed: cue there
  void onCenterTimeChanged(qlonglong t_ns);   // live: move the map's position arrow

private:
  // Cross-pane linked cursor + click-to-seek coordination. A world map point is the
  // shared currency; the echogram works in along-track fraction within the window.
  void onCursorHover(double map_x, double map_y, bool valid);   // broadcast the cursor
  void onCursorSeek(double map_x, double map_y);                // seek scrub to a point
  void onEchogramHover(double frac, bool valid);                // echogram-sourced hover
  void onEchogramSeek(double frac);                             // echogram-sourced seek

  void refreshContacts();   // push the store to the map overlay + the list

  // Populate the basemap layer combo from the store layers under `root`
  // (subdirectories holding GGGS *.tif tiles), selecting `initial_dir`.
  void discoverBasemapLayers(const std::string & root, const std::string & initial_dir);
  // (Re)load the selected basemap layer with the selected colormap on a
  // worker thread; a newer request supersedes via basemap_gen_.
  void requestBasemapLoad();
  // Leave selection-cloud mode: restore the scrub-window cloud + colour mode.
  void exitSelectionCloud();
  // Cloud-legend label for a pass, in the time bar's display zone (#26).
  std::string passLabel(std::int64_t t_start_ns, const std::string & bag_path) const;
  // Re-render the legend's baked pass labels after a display-zone change.
  void refreshPassLabels();
  // Launch a window render on a worker thread, coalescing rapid scrub changes:
  // if a render is in flight, just flag a pending one and re-launch on finish
  // with the latest scrub position.
  void requestRender();

  // Set the scrub slider's arrow-key step to 20% of the window and the page step
  // to a full window (slider units are metres).
  void updateScrubStep();

  SidescanCanvas * canvas_ = nullptr;
  QSlider * scrub_ = nullptr;
  QDoubleSpinBox * grid_spin_ = nullptr;
  QDoubleSpinBox * window_spin_ = nullptr;
  QSpinBox * max_pings_spin_ = nullptr;
  QLabel * status_ = nullptr;
  QProgressBar * progress_ = nullptr;
  QPushButton * mark_button_ = nullptr;
  QListWidget * contact_list_ = nullptr;
  QComboBox * palette_combo_ = nullptr;
  marine_sonar_widgets::WaterfallWidget * waterfall_ = nullptr;
  PointCloudView * cloud_ = nullptr;
  QComboBox * cloud_color_combo_ = nullptr;
  QDoubleSpinBox * zexag_spin_ = nullptr;
  QDoubleSpinBox * point_size_spin_ = nullptr;
  marine_sonar_widgets::WaterfallWidget * mbes_waterfall_ = nullptr;
  marine_sonar_widgets::EchogramWidget * echogram_ = nullptr;

  // Per-pane colormap selectors (the map keeps palette_combo_; the 3D cloud gets
  // its own marine_colormap palette via cloud_palette_).
  QComboBox * sidescan_cmap_ = nullptr;
  QComboBox * mbes_cmap_ = nullptr;
  QComboBox * echo_cmap_ = nullptr;
  QComboBox * cloud_palette_ = nullptr;

  // Nested resizable-pane layout: [contacts | map | 2x2 grid]; the grid is a
  // vertical splitter of two horizontal rows. Held so their sizes persist (QSettings).
  QSplitter * outer_split_ = nullptr;
  QSplitter * grid_split_ = nullptr;
  QSplitter * grid_top_split_ = nullptr;
  QSplitter * grid_bot_split_ = nullptr;

  marine_contacts::ContactStore contact_store_;
  int contact_counter_ = 0;

  QFutureWatcher<void> index_watcher_;   // background index build (buildIndex)
  // Index workers superseded by a newer openBag. Their lambdas capture `this`
  // for the progress emit, and setFuture neither cancels nor waits the old
  // future — so the destructor must wait these out too, or an orphaned
  // indexer's emit dereferences a freed window (found in #24 review).
  std::vector<QFuture<void>> superseded_index_futures_;
  uint64_t index_epoch_ = 0;             // bumped per opened bag; guards stale progress
  QFutureWatcher<SidescanRenderResult> render_watcher_;
  bool loading_ = false;
  bool rendering_ = false;
  bool render_pending_ = false;

  std::shared_ptr<SidescanBagSession> session_;
  double window_len_m_ = 100.0;
  uint64_t session_epoch_ = 0;   // bumped on each loaded bag; stale renders are dropped
  int max_window_pings_ = 600;   // stationary cap; also sets raster res = window / this
  double last_win_lo_ = 0.0;     // current render window span, for echogram cursor mapping
  double last_win_hi_ = 0.0;
  // Jump-to-pass cue (UNIX ns), applied once when indexing completes then cleared;
  // 0 = no cue pending.
  int64_t pending_cue_start_ns_ = 0;
  int64_t pending_cue_end_ns_ = 0;

  // --- survey-explorer mode (#24) ---
  std::unique_ptr<SurveyIndexBridge> bridge_;   // null in bag-only mode
  std::vector<IndexedTile> indexed_tiles_;      // canvas selection indices map here
  QTreeWidget * cloud_legend_ = nullptr;        // per-pass colours + counts
  QSplitter * cloud_split_ = nullptr;           // [cloud | legend]
  QLabel * hover_geo_ = nullptr;                // lat/lon readout (geo mode)
  QFutureWatcher<CloudLoadTicket> cloud_watcher_;
  std::uint64_t cloud_gen_ = 0;                 // bumped per selection change
  std::vector<CloudPassInfo> cloud_passes_;     // passes of the in-flight/last load
  // The last load's per-pass clouds, kept so legend checkboxes can toggle
  // passes without re-reading bags (index-aligned with cloud_passes_).
  std::vector<std::vector<MbesSounding>> cloud_pass_clouds_;
  bool selection_cloud_ = false;   // cloud pane shows the tile selection, not the scrub window
  TimeBarWidget * time_bar_ = nullptr;   // GeoZui-style time navigator (replaced phase d's axis)
  std::vector<TimelinePassInfo> selection_passes_;   // for time->bag lookup on cue
  // Campaign nav track + bag paths (index mode): the time-bar position arrow
  // interpolates the boat's fix from these, and a committed time resolves to
  // an openable bag through them.
  std::vector<marine_survey_index::NavPoint> nav_track_points_;
  std::vector<std::pair<std::int64_t, std::string>> bag_paths_;
  // Bag-index cache directory (session_index_io); empty disables caching.
  std::string cache_dir_;

public:
  void setCacheDir(const std::string & dir) {cache_dir_ = dir;}

private:
  std::string current_bag_uri_;    // open bag; a same-bag timeline cue skips the re-open

  // Basemap controls (#24 follow-up from desk verify): store layer + colormap,
  // and declutter toggles for the overlays that otherwise blanket the basemap.
  QComboBox * basemap_layer_ = nullptr;
  QComboBox * basemap_cmap_ = nullptr;
  QCheckBox * show_track_check_ = nullptr;
  QCheckBox * show_grid_check_ = nullptr;
  QCheckBox * utc_check_ = nullptr;
  // Clip the selection cloud to the selected contact + margin (#24 desk
  // finding: several passes over a tile is millions of soundings).
  QCheckBox * clip_contact_check_ = nullptr;
  QDoubleSpinBox * clip_margin_spin_ = nullptr;
  std::vector<std::pair<QString, std::string>> basemap_layers_;   // {label, dir}
  QFutureWatcher<BasemapLoadTicket> basemap_watcher_;
  std::uint64_t basemap_gen_ = 0;
};

}  // namespace marine_perception_tools

#endif  // SIDESCAN_VIEWER_WINDOW_HPP_
