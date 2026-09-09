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
#include <QMutex>
#include <QString>
#include <QTimer>

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cube_lab.hpp"
#include "sidescan_drape.hpp"
#include "marine_contacts/contact_store.hpp"
#include "marine_sonar_widgets/waterfall_model.hpp"
#include "mbes_pass_loader.hpp"
#include "sidescan_bag_session.hpp"
#include "sidescan_canvas.hpp"   // OverviewTile/GeoRect (index-map layer types)
#include "survey_index_bridge.hpp"
#include "time_bar_widget.hpp"   // TimelinePassInfo (pass bars on the time bar)

class QAction;
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
class BasemapLod;
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
  bool cancelled = false;   // the window is closing (#44): do not publish
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
  CloudLoadOutcome outcome;   // outcome.cancelled marks a teardown-abandoned load
};

// A box-CUBE run in flight (#27): the gathered box soundings (reference
// world frame) + the estimated surface. Superseded via the generation.
struct CubeLabTicket
{
  std::uint64_t generation = 0;
  std::vector<MbesSounding> soundings;
  CubeSurface surface;
  QStringList notes;
  qint64 elapsed_ms = 0;
  bool cancelled = false;   // the window is closing (#44): do not publish
  // Reference frame identity from the cloud load (#29): the sidescan drape
  // reprojects its pings into this frame.
  std::string ref_bag;
  bool ref_has_geo = false;
  geometry_msgs::msg::TransformStamped ref_earth_from_world;
};

// A sidescan drape load+march in flight (#29), superseded via generation.
struct DrapeTicket
{
  std::uint64_t generation = 0;
  SidescanDrape drape;
  // The terrain the drape marched on: the CUBE surface extended to the
  // pass's swath, holes filled / edges extrapolated (uncertainty NaN on
  // interpolated nodes — never presented as bathymetry).
  CubeSurface terrain;
  QStringList notes;
  qint64 elapsed_ms = 0;
  bool cancelled = false;   // the window is closing (#44): do not publish
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

  // Startup convenience: open the remembered last index (QSettings), falling
  // back to the conventional world collection (#40). Called by main when the
  // app launches with neither --index nor a bag argument, so a plain start
  // comes back where the operator left off, or at the world model, instead of
  // empty.
  void openStartupIndex();

protected:
  // Persist window geometry + splitter sizes on close (QSettings).
  void closeEvent(QCloseEvent * event) override;

  // Set every worker cancel token (#44). Called from closeEvent (so the jobs
  // have the whole teardown to notice) and again from the destructor.
  void cancelWorkers();

  // Route scrub keys (Left/Right/PageUp/PageDown/Home/End) to the scrub slider from
  // anywhere in the window, so scrubbing works without the slider holding focus —
  // except while editing a control (spin box / combo / list / the slider itself).
  bool eventFilter(QObject * obj, QEvent * event) override;

signals:
  // Emitted from the background indexer (worker thread) as the bag resolves; the
  // queued connection marshals it to the UI thread. `epoch` guards against a stale
  // bag's worker updating after a newer bag was opened.
  void indexProgress(quint64 epoch, double resolved_distance_m, bool done);
  // The worker constructed the session (parked in pending_open_session_) /
  // failed to open the bag. Queued to the UI thread like indexProgress.
  void sessionOpened(quint64 epoch);
  void openFailed(quint64 epoch, const QString & message);

private slots:
  void onOpenBag();
  void onOpenIndex();   // File menu: pick a survey_index.db (#27 follow-up)
  void onExportSurfaceData();   // GeoTIFF: float depth/uncertainty/backscatter
  void onExportSurfaceRgba();   // GeoTIFF: the rendered shade, RGBA
  void onReopenLastIndex();   // File menu: reload the remembered index
  void onIndexProgress(quint64 epoch, double resolved_distance_m, bool done);
  void onSessionOpened(quint64 epoch);
  void onOpenFailed(quint64 epoch, const QString & message);
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

  // Load the vendored world coastline into the map's bottom layer (#41), once
  // per session and never over the network — the operator station and the
  // boat have no route to one. A missing or unreadable dataset leaves the map
  // without a coastline; it is a packaging fault, not an operator's problem,
  // so it warns and carries on.
  void loadCoastlineLayer();
  bool coastline_loaded_ = false;

  // Populate the basemap layer combo from the store layers under `root`
  // (subdirectories holding GGGS *.tif tiles), selecting `initial_dir`.
  void discoverBasemapLayers(const std::string & root, const std::string & initial_dir);
  // (Re)open the selected basemap layer with the selected colormap in the
  // LOD loader (#26); a newer request supersedes in the loader.
  void requestBasemapLoad();
  // Push the canvas's settled view into the LOD loader (level + demand load).
  void pushBasemapView();
  // Leave selection-cloud mode: restore the scrub-window cloud + colour mode.
  void exitSelectionCloud();
  // Cloud-legend label for a pass, in the time bar's display zone (#26).
  std::string passLabel(std::int64_t t_start_ns, const std::string & bag_path) const;
  // Debounced scrub-driven open: the last commit before the hand settles wins.
  void scheduleOpen(const std::string & bag_uri, int64_t t0_ns, int64_t t1_ns);
  // Build + wire the per-pane colour-range controls (#26); ctor helper, must
  // run before the pane headers consume the widgets.
  void setupRangeControls();
  // Build + wire the CUBE-lab controls row (#27) into the cloud pane;
  // ctor helper, run after the pane exists.
  void setupCubeLab(QWidget * cloud_pane);
  // Open an index with the default sibling stores root; errors -> message box.
  void openIndexWithDefaults(const std::string & index_path);
  // Sync the Reopen Last Index action's label/enabled state with QSettings.
  void refreshReopenIndexAction();
  // Derive an ARA curve from the last run's own beams, adopt it, re-run.
  void selfCalibrateBackscatter();
  // Gather the box's mbes passes, load + clip their soundings, run CUBE at
  // the chosen cell size on a worker; results land in onCubeLabFinished.
  void runCubeLab();
  // Re-triangulate + recolour the stored surface for the shade combo (cheap;
  // no CUBE re-run) and hand it to the cloud pane.
  void refreshCubeSurface();
  // The CUBE frame's world->geo affine, probed through the reference
  // earth anchor; nullopt without a geo reference (export refuses then).
  std::optional<MapGeoAffine> cubeSurfaceAnchor() const;
  // Fill the drape pass combo with the sidescan passes crossing the box
  // (port + starboard of the same bag interval merged into one entry).
  void populateDrapePasses();
  // Load + march the selected pass onto the current surface on a worker.
  void requestDrape();
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
  // Cancel token for the CURRENT scan; superseding an open sets it so the
  // abandoned worker stops streaming the bag instead of running to the end.
  std::shared_ptr<std::atomic<bool>> scan_cancel_;
  // Teardown cancel token for the four workers that had none (#44): the
  // sidescan render, the cloud-pass load, the CUBE run and the drape. Set in
  // closeEvent — the whole teardown before the destructor's waits, so the
  // operator gets their prompt back instead of watching a dead window — and
  // again in the destructor as the backstop for a window destroyed without a
  // close. Never reset: it means "this window is going away", not "this job
  // is superseded" (supersede is the generation counters' job).
  std::shared_ptr<std::atomic<bool>> worker_cancel_ =
    std::make_shared<std::atomic<bool>>(false);
  // Session handoff worker -> UI (the constructor runs in the worker so the
  // UI never touches a multi-GB bag synchronously): the worker parks the
  // session + its epoch here, then emits sessionOpened.
  QMutex pending_open_mutex_;
  std::shared_ptr<SidescanBagSession> pending_open_session_;
  quint64 pending_open_epoch_ = 0;
  // Debounce for scrub-driven opens (time-bar commits): rapid fine-tune
  // commits collapse into one openBag once the hand settles.
  QTimer open_debounce_;
  std::string debounce_uri_;
  int64_t debounce_t0_ns_ = 0;
  int64_t debounce_t1_ns_ = 0;
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

  // Basemap controls (#24 follow-up from desk verify): store layer + colormap.
  QComboBox * basemap_layer_ = nullptr;
  QComboBox * basemap_cmap_ = nullptr;
  QCheckBox * utc_check_ = nullptr;

  // Map overlay toggles, in the View menu (#42): four checkboxes crowding the
  // map pane header read as clutter and their one-word labels only worked
  // because they sat in a row. As a vertical list they can say what they are.
  // Disabled until an index opens, the moment the overlays have anything to
  // draw. A proper layer list is later, deliberate work (#36).
  QAction * show_track_action_ = nullptr;
  QAction * show_grid_action_ = nullptr;
  QAction * show_coast_action_ = nullptr;
  QAction * show_metric_grid_action_ = nullptr;

  // Per-pane colour-range controls (#26): auto (default) or a manual lo/hi
  // in the pane's native units. The sidescan range also drives the map's
  // coverage-overlay render (same data, same scale).
  struct RangeControls
  {
    QCheckBox * auto_check = nullptr;
    QDoubleSpinBox * lo = nullptr;
    QDoubleSpinBox * hi = nullptr;
  };
  RangeControls ss_range_;      // sidescan waterfall + map coverage, 0..1
  RangeControls bs_range_;      // MBES backscatter waterfall, 0..1
  RangeControls wc_range_;      // water column: black/white points, 0..1
  RangeControls cloud_range_;   // 3D cloud scalar (depth m / intensity)
  RangeControls map_range_;     // basemap contrast, layer units

  // CUBE lab (#27): the shift-drag box, its in-flight run and last surface.
  std::optional<GeoRect> cube_box_;
  QFutureWatcher<CubeLabTicket> cube_watcher_;
  std::uint64_t cube_gen_ = 0;
  CubeSurface cube_surface_;
  QDoubleSpinBox * cube_cell_spin_ = nullptr;
  QComboBox * cube_order_combo_ = nullptr;
  QPushButton * cube_run_btn_ = nullptr;
  QCheckBox * cube_points_check_ = nullptr;
  QCheckBox * cube_surf_check_ = nullptr;
  QComboBox * cube_mesh_combo_ = nullptr;   // crisp+smooth / stepped / blended
  QComboBox * cube_palette_ = nullptr;      // surface palette, independent of the cloud's

  // Sidescan drape (#29): pass picker + the draped amplitudes for the
  // "Sidescan" surface shade. The reference identity comes from the CUBE
  // load; entries merge port+starboard of the same bag interval.
  struct DrapePassEntry
  {
    std::string bag_path;
    std::int64_t t0_ns = 0;
    std::int64_t t1_ns = 0;
  };
  QComboBox * cube_drape_combo_ = nullptr;
  QComboBox * cube_range_score_combo_ = nullptr;   // near-wins vs mid-range-wins
  RangeControls cube_srange_;   // surface-shade colour range (active shade's units)
  std::vector<DrapePassEntry> drape_passes_;
  SidescanDrape cube_drape_;
  CubeSurface cube_drape_terrain_;   // the extended terrain the drape rode
  QFutureWatcher<DrapeTicket> drape_watcher_;
  std::uint64_t drape_gen_ = 0;
  std::string cube_ref_bag_;
  bool cube_ref_has_geo_ = false;
  geometry_msgs::msg::TransformStamped cube_ref_anchor_;
  QPushButton * cube_params_btn_ = nullptr;
  QPushButton * cube_selfcal_btn_ = nullptr;   // derive ARA curve from the box
  std::vector<MbesSounding> cube_soundings_;   // last run's beams (self-cal input)
  CubeTuning cube_tuning_;   // seeded from the library defaults in setupCubeLab
  // File-menu quick reload: shows the remembered index (QSettings) and
  // refreshes after every successful openSurveyIndex.
  QAction * reopen_index_action_ = nullptr;
  QDoubleSpinBox * cube_alpha_spin_ = nullptr;
  QComboBox * cube_shade_combo_ = nullptr;
  // Clip the selection cloud to the selected contact + margin (#24 desk
  // finding: several passes over a tile is millions of soundings).
  QCheckBox * clip_contact_check_ = nullptr;
  QDoubleSpinBox * clip_margin_spin_ = nullptr;
  std::vector<std::pair<QString, std::string>> basemap_layers_;   // {label, dir}
  BasemapLod * basemap_lod_ = nullptr;   // LOD basemap loader (#26), child
};

}  // namespace marine_perception_tools

#endif  // SIDESCAN_VIEWER_WINDOW_HPP_
