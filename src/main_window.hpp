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

#ifndef MAIN_WINDOW_HPP_
#define MAIN_WINDOW_HPP_

#include <QFutureWatcher>
#include <QMainWindow>
#include <QString>

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "bag_loader.hpp"
#include "buffer_policy.hpp"
#include "resim_engine.hpp"

class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QResizeEvent;
class QSlider;

namespace marine_perception_tools
{

// The sea_surface_tuner main window: a forward-camera segmentation pane beside
// the re-simulated costmap, a frame scrubber, and a parameter dock. Editing a
// knob re-simulates and refreshes the costmap; scrubbing seeks the engine.
//
// The window owns the engine and rebuilds it on File->Open, so it can be opened
// with no bag (empty state) and load one interactively. The window geometry /
// replay options are fixed for the window's lifetime (they size the
// OccupancyBuffer); each opened bag builds a fresh engine with them.
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  // window_m/res/max_range/min_grazing_deg size the OccupancyBuffer + projection
  // (construction-fixed). integration_halflives/margin_s/retention_s tune the
  // windowed-buffer policy (Milestone D4): window = integration_halflives *
  // decay_half_life_s of warm-up, +/- margin_s of reload-free scrub slack, with
  // retention_s of extra already-read frames kept beyond the guaranteed window.
  MainWindow(
    BagLoadOptions load_opts, double window_m, double res, double max_range,
    double min_grazing_deg, double integration_halflives = 1.0,
    double margin_s = 10.0, double retention_s = 120.0, QWidget * parent = nullptr);

  // Wait for any in-flight background window load before teardown, so its
  // completion can't fire onLoadFinished on a half-destructed window.
  ~MainWindow() override;

  // Open a bag: build a BagSession (one full scan), set the whole-bag time
  // scrubber range, and load the initial window around t=0. On failure the
  // message goes to the status bar and the prior session/engine (if any) is left
  // intact. Safe to call repeatedly (File->Open).
  void openBag(const QString & bag_uri);

private slots:
  void onSeek(int decisec);          // live during drag: in-window seek only
  void onSeekReleased();             // commit: reload the window if out of span
  void onParamEdited();              // mark a knob dirty (no re-sim — batched)
  void onApplyParams();              // validate+apply the dirty batch, one re-sim
  void onResetParams();              // revert dirty boxes to the applied values
  void onLoadFinished();             // a background window load completed
  void onOpen();

private:
  // One row of the data-driven param dock. Milestone B (after
  // unh_marine_perception#22-P1) swaps the knob set by editing the table that
  // builds these — not the widget code.
  //
  // Editing a box does NOT re-simulate (a full warm-up replay is too expensive
  // per keystroke — see Milestone D4 profiling). Instead an edit marks the knob
  // dirty (distinct box colour); pressing Apply validates the whole batch and
  // re-sims once. `read` pulls the applied value from the engine (for display /
  // Reset); `stage` writes the box's value into a staged params struct that Apply
  // hands to ReSimEngine::setParams.
  struct Knob
  {
    std::string label;
    std::function<double()> read;  // applied value from the engine (or default)
    std::function<void(
        double v,
        sea_surface_segmentation::OccupancyParams & occ,
        sea_surface_segmentation::AccumulateParams & acc)> stage;  // box -> staged
    QDoubleSpinBox * box = nullptr;
    bool dirty = false;
  };

  void buildMenu();
  void buildParamDock();
  void setKnobDirty(Knob & knob, bool dirty);  // toggle the dirty-colour cue
  void syncToEngine();  // scrubber range + knob spinboxes + views for current engine
  void refreshViews();
  bool haveEngine() const {return engine_ != nullptr;}

  // Buffer manager (Milestone D4). Ensure bag-relative time `t_s` is shown:
  //  - if the loaded engine already covers it, seek synchronously (cheap via the
  //    checkpoint store);
  //  - otherwise launch a BACKGROUND window load (D3 span policy → loadWindow →
  //    new engine warmed to t_s) and grey the panes until it lands. The GUI stays
  //    responsive; a newer requestCoverage while a load is in flight just updates
  //    the chased target, and onLoadFinished re-launches for it if the finished
  //    window doesn't cover it (single-flight + chase-latest supersession).
  void requestCoverage(double t_s);
  void startLoad(double t_s);        // kick a background load for t_s
  bool engineCovers(double t_s) const;  // does the loaded engine's window cover t_s?
  // Integration warm-up in seconds, derived from the current decay half-life
  // (integration = integration_halflives_ * decay_half_life_s). Recomputed each
  // call so a half-life knob change grows/shrinks the window (R-series).
  double integrationSeconds() const;
  // Build BufferParams from the current knobs + the open session's bounds.
  BufferParams bufferParams() const;

  // The product of one background window load, moved back to the GUI thread.
  // shared_ptr so it survives the QFuture copy; engine null + error set on
  // failure. `target_s`/`cache_*` echo the request so onLoadFinished can install
  // the buffer_state_ and detect supersession.
  struct LoadResult
  {
    std::shared_ptr<ReSimEngine> engine;
    double target_s = 0.0;
    double cache_lo = 0.0;
    double cache_hi = 0.0;
    std::string error;  // empty on success
    std::uint64_t request_id = 0;
  };
  // Run on a worker thread (no Qt calls): load the window + build & warm the
  // engine. Static so it can't accidentally touch GUI state.
  static LoadResult loadWindowJob(
    std::shared_ptr<BagSession> session, BufferParams params, BufferState state,
    double t_s, double window_m, double res, double max_range, double min_grazing_deg,
    sea_surface_segmentation::OccupancyParams occ, std::uint64_t request_id);

  BagLoadOptions load_opts_;
  double window_m_;
  double res_;
  double max_range_;
  double min_grazing_deg_;
  double integration_halflives_ = 1.0;  // window = this * decay_half_life_s
  double margin_s_ = 10.0;              // reload-free scrub slack each side
  double retention_s_ = 120.0;          // extra cached span kept beyond guaranteed

  std::shared_ptr<BagSession> session_;  // one full scan; serves windowed reloads
  BufferState buffer_state_;             // current I/O cache extent (bag-relative s)
  double pending_seek_s_ = -1.0;         // out-of-span target deferred to release
  std::shared_ptr<ReSimEngine> engine_;

  // Async window-load state. One load in flight at a time; `load_in_flight_`
  // guards it, `chase_target_s_` (>= 0) holds a target requested while busy that
  // the in-flight result must be checked against, and `request_id_` supersedes
  // stale finishes.
  QFutureWatcher<LoadResult> load_watcher_;
  bool load_in_flight_ = false;
  double chase_target_s_ = -1.0;
  std::uint64_t request_id_ = 0;
  std::array<QLabel *, kNumCameras> rgb_labels_{};   // Row 1: camera RGB (H.265)
  std::array<QLabel *, kNumCameras> seg_labels_{};   // Row 2: segmentation masks
  QLabel * recorded_label_ = nullptr;                // Row 3 left: recorded costmap
  QLabel * grid_label_ = nullptr;                    // Row 3 right: regenerated costmap
  QSlider * scrubber_ = nullptr;
  QPushButton * apply_btn_ = nullptr;   // commit the dirty knob batch (one re-sim)
  QPushButton * reset_btn_ = nullptr;   // revert dirty knobs to applied values
  std::vector<Knob> knobs_;

protected:
  // Re-fit the panes when the window / splitter resizes (pixmaps are scaled to
  // the labels' current size in refreshViews).
  void resizeEvent(QResizeEvent * event) override;
};

}  // namespace marine_perception_tools

#endif  // MAIN_WINDOW_HPP_
