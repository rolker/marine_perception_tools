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

#include <QMainWindow>
#include <QString>

#include <array>
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
  MainWindow(
    BagLoadOptions load_opts, double window_m, double res, double max_range,
    double min_grazing_deg, QWidget * parent = nullptr);

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

  // Buffer manager (Milestone D4). Ensure the engine covers bag-relative time
  // `t_s`: consult the D3 span policy against the current cache; if a reload is
  // needed, read the new window via the BagSession and rebuild the engine, then
  // seek to `t_s`. Cheap (in-window seek) when the policy says NoReload. Returns
  // false and leaves prior state intact on a load error (message to status bar).
  bool ensureCovers(double t_s);
  // Integration warm-up in seconds, derived from the current decay half-life
  // (integration = integration_halflives_ * decay_half_life_s). Recomputed each
  // call so a half-life knob change grows/shrinks the window (R-series).
  double integrationSeconds() const;
  // Build BufferParams from the current knobs + the open session's bounds.
  BufferParams bufferParams() const;

  BagLoadOptions load_opts_;
  double window_m_;
  double res_;
  double max_range_;
  double min_grazing_deg_;
  double integration_halflives_ = 1.0;  // window = this * decay_half_life_s
  double margin_s_ = 10.0;              // reload-free scrub slack each side
  double retention_s_ = 120.0;          // extra cached span kept beyond guaranteed

  std::unique_ptr<BagSession> session_;  // one full scan; serves windowed reloads
  BufferState buffer_state_;             // current I/O cache extent (bag-relative s)
  double pending_seek_s_ = -1.0;         // out-of-span target deferred to release
  std::unique_ptr<ReSimEngine> engine_;
  std::array<QLabel *, kNumCameras> rgb_labels_{};   // Row 1: camera RGB (H.265)
  std::array<QLabel *, kNumCameras> seg_labels_{};   // Row 2: segmentation masks
  QLabel * recorded_label_ = nullptr;                // Row 3 left: recorded costmap
  QLabel * grid_label_ = nullptr;                    // Row 3 right: regenerated costmap
  QSlider * scrubber_ = nullptr;
  QPushButton * apply_btn_ = nullptr;   // commit the dirty knob batch (one re-sim)
  QPushButton * reset_btn_ = nullptr;   // revert dirty knobs to applied values
  int panel_px_ = 360;        // costmap panel side (square)
  int cam_tile_h_ = 165;      // camera/seg tile height (4:3)
  std::vector<Knob> knobs_;
};

}  // namespace marine_perception_tools

#endif  // MAIN_WINDOW_HPP_
