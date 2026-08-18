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

#ifndef BASEMAP_LOD_HPP_
#define BASEMAP_LOD_HPP_

// LOD basemap for the survey explorer (#26), borrowing camp's ADR-0013
// (camp#103) shape at QPainter-canvas scale:
//  - the level ladder is the layer directory's native level plus whatever the
//    `overviews/` sidecar carries (uma ADR-0011: coarser GGGS tiles folded
//    4->1 down to an L0 apex);
//  - selectLodLevel() picks the finest level whose cells the screen can
//    resolve;
//  - loads are demand-driven: only not-yet-resident tiles of the selected
//    level intersecting the viewport are read, one worker at a time with a
//    snapshotted filter;
//  - progressive refinement: stale levels stay resident as the backdrop
//    (drawn coarse->fine under the selected level) until the selected
//    level's visible set has fully landed, then they are released — except
//    the coarsest level, which stays as the permanent cheap backdrop.
// The contrast range and colormap LUT are fixed per open() across every
// level (the overview fold is MEAN — uma ADR-0011 — so overview values are
// contained in the fine range), keeping colours stable across LOD switches.

#include <QFutureWatcher>
#include <QObject>
#include <QString>

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "marine_colormap/colormap.hpp"
#include "sidescan_canvas.hpp"   // OverviewTile, GeoRect

namespace marine_perception_tools
{

// [camp ADR-0013] The display LOD: the level of `available_levels` closest
// to the ideal GGGS level for `ground_metres_per_pixel`, preferring the
// coarser side; -1 for an empty ladder. The ideal is the coarsest level
// whose cells are at most one screen pixel; from the available levels the
// finest coarser-or-equal one wins (never loading finer data than the
// screen can resolve), falling back to the coarsest available when
// everything is finer than ideal.
int selectLodLevel(
  double ground_metres_per_pixel, const std::vector<int> & available_levels);

using BasemapTileKey = std::pair<std::uint32_t, std::uint32_t>;   // row, col

struct BasemapTileRef
{
  GeoRect bounds;
  std::string path;
};

class BasemapLod : public QObject
{
  Q_OBJECT

public:
  explicit BasemapLod(QObject * parent = nullptr);
  ~BasemapLod() override;

  // Discover `dir` (+ `dir/overviews`), sample the layer contrast range,
  // bake the LUT and load the coarsest level — all on a worker. A newer
  // open() supersedes an in-flight one. Emits opened() then tilesChanged().
  void open(const std::string & dir, std::size_t palette_idx, bool zero_is_nodata);

  // The view moved: (re)select the level for this zoom and, when idle, kick
  // a load for its visible not-yet-resident tiles. Cheap when nothing is
  // missing. `viewport` in degrees; `ground_metres_per_pixel` true ground.
  void viewChanged(const GeoRect & viewport, double ground_metres_per_pixel);

  // Resident tiles in paint order: non-selected levels coarse->fine (the
  // refinement backdrop), the selected level last (on top).
  std::vector<OverviewTile> renderTiles() const;

  // Union of the native (main-directory) level's tile bounds — the data
  // footprint. Overview tiles pad to their coarse grid cell (the L0 apex
  // spans a whole grid row), so they never participate (camp ADR-0013
  // extent semantics). Known as soon as opened() fires (filename-derived).
  std::optional<GeoRect> dataExtent() const {return data_extent_;}

  QString note() const {return note_;}
  bool isOpen() const {return discovered_;}

signals:
  // Discovery finished: dataExtent()/note() are valid (pixels may still be
  // loading; tilesChanged() follows per landed batch).
  void opened();
  // The resident tile set changed — re-pull renderTiles().
  void tilesChanged();

private slots:
  void onWorkerFinished();

private:
  using TileKey = BasemapTileKey;
  using TileRef = BasemapTileRef;

  // Everything a load pass needs, snapshotted (camp ADR-0013: the live
  // members are reassigned per view change; the worker must never read them).
  struct LoadJob
  {
    quint64 gen = 0;
    int level = -1;
    bool is_open = false;   // open() pass: carries discovery + range + LUT
    std::string dir;
    std::size_t palette_idx = 0;
    bool zero_is_nodata = false;
    std::vector<std::pair<TileKey, TileRef>> tiles;   // refs to load
  };

  struct LoadResult
  {
    quint64 gen = 0;
    int level = -1;
    bool is_open = false;
    std::map<int, std::map<TileKey, TileRef>> discovery;   // open pass only
    std::optional<GeoRect> extent;                          // open pass only
    double range_lo = 0.0;
    double range_hi = 0.0;
    std::vector<marine_colormap::Rgba8> lut;                // open pass only
    QString note;
    std::vector<std::pair<TileKey, OverviewTile>> tiles;
  };

  void kickIfNeeded();
  std::vector<std::pair<TileKey, TileRef>> missingVisible(int level) const;
  void releaseStaleLevels();
  void enforceResidentCap();

  // Discovery (per open()).
  std::map<int, std::map<TileKey, TileRef>> level_refs_;
  std::optional<GeoRect> data_extent_;
  bool discovered_ = false;
  std::string dir_;
  std::size_t palette_idx_ = 0;
  bool zero_is_nodata_ = false;
  double range_lo_ = 0.0;
  double range_hi_ = 0.0;
  std::vector<marine_colormap::Rgba8> lut_;
  QString note_;

  // Resident pixels per level.
  std::map<int, std::map<TileKey, OverviewTile>> resident_;

  // View + selection state.
  GeoRect viewport_{};
  bool have_viewport_ = false;
  double ground_mpp_ = 1.0;
  int selected_level_ = -1;

  // Worker (one at a time, latest-wins via gen; cancel stops mid-list).
  QFutureWatcher<LoadResult> worker_;
  quint64 gen_ = 0;
  std::shared_ptr<std::atomic<bool>> cancel_;
};

}  // namespace marine_perception_tools

#endif  // BASEMAP_LOD_HPP_
