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

#include "basemap_lod.hpp"

#include <QtConcurrent/QtConcurrent>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "marine_autonomy/gggs.h"
#include "marine_tiled_raster_store/tile_io.hpp"

#include "basemap_contrast.hpp"

namespace marine_perception_tools
{

int selectLodLevel(
  double ground_metres_per_pixel, const std::vector<int> & available_levels)
{
  // [camp ADR-0013] fromCellSize clamps into the valid GGGS range, so any
  // positive metres-per-pixel is safe; guard non-positive and NaN inputs.
  if (available_levels.empty()) {
    return -1;
  }
  const double mpp = std::isfinite(ground_metres_per_pixel) ?
    std::max(ground_metres_per_pixel, 1e-6) : 1e-6;
  const int ideal = gggs::Level::fromCellSize(static_cast<float>(mpp)).level();
  int best_coarser = -1;   // largest available level number <= ideal
  int coarsest = available_levels.front();
  for (const int level : available_levels) {
    coarsest = std::min(coarsest, level);
    if (level <= ideal && level > best_coarser) {
      best_coarser = level;
    }
  }
  return best_coarser >= 0 ? best_coarser : coarsest;
}

namespace
{

// Per-kick and residency bounds: a load pass reads at most kMaxTilesPerKick
// tiles (further batches follow demand-driven), and the resident set stays
// under kResidentCapTiles full-res ARGB tiles (~1 MB each) by evicting
// farthest-from-view tiles of non-selected levels first.
constexpr std::size_t kMaxTilesPerKick = 256;
constexpr std::size_t kResidentCapTiles = 320;
constexpr std::size_t kMaxRangeSampleTiles = 64;
constexpr std::size_t kMaxRangeSamples = 2000000;

bool intersects(const GeoRect & a, const GeoRect & b)
{
  return a.west < b.east && b.west < a.east && a.south < b.north && b.south < a.north;
}

QPointF centreOf(const GeoRect & r)
{
  return QPointF(0.5 * (r.west + r.east), 0.5 * (r.south + r.north));
}

double centreDist2(const GeoRect & r, const QPointF & c)
{
  const QPointF d = centreOf(r) - c;
  return d.x() * d.x() + d.y() * d.y();
}

// Tile bounds from (level, row, col) — mirrors gggs::GridIndex's accessors
// via the public gggs::levels specs (the GridIndex row/col constructor is
// private; same pattern as SurveyIndexBridge::indexedTiles, whose test pins
// the formulas against a coordinate-built GridIndex).
GeoRect tileBounds(int level, std::uint32_t row, std::uint32_t col)
{
  const auto & spec = gggs::levels[static_cast<std::size_t>(level)];
  const double row_d = static_cast<double>(row);
  const double col_d = static_cast<double>(col);
  GeoRect r;
  r.south = std::clamp(-96.0 + row_d * spec.grid_angular_span, -90.0, 90.0);
  r.north = std::clamp(-96.0 + (row_d + 1.0) * spec.grid_angular_span, -90.0, 90.0);
  const double lon_span = spec.gridLongitudinalSpan(row);
  r.west = -180.0 + col_d * lon_span;
  r.east = -180.0 + (col_d + 1.0) * lon_span;
  return r;
}

// Colormap one store tile's band 0 into an RGBA image (moved here from the
// window's retired one-shot loader): valid values span [lo, hi], NoData stays
// transparent. NoData is NaN (float stores) or 0 (uint16-backed sidescan
// composites — loadTile reads raw values without honoring file metadata).
QImage tileToImage(
  const marine_tiled_raster_store::TiledRasterTile<double> & tile,
  double lo, double hi, const std::vector<marine_colormap::Rgba8> & lut,
  bool zero_is_nodata)
{
  const auto & band = tile.band(0);
  const int rows = marine_tiled_raster_store::TiledRasterTile<double>::edge;
  const int cols = marine_tiled_raster_store::TiledRasterTile<double>::edge;
  QImage image(cols, rows, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  const double span = (hi > lo) ? (hi - lo) : 1.0;
  for (int r = 0; r < rows; ++r) {
    // Store row 0 is the SOUTH edge (GGGS rows grow northward); QImage row 0
    // draws at the TOP (north) of the target rect, so flip vertically.
    QRgb * out = reinterpret_cast<QRgb *>(image.scanLine(rows - 1 - r));
    for (int c = 0; c < cols; ++c) {
      const double v = band[static_cast<std::size_t>(r) * cols + c];
      if (std::isnan(v) || (zero_is_nodata && v == 0.0)) {
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

std::optional<marine_tiled_raster_store::TiledRasterTile<double>> loadOne(
  const std::string & path, int level)
{
  try {
    const int band_count = marine_tiled_raster_store::tileRasterCount(path);
    if (band_count < 1) {
      return std::nullopt;
    }
    return marine_tiled_raster_store::loadTile<double>(
      path, gggs::Level(static_cast<std::uint8_t>(level)),
      static_cast<std::size_t>(band_count));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

// Scan one directory of `<level>_<row>_<col>.tif` tiles into the ladder map.
void scanTileDir(
  const std::filesystem::path & dir,
  std::map<int, std::map<BasemapTileKey, BasemapTileRef>> & levels)
{
  std::error_code ec;
  for (const auto & entry : std::filesystem::directory_iterator(dir, ec)) {
    const auto name = entry.path().filename().string();
    if (entry.path().extension() != ".tif") {
      continue;
    }
    int level = -1;
    unsigned long row = 0;   // NOLINT(runtime/int) — strtoul contract
    unsigned long col = 0;   // NOLINT(runtime/int)
    try {
      std::size_t p1 = name.find('_');
      std::size_t p2 = (p1 == std::string::npos) ? std::string::npos :
        name.find('_', p1 + 1);
      if (p2 == std::string::npos) {
        continue;
      }
      level = std::stoi(name.substr(0, p1));
      row = std::stoul(name.substr(p1 + 1, p2 - p1 - 1));
      col = std::stoul(name.substr(p2 + 1));
    } catch (const std::exception &) {
      continue;
    }
    if (level < 0 || static_cast<std::size_t>(level) >= gggs::levels.size() ||
      row > std::numeric_limits<std::uint32_t>::max() ||
      col > std::numeric_limits<std::uint32_t>::max())
    {
      continue;   // junk name must not enter the ladder
    }
    const auto r = static_cast<std::uint32_t>(row);
    const auto c = static_cast<std::uint32_t>(col);
    levels[level][{r, c}] = {tileBounds(level, r, c), entry.path().string()};
  }
}

}  // namespace

BasemapLod::BasemapLod(QObject * parent)
: QObject(parent)
{
  connect(&worker_, &QFutureWatcher<LoadResult>::finished,
    this, &BasemapLod::onWorkerFinished);
}

BasemapLod::~BasemapLod()
{
  if (cancel_) {
    cancel_->store(true);
  }
  if (worker_.isRunning()) {
    worker_.waitForFinished();
  }
}

void BasemapLod::open(
  const std::string & dir, std::size_t palette_idx, bool zero_is_nodata)
{
  dir_ = dir;
  palette_idx_ = palette_idx;
  zero_is_nodata_ = zero_is_nodata;
  discovered_ = false;
  if (cancel_) {
    cancel_->store(true);   // a superseded pass stops mid-list
  }
  cancel_ = std::make_shared<std::atomic<bool>>(false);
  ++gen_;

  LoadJob job;
  job.gen = gen_;
  job.is_open = true;
  job.dir = dir;
  job.palette_idx = palette_idx;
  job.zero_is_nodata = zero_is_nodata;
  const auto cancel = cancel_;
  // The worker is a pure function of its arguments (camp ADR-0013's
  // snapshotted filter): safe to abandon via gen and to outlive `this`
  // briefly on supersede (setFuture drops the old future unwatched).
  worker_.setFuture(QtConcurrent::run([job, cancel]() {
      LoadResult res;
      res.gen = job.gen;
      res.is_open = true;

      // Discovery: the native level(s) from the directory itself, the
      // coarser ladder from the overviews/ sidecar (uma ADR-0011).
      decltype(res.discovery) native;
      scanTileDir(job.dir, native);
      res.discovery = native;
      scanTileDir(std::filesystem::path(job.dir) / "overviews", res.discovery);
      if (res.discovery.empty()) {
        res.note = QString(" (no store tiles under %1)")
        .arg(QString::fromStdString(job.dir));
        return res;
      }

      // Data extent: native-level tiles only — overview tiles pad to their
      // coarse grid cell and would balloon fit-to-extent (camp ADR-0013).
      // A mixed native directory unions across its levels.
      for (const auto & [level, tiles] : native) {
        (void)level;
        for (const auto & [key, ref] : tiles) {
          (void)key;
          if (!res.extent) {
            res.extent = ref.bounds;
          } else {
            res.extent->south = std::min(res.extent->south, ref.bounds.south);
            res.extent->west = std::min(res.extent->west, ref.bounds.west);
            res.extent->north = std::max(res.extent->north, ref.bounds.north);
            res.extent->east = std::max(res.extent->east, ref.bounds.east);
          }
        }
      }

      // Contrast range: sampled once per open from the level whose tile
      // count is closest to a hand-sized sample set, reused across every
      // level — the overview fold is MEAN (uma ADR-0011), so overview
      // values are contained in the fine range and colours stay stable
      // across LOD switches.
      int sample_level = res.discovery.begin()->first;
      std::size_t best_gap = std::numeric_limits<std::size_t>::max();
      for (const auto & [level, tiles] : res.discovery) {
        const std::size_t gap = (tiles.size() > 32) ?
        tiles.size() - 32 : 32 - tiles.size();
        if (gap < best_gap) {
          best_gap = gap;
          sample_level = level;
        }
      }
      std::vector<double> samples;
      {
        const auto & tiles = res.discovery[sample_level];
        constexpr int kEdge = marine_tiled_raster_store::TiledRasterTile<double>::edge;
        const std::size_t n_tiles = std::min(tiles.size(), kMaxRangeSampleTiles);
        const std::size_t total_cells = n_tiles *
        static_cast<std::size_t>(kEdge) * static_cast<std::size_t>(kEdge);
        const std::size_t stride =
        std::max<std::size_t>(1, total_cells / kMaxRangeSamples);
        std::size_t used = 0;
        for (const auto & [key, ref] : tiles) {
          (void)key;
          if (used++ >= n_tiles || (cancel && cancel->load())) {
            break;
          }
          const auto tile = loadOne(ref.path, sample_level);
          if (!tile) {
            continue;
          }
          const auto & band = tile->band(0);
          for (std::size_t i = 0; i < band.size(); i += stride) {
            const double v = band[i];
            if (!std::isnan(v) && !(job.zero_is_nodata && v == 0.0)) {
              samples.push_back(v);
            }
          }
        }
      }
      if (samples.empty()) {
        res.note = " (store tiles hold no valid values)";
        return res;
      }
      const auto [lo, hi] = robust_range(samples);
      res.range_lo = lo;
      res.range_hi = hi;
      res.lut = marine_colormap::bake_lut(
        marine_colormap::palette(job.palette_idx),
        marine_colormap::TransferParams{}, 256);

      // First pixels: the coarsest level (the permanent cheap backdrop),
      // capped like any kick; demand loads refine from here.
      const int coarsest = res.discovery.begin()->first;
      std::size_t loaded = 0;
      for (const auto & [key, ref] : res.discovery[coarsest]) {
        if (loaded >= kMaxTilesPerKick || (cancel && cancel->load())) {
          break;
        }
        const auto tile = loadOne(ref.path, coarsest);
        if (!tile) {
          continue;
        }
        OverviewTile ot;
        ot.image = tileToImage(
          *tile, lo, hi, res.lut, job.zero_is_nodata);
        ot.south = ref.bounds.south;
        ot.west = ref.bounds.west;
        ot.north = ref.bounds.north;
        ot.east = ref.bounds.east;
        res.level = coarsest;
        res.tiles.emplace_back(key, std::move(ot));
        ++loaded;
      }
      std::size_t ladder_tiles = 0;
      for (const auto & [level, tiles] : res.discovery) {
        (void)level;
        ladder_tiles += tiles.size();
      }
      res.note = QString(" (%1 levels, %2 tiles, %3–%4)")
      .arg(res.discovery.size()).arg(ladder_tiles)
      .arg(lo, 0, 'f', 1).arg(hi, 0, 'f', 1);
      return res;
    }));
}

void BasemapLod::viewChanged(
  const GeoRect & viewport, double ground_metres_per_pixel)
{
  viewport_ = viewport;
  ground_mpp_ = ground_metres_per_pixel;
  have_viewport_ = true;
  kickIfNeeded();
}

std::vector<OverviewTile> BasemapLod::renderTiles() const
{
  // Paint order (camp ADR-0013 progressive refinement): non-selected levels
  // coarse->fine as the backdrop, the selected level last (on top).
  std::vector<OverviewTile> out;
  std::size_t total = 0;
  for (const auto & [level, tiles] : resident_) {
    (void)level;
    total += tiles.size();
  }
  out.reserve(total);
  for (const auto & [level, tiles] : resident_) {
    if (level == selected_level_) {
      continue;
    }
    for (const auto & [key, tile] : tiles) {
      (void)key;
      out.push_back(tile);
    }
  }
  const auto sel = resident_.find(selected_level_);
  if (sel != resident_.end()) {
    for (const auto & [key, tile] : sel->second) {
      (void)key;
      out.push_back(tile);
    }
  }
  return out;
}

std::vector<std::pair<BasemapLod::TileKey, BasemapLod::TileRef>>
BasemapLod::missingVisible(int level) const
{
  std::vector<std::pair<TileKey, TileRef>> missing;
  const auto refs = level_refs_.find(level);
  if (refs == level_refs_.end() || !have_viewport_) {
    return missing;
  }
  const auto resident = resident_.find(level);
  for (const auto & [key, ref] : refs->second) {
    if (!intersects(ref.bounds, viewport_)) {
      continue;
    }
    if (resident != resident_.end() && resident->second.count(key)) {
      continue;
    }
    missing.push_back({key, ref});
  }
  // Centre-out, so the middle of the view refines first.
  const QPointF c = centreOf(viewport_);
  std::sort(
    missing.begin(), missing.end(),
    [&c](const auto & a, const auto & b) {
      return centreDist2(a.second.bounds, c) < centreDist2(b.second.bounds, c);
    });
  if (missing.size() > kMaxTilesPerKick) {
    missing.resize(kMaxTilesPerKick);
  }
  return missing;
}

void BasemapLod::kickIfNeeded()
{
  if (!discovered_ || worker_.isRunning() || !have_viewport_) {
    return;
  }
  std::vector<int> ladder;
  ladder.reserve(level_refs_.size());
  for (const auto & [level, tiles] : level_refs_) {
    (void)tiles;
    ladder.push_back(level);
  }
  selected_level_ = selectLodLevel(ground_mpp_, ladder);
  if (selected_level_ < 0) {
    return;
  }
  auto missing = missingVisible(selected_level_);
  if (missing.empty()) {
    releaseStaleLevels();
    return;
  }
  LoadJob job;
  job.gen = gen_;
  job.level = selected_level_;
  job.dir = dir_;
  job.palette_idx = palette_idx_;
  job.zero_is_nodata = zero_is_nodata_;
  job.tiles = std::move(missing);
  const auto cancel = cancel_;
  const auto lut = lut_;
  const double lo = range_lo_;
  const double hi = range_hi_;
  worker_.setFuture(QtConcurrent::run([job, cancel, lut, lo, hi]() {
      LoadResult res;
      res.gen = job.gen;
      res.level = job.level;
      for (const auto & [key, ref] : job.tiles) {
        if (cancel && cancel->load()) {
          break;
        }
        const auto tile = loadOne(ref.path, job.level);
        if (!tile) {
          continue;
        }
        OverviewTile ot;
        ot.image = tileToImage(*tile, lo, hi, lut, job.zero_is_nodata);
        ot.south = ref.bounds.south;
        ot.west = ref.bounds.west;
        ot.north = ref.bounds.north;
        ot.east = ref.bounds.east;
        res.tiles.emplace_back(key, std::move(ot));
      }
      return res;
    }));
}

void BasemapLod::onWorkerFinished()
{
  LoadResult res = worker_.result();
  if (res.gen != gen_) {
    kickIfNeeded();   // a superseded pass's result is dead; serve the live one
    return;
  }
  if (res.is_open) {
    level_refs_ = std::move(res.discovery);
    data_extent_ = res.extent;
    range_lo_ = res.range_lo;
    range_hi_ = res.range_hi;
    lut_ = std::move(res.lut);
    note_ = res.note;
    resident_.clear();
    selected_level_ = -1;
    discovered_ = true;
    if (res.level >= 0) {
      auto & bucket = resident_[res.level];
      for (auto & [key, tile] : res.tiles) {
        bucket[key] = std::move(tile);
      }
    }
    Q_EMIT opened();
    Q_EMIT tilesChanged();
    kickIfNeeded();
    return;
  }
  if (!res.tiles.empty() && res.level >= 0) {
    auto & bucket = resident_[res.level];
    for (auto & [key, tile] : res.tiles) {
      bucket[key] = std::move(tile);
    }
    enforceResidentCap();
    Q_EMIT tilesChanged();
  }
  kickIfNeeded();   // continue capped batches / follow a moved view
}

void BasemapLod::releaseStaleLevels()
{
  // The selected level's visible set is complete (missingVisible was empty):
  // drop other levels except the coarsest, the permanent backdrop.
  if (level_refs_.empty()) {
    return;
  }
  const int coarsest = level_refs_.begin()->first;
  bool released = false;
  for (auto it = resident_.begin(); it != resident_.end(); ) {
    if (it->first != selected_level_ && it->first != coarsest) {
      it = resident_.erase(it);
      released = true;
    } else {
      ++it;
    }
  }
  if (released) {
    Q_EMIT tilesChanged();
  }
}

void BasemapLod::enforceResidentCap()
{
  std::size_t total = 0;
  for (const auto & [level, tiles] : resident_) {
    (void)level;
    total += tiles.size();
  }
  if (total <= kResidentCapTiles || !have_viewport_) {
    return;
  }
  // Evict farthest-from-view first, never the coarsest backdrop, and the
  // selected level only after everything else.
  const int coarsest = level_refs_.empty() ? -1 : level_refs_.begin()->first;
  const QPointF c = centreOf(viewport_);
  struct Candidate
  {
    int level;
    TileKey key;
    double d2;
  };
  std::vector<Candidate> candidates;
  for (const auto & [level, tiles] : resident_) {
    if (level == coarsest) {
      continue;
    }
    for (const auto & [key, tile] : tiles) {
      const GeoRect r{tile.south, tile.west, tile.north, tile.east};
      // Non-selected levels sort past any distance: they evict first.
      const double bias = (level == selected_level_) ? 0.0 : 1e12;
      candidates.push_back({level, key, centreDist2(r, c) + bias});
    }
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {return a.d2 > b.d2;});
  for (const auto & cand : candidates) {
    if (total <= kResidentCapTiles) {
      break;
    }
    resident_[cand.level].erase(cand.key);
    --total;
  }
}

}  // namespace marine_perception_tools
