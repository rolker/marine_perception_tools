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

#ifndef WORLD_LAYOUT_HPP_
#define WORLD_LAYOUT_HPP_

// Where the world collection lives and what it holds (#40).
//
// The explorer's store paths predated `uma-ADR-0010` D3 and named a
// `bathymetry/survey` layer that exists in no collection. The taxonomy is
// theme-then-provenance: `depths/{chart,reference,draft,processed}` and an
// `imagery/` theme carrying the sidescan and MBES-backscatter stores. This
// header is the one place those names appear.
//
// Header-only and free of Qt, ROS and GDAL, so the layout contract unit-tests
// against a temporary directory with no display.

#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace marine_perception_tools
{

/// The conventional root of the world collection: `$HOME/data/world`.
/// Returns an empty path when `$HOME` is unset, never a relative path — a
/// caller must not silently resolve the collection against the CWD.
inline std::filesystem::path defaultWorldRoot()
{
  const char * home = std::getenv("HOME");
  if (home == nullptr || *home == '\0') {
    return {};
  }
  return std::filesystem::path(home) / "data" / "world";
}

/// The survey index at the top of a collection — a sibling of the layer
/// themes, not a member of one: the index records where the sensor looked,
/// the stores what was accepted, and either is regenerable without the other.
inline std::filesystem::path worldIndexPath(const std::filesystem::path & root)
{
  return root.empty() ? std::filesystem::path{} : root / "survey_index.db";
}

/// The layer a fresh open shows: the authoritative off-boat depth product.
inline std::filesystem::path defaultStoresDir(const std::filesystem::path & root)
{
  return root.empty() ? std::filesystem::path{} : root / "depths" / "processed";
}

/// The default stores directory for an index the operator named on the command
/// line, resolved so a BARE RELATIVE filename still works.
///
/// `std::filesystem::path("survey_index.db").parent_path()` is empty, and
/// defaultStoresDir() returns {} for an empty root by design — so
/// `survey_explorer --index survey_index.db`, run from the directory holding
/// it, opened with no basemap and no explanation. The "never a relative path"
/// rule this file states is about $HOME, not about a path the operator typed.
/// Falls back to the unresolved parent when the filesystem cannot answer.
inline std::filesystem::path defaultStoresDirForIndex(
  const std::filesystem::path & index_path)
{
  std::error_code ec;
  const auto resolved = std::filesystem::absolute(index_path, ec);
  return defaultStoresDir(ec ? index_path.parent_path() : resolved.parent_path());
}

/// Preferred basemap layers as paths relative to the collection root, in the
/// order an operator reaches for them. Discovery still lists anything else it
/// finds; this decides what comes first and what opens.
///
/// `imagery/backscatter` appears twice on purpose, and `survey` is the CURRENT
/// name, not an alias awaiting a rename. uma-ADR-0007 A.2 collapsed that
/// store's `draft` / `processed` overlay to a single `survey` layer, and
/// uma-ADR-0010 D3 puts the provenance names (`chart` / `reference` / `draft` /
/// `processed`) under `depths/` only — imagery layer names are not renamed to
/// match the depth theme. So `imagery/backscatter/survey` is what a populated
/// store holds today; `imagery/backscatter/processed` is kept below it, after
/// the current name, for stores written before that collapse.
inline const std::vector<std::string> & preferredLayerPaths()
{
  static const std::vector<std::string> kPaths = {
    "depths/processed",
    "imagery/backscatter/survey",
    "imagery/backscatter/processed",
    "imagery/sidescan/processed",
    "depths/reference",
    "depths/chart",
    "imagery/sidescan/tier1",
  };
  return kPaths;
}

}  // namespace marine_perception_tools

#endif  // WORLD_LAYOUT_HPP_
