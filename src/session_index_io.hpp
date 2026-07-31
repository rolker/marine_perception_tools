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

#ifndef SESSION_INDEX_IO_HPP_
#define SESSION_INDEX_IO_HPP_

// The bag-index cache (#24): a SidescanBagSession's SessionIndex is the
// entire product of the whole-bag metadata scan (buildIndex) — per-ping
// stamps, poses, distances, the geo anchor; never the sample data. It is
// deterministic per bag, so persisting it turns every later open of that bag
// (time-bar cues, timeline clicks, jump-to-pass) from a multi-second scan
// into a file read. Caches live OUTSIDE the bag directories (bags are
// data-of-record) and are keyed by the bag's identity (uri + total byte size
// + newest mtime, the same identity fields the survey index's bags table
// tracks) — any mismatch, version bump, or truncation invalidates the cache
// and the caller falls back to a fresh scan. Qt-free.

#include <cstdint>
#include <optional>
#include <string>

#include "sidescan_bag_session.hpp"

namespace marine_perception_tools
{

struct BagIdentity
{
  std::string bag_uri;         // as opened (path normalization is the caller's)
  std::uint64_t size_bytes = 0;   // sum of the bag directory's file sizes
  std::int64_t mtime_ns = 0;      // newest file mtime in the bag directory
};

// Identity of the bag at `bag_uri` right now (scans the directory's files).
// size 0 / mtime 0 when the directory is unreadable — such an identity never
// validates a cache, so a broken path degrades to the normal scan path.
BagIdentity bagIdentity(const std::string & bag_uri);

// The cache file for this bag under `cache_dir` (FNV-1a of the uri; the full
// identity is verified from the file's own header on load).
std::string cachePathFor(const std::string & cache_dir, const std::string & bag_uri);

// $XDG_CACHE_HOME/survey_explorer (or ~/.cache/survey_explorer): derived
// regenerable data, deliberately away from both the bags and the stores.
std::string defaultCacheDir();

// Write `index` (which must be complete) atomically (temp file + rename).
// Returns false on any I/O failure — the cache is best-effort.
bool saveSessionIndex(
  const std::string & path, const BagIdentity & identity, const SessionIndex & index);

// Load and validate: nullopt on missing file, magic/version mismatch,
// identity mismatch, or truncation.
std::optional<SessionIndex> loadSessionIndex(
  const std::string & path, const BagIdentity & identity);

}  // namespace marine_perception_tools

#endif  // SESSION_INDEX_IO_HPP_
