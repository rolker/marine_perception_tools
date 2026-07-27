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

#include "session_index_io.hpp"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <utility>
#include <vector>

namespace marine_perception_tools
{

namespace
{

constexpr char kMagic[4] = {'S', 'S', 'V', 'C'};
// Bump on ANY layout change of the structs written below — an old cache then
// simply misses and the bag rescans.
constexpr std::uint32_t kVersion = 1;

// Field-by-field primitive I/O: immune to struct padding and layout drift
// (a reinterpret_cast dump would silently corrupt across compilers/versions).
template<typename T>
void put(std::ostream & os, const T & v)
{
  os.write(reinterpret_cast<const char *>(&v), sizeof(T));
}

template<typename T>
bool get(std::istream & is, T & v)
{
  is.read(reinterpret_cast<char *>(&v), sizeof(T));
  return static_cast<bool>(is);
}

void putString(std::ostream & os, const std::string & s)
{
  put(os, static_cast<std::uint64_t>(s.size()));
  os.write(s.data(), static_cast<std::streamsize>(s.size()));
}

bool getString(std::istream & is, std::string & s)
{
  std::uint64_t n = 0;
  if (!get(is, n) || n > (1u << 20)) {   // a bag uri is never a megabyte
    return false;
  }
  s.resize(n);
  is.read(s.data(), static_cast<std::streamsize>(n));
  return static_cast<bool>(is);
}

void putPing(std::ostream & os, const SidescanPing & p)
{
  put(os, static_cast<std::int32_t>(p.channel));
  put(os, p.stamp_s);
  put(os, p.stamp_ns);
  put(os, p.cumulative_distance_m);
  put(os, static_cast<std::uint8_t>(p.has_pose));
  put(os, p.geometry.sensor_x);
  put(os, p.geometry.sensor_y);
  put(os, p.geometry.yaw);
  put(os, p.geometry.sample0);
  put(os, p.geometry.metres_per_sample);
  put(os, p.geometry.altitude);
  put(os, static_cast<std::int32_t>(p.geometry.lateral_sign));
  put(os, p.samples_per_beam);
  put(os, p.sound_speed);
  put(os, p.sample_rate);
}

bool getPing(std::istream & is, SidescanPing & p)
{
  std::int32_t channel = 0;
  std::uint8_t has_pose = 0;
  std::int32_t lateral_sign = 0;
  const bool ok = get(is, channel) && get(is, p.stamp_s) && get(is, p.stamp_ns) &&
    get(is, p.cumulative_distance_m) && get(is, has_pose) &&
    get(is, p.geometry.sensor_x) && get(is, p.geometry.sensor_y) &&
    get(is, p.geometry.yaw) && get(is, p.geometry.sample0) &&
    get(is, p.geometry.metres_per_sample) && get(is, p.geometry.altitude) &&
    get(is, lateral_sign) && get(is, p.samples_per_beam) &&
    get(is, p.sound_speed) && get(is, p.sample_rate);
  if (!ok) {
    return false;
  }
  // A readable-but-corrupt cache can carry an out-of-range channel; casting it
  // to SidescanChannel and later using it to index a kNumSidescanChannels-element
  // array is an out-of-bounds access. Reject the cache so the bag re-indexes.
  if (channel < 0 || channel >= kNumSidescanChannels) {
    return false;
  }
  p.channel = static_cast<SidescanChannel>(channel);
  p.has_pose = has_pose != 0;
  p.geometry.lateral_sign = lateral_sign;
  return true;
}

void putMbes(std::ostream & os, const MbesPing & p)
{
  put(os, p.stamp_s);
  put(os, p.stamp_ns);
  put(os, p.cumulative_distance_m);
  put(os, static_cast<std::uint8_t>(p.tf_ok));
  put(os, static_cast<std::uint8_t>(p.has_pose));
  put(os, p.tx);
  put(os, p.ty);
  put(os, p.tz);
  put(os, p.qx);
  put(os, p.qy);
  put(os, p.qz);
  put(os, p.qw);
}

bool getMbes(std::istream & is, MbesPing & p)
{
  std::uint8_t tf_ok = 0;
  std::uint8_t has_pose = 0;
  const bool ok = get(is, p.stamp_s) && get(is, p.stamp_ns) &&
    get(is, p.cumulative_distance_m) && get(is, tf_ok) && get(is, has_pose) &&
    get(is, p.tx) && get(is, p.ty) && get(is, p.tz) &&
    get(is, p.qx) && get(is, p.qy) && get(is, p.qz) && get(is, p.qw);
  if (!ok) {
    return false;
  }
  p.tf_ok = tf_ok != 0;
  p.has_pose = has_pose != 0;
  return true;
}

}  // namespace

BagIdentity bagIdentity(const std::string & bag_uri)
{
  BagIdentity id;
  id.bag_uri = bag_uri;
  std::error_code ec;
  for (const auto & entry : std::filesystem::directory_iterator(bag_uri, ec)) {
    std::error_code fec;
    if (!entry.is_regular_file(fec)) {
      continue;
    }
    id.size_bytes += entry.file_size(fec);
    const auto mtime = entry.last_write_time(fec);
    if (!fec) {
      const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        mtime.time_since_epoch()).count();
      id.mtime_ns = std::max<std::int64_t>(id.mtime_ns, ns);
    }
  }
  return id;
}

std::string cachePathFor(const std::string & cache_dir, const std::string & bag_uri)
{
  // FNV-1a over the uri; collisions are resolved by the in-file identity check.
  std::uint64_t h = 14695981039346656037ULL;
  for (const char c : bag_uri) {
    h ^= static_cast<unsigned char>(c);
    h *= 1099511628211ULL;
  }
  char name[32];
  std::snprintf(name, sizeof(name), "%016llx.ssvc",
    static_cast<unsigned long long>(h));  // NOLINT(runtime/int) -- %llx portability
  return (std::filesystem::path(cache_dir) / name).string();
}

std::string defaultCacheDir()
{
  const char * xdg = std::getenv("XDG_CACHE_HOME");
  if (xdg != nullptr && xdg[0] != '\0') {
    return (std::filesystem::path(xdg) / "survey_explorer").string();
  }
  const char * home = std::getenv("HOME");
  if (home != nullptr && home[0] != '\0') {
    return (std::filesystem::path(home) / ".cache" / "survey_explorer").string();
  }
  return {};
}

bool saveSessionIndex(
  const std::string & path, const BagIdentity & identity, const SessionIndex & index)
{
  if (!index.complete || identity.size_bytes == 0) {
    return false;   // never cache a partial index or an unverifiable bag
  }
  std::error_code ec;
  std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
  // Per-writer temp name: two processes (an interactive explorer + a
  // concurrent --warm-cache) may cache the same bag into the same dir; a
  // shared .tmp would interleave (review round-2 finding).
  const std::string tmp = path + ".tmp." + std::to_string(::getpid());
  {
    std::ofstream os(tmp, std::ios::binary | std::ios::trunc);
    if (!os) {
      return false;
    }
    os.write(kMagic, sizeof(kMagic));
    put(os, kVersion);
    putString(os, identity.bag_uri);
    put(os, identity.size_bytes);
    put(os, identity.mtime_ns);
    put(os, index.total_distance_m);
    put(os, static_cast<std::uint64_t>(index.poses_resolved));
    put(os, static_cast<std::uint64_t>(index.poses_skipped));
    put(os, static_cast<std::uint64_t>(index.decode_errors));
    put(os, static_cast<std::uint8_t>(index.has_geo_reference));
    put(os, static_cast<std::uint8_t>(index.used_nadir_depth));
    put(os, index.geo_tx);
    put(os, index.geo_ty);
    put(os, index.geo_tz);
    put(os, index.geo_qx);
    put(os, index.geo_qy);
    put(os, index.geo_qz);
    put(os, index.geo_qw);
    put(os, static_cast<std::uint64_t>(index.pings.size()));
    for (const auto & p : index.pings) {
      putPing(os, p);
    }
    put(os, static_cast<std::uint64_t>(index.mbes_pings.size()));
    for (const auto & p : index.mbes_pings) {
      putMbes(os, p);
    }
    if (!os) {
      return false;
    }
  }
  std::filesystem::rename(tmp, path, ec);
  if (ec) {
    std::filesystem::remove(tmp, ec);
    return false;
  }
  return true;
}

std::optional<SessionIndex> loadSessionIndex(
  const std::string & path, const BagIdentity & identity)
{
  std::ifstream is(path, std::ios::binary);
  if (!is) {
    return std::nullopt;
  }
  char magic[4] = {};
  is.read(magic, sizeof(magic));
  std::uint32_t version = 0;
  if (!is || !std::equal(magic, magic + 4, kMagic) || !get(is, version) ||
    version != kVersion)
  {
    return std::nullopt;
  }
  std::string uri;
  std::uint64_t size_bytes = 0;
  std::int64_t mtime_ns = 0;
  if (!getString(is, uri) || !get(is, size_bytes) || !get(is, mtime_ns)) {
    return std::nullopt;
  }
  if (uri != identity.bag_uri || size_bytes != identity.size_bytes ||
    mtime_ns != identity.mtime_ns || identity.size_bytes == 0)
  {
    return std::nullopt;   // the bag changed (or can't be verified): rescan
  }

  SessionIndex index;
  std::uint64_t poses_resolved = 0;
  std::uint64_t poses_skipped = 0;
  std::uint64_t decode_errors = 0;
  std::uint8_t has_geo = 0;
  std::uint8_t used_nadir = 0;
  if (!get(is, index.total_distance_m) || !get(is, poses_resolved) ||
    !get(is, poses_skipped) || !get(is, decode_errors) || !get(is, has_geo) ||
    !get(is, used_nadir) || !get(is, index.geo_tx) || !get(is, index.geo_ty) ||
    !get(is, index.geo_tz) || !get(is, index.geo_qx) || !get(is, index.geo_qy) ||
    !get(is, index.geo_qz) || !get(is, index.geo_qw))
  {
    return std::nullopt;
  }
  index.poses_resolved = poses_resolved;
  index.poses_skipped = poses_skipped;
  index.decode_errors = decode_errors;
  index.has_geo_reference = has_geo != 0;
  index.used_nadir_depth = used_nadir != 0;

  std::uint64_t n = 0;
  if (!get(is, n) || n > (1ULL << 27)) {
    return std::nullopt;
  }
  index.pings.resize(n);
  for (auto & p : index.pings) {
    if (!getPing(is, p)) {
      return std::nullopt;   // truncated
    }
  }
  if (!get(is, n) || n > (1ULL << 27)) {
    return std::nullopt;
  }
  index.mbes_pings.resize(n);
  for (auto & p : index.mbes_pings) {
    if (!getMbes(is, p)) {
      return std::nullopt;
    }
  }
  index.complete = true;
  return index;
}

}  // namespace marine_perception_tools
