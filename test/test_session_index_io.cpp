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

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "session_index_io.hpp"

namespace
{

using marine_perception_tools::BagIdentity;
using marine_perception_tools::loadSessionIndex;
using marine_perception_tools::MbesPing;
using marine_perception_tools::saveSessionIndex;
using marine_perception_tools::SessionIndex;
using marine_perception_tools::SidescanChannel;
using marine_perception_tools::SidescanPing;

BagIdentity identity()
{
  BagIdentity id;
  id.bag_uri = "/data/bag_a";
  id.size_bytes = 123456789;
  id.mtime_ns = 1780000000000000000LL;
  return id;
}

SessionIndex makeIndex()
{
  SessionIndex index;
  SidescanPing ping;
  ping.channel = SidescanChannel::Starboard;
  ping.stamp_s = 1234.5;
  ping.stamp_ns = 1234500000000LL;
  ping.cumulative_distance_m = 87.25;
  ping.has_pose = true;
  ping.geometry.sensor_x = 10.5;
  ping.geometry.sensor_y = -3.25;
  ping.geometry.yaw = 0.75;
  ping.geometry.sample0 = 14;
  ping.geometry.metres_per_sample = 0.0171;
  ping.geometry.altitude = 4.5;
  ping.geometry.lateral_sign = -1;
  ping.samples_per_beam = 2035;
  ping.sound_speed = 1481.0;
  ping.sample_rate = 43900.0;
  index.pings.push_back(ping);
  MbesPing mbes;
  mbes.stamp_ns = 1234600000000LL;
  mbes.cumulative_distance_m = 88.0;
  mbes.tf_ok = true;
  mbes.has_pose = true;
  mbes.tx = 1.0;
  mbes.qz = 0.5;
  mbes.qw = 0.866;
  index.mbes_pings.push_back(mbes);
  index.total_distance_m = 19473.7;
  index.poses_resolved = 42;
  index.has_geo_reference = true;
  index.geo_tx = 1.5e6;
  index.geo_qw = 0.99;
  index.complete = true;
  return index;
}

std::string tempCachePath(const char * name)
{
  return std::string(::testing::TempDir()) + "/" + name + ".ssvc";
}

TEST(SessionIndexIo, RoundTripPreservesEverything)
{
  const auto path = tempCachePath("roundtrip");
  std::remove(path.c_str());
  const auto index = makeIndex();
  ASSERT_TRUE(saveSessionIndex(path, identity(), index));
  const auto loaded = loadSessionIndex(path, identity());
  ASSERT_TRUE(loaded.has_value());
  ASSERT_EQ(loaded->pings.size(), 1u);
  const auto & p = loaded->pings[0];
  EXPECT_EQ(p.channel, SidescanChannel::Starboard);
  EXPECT_EQ(p.stamp_ns, 1234500000000LL);
  EXPECT_DOUBLE_EQ(p.cumulative_distance_m, 87.25);
  EXPECT_TRUE(p.has_pose);
  EXPECT_DOUBLE_EQ(p.geometry.metres_per_sample, 0.0171);
  EXPECT_EQ(p.geometry.lateral_sign, -1);
  EXPECT_EQ(p.samples_per_beam, 2035u);
  ASSERT_EQ(loaded->mbes_pings.size(), 1u);
  EXPECT_DOUBLE_EQ(loaded->mbes_pings[0].qz, 0.5);
  EXPECT_DOUBLE_EQ(loaded->total_distance_m, 19473.7);
  EXPECT_EQ(loaded->poses_resolved, 42u);
  EXPECT_TRUE(loaded->has_geo_reference);
  EXPECT_DOUBLE_EQ(loaded->geo_tx, 1.5e6);
  EXPECT_TRUE(loaded->complete);
  std::remove(path.c_str());
}

TEST(SessionIndexIo, ChangedBagInvalidatesTheCache)
{
  const auto path = tempCachePath("identity");
  std::remove(path.c_str());
  ASSERT_TRUE(saveSessionIndex(path, identity(), makeIndex()));
  auto grown = identity();
  grown.size_bytes += 1;   // the bag grew: stale cache must miss
  EXPECT_FALSE(loadSessionIndex(path, grown).has_value());
  auto touched = identity();
  touched.mtime_ns += 1;
  EXPECT_FALSE(loadSessionIndex(path, touched).has_value());
  auto moved = identity();
  moved.bag_uri = "/data/bag_b";
  EXPECT_FALSE(loadSessionIndex(path, moved).has_value());
  EXPECT_TRUE(loadSessionIndex(path, identity()).has_value());
  std::remove(path.c_str());
}

TEST(SessionIndexIo, TruncatedFileIsRejected)
{
  const auto path = tempCachePath("truncated");
  std::remove(path.c_str());
  ASSERT_TRUE(saveSessionIndex(path, identity(), makeIndex()));
  // Chop the tail off: the per-ping reads must fail, not return junk.
  std::ifstream in(path, std::ios::binary);
  std::string bytes((std::istreambuf_iterator<char>(in)), {});
  in.close();
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out.write(bytes.data(), static_cast<std::streamsize>(bytes.size() - 20));
  out.close();
  EXPECT_FALSE(loadSessionIndex(path, identity()).has_value());
  std::remove(path.c_str());
}

TEST(SessionIndexIo, PartialIndexIsNeverCached)
{
  const auto path = tempCachePath("partial");
  std::remove(path.c_str());
  auto index = makeIndex();
  index.complete = false;
  EXPECT_FALSE(saveSessionIndex(path, identity(), index));
  EXPECT_FALSE(loadSessionIndex(path, identity()).has_value());
}

TEST(SessionIndexIo, UnverifiableBagIsNeverCached)
{
  const auto path = tempCachePath("unverifiable");
  std::remove(path.c_str());
  auto id = identity();
  id.size_bytes = 0;   // bagIdentity() of an unreadable directory
  EXPECT_FALSE(saveSessionIndex(path, id, makeIndex()));
}

}  // namespace
