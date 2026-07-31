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

#include <vector>

#include "mbes_pass_loader.hpp"

namespace
{

using marine_perception_tools::CloudPassInfo;
using marine_perception_tools::load_cloud_passes;

TEST(MbesPassLoader, EmptyInputYieldsEmptyOutcome)
{
  const auto out = load_cloud_passes({});
  EXPECT_TRUE(out.pass_clouds.empty());
  EXPECT_TRUE(out.sounding_counts.empty());
  EXPECT_TRUE(out.notes.isEmpty());
  EXPECT_EQ(out.skipped_passes, 0);
  EXPECT_EQ(out.skipped_pings, 0);
}

TEST(MbesPassLoader, MissingBagCostsOnlyItsOwnPass)
{
  CloudPassInfo pass;
  pass.bag_path = "/nonexistent/bag_dir";
  pass.label = "2026-06-15 15:10:00  (bag_a)";
  pass.t_start_ns = 1000;
  pass.t_end_ns = 2000;
  const auto out = load_cloud_passes({pass, pass});

  // Both passes fail (same missing bag) — one slot, one note each; the loader
  // never throws and the per-pass structure stays index-aligned with the input.
  ASSERT_EQ(out.pass_clouds.size(), 2u);
  ASSERT_EQ(out.sounding_counts.size(), 2u);
  EXPECT_TRUE(out.pass_clouds[0].empty());
  EXPECT_TRUE(out.pass_clouds[1].empty());
  EXPECT_EQ(out.sounding_counts[0], 0);
  EXPECT_EQ(out.sounding_counts[1], 0);
  EXPECT_EQ(out.skipped_passes, 2);
  ASSERT_EQ(out.notes.size(), 2);
  EXPECT_TRUE(out.notes[0].contains("bag read failed"));
  EXPECT_TRUE(out.notes[0].contains("2026-06-15 15:10:00"));
}

}  // namespace
