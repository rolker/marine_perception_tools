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

// The world-collection layout contract (#40): the taxonomy names the explorer
// reaches for, and the rules that keep an unset or unconventional environment
// from resolving the collection somewhere surprising.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <string>

#include "world_layout.hpp"

using marine_perception_tools::defaultStoresDir;
using marine_perception_tools::defaultWorldRoot;
using marine_perception_tools::preferredLayerPaths;
using marine_perception_tools::worldIndexPath;

TEST(WorldLayout, RootIsUnderHome)
{
  ASSERT_EQ(0, ::setenv("HOME", "/home/tester", 1));
  EXPECT_EQ("/home/tester/data/world", defaultWorldRoot().string());
}

// An unset HOME must not degrade to a relative path: "data/world" would
// resolve against whatever directory the app happened to be launched from.
TEST(WorldLayout, RootIsEmptyWithoutHome)
{
  ASSERT_EQ(0, ::unsetenv("HOME"));
  EXPECT_TRUE(defaultWorldRoot().empty());
  EXPECT_TRUE(worldIndexPath(defaultWorldRoot()).empty());
  EXPECT_TRUE(defaultStoresDir(defaultWorldRoot()).empty());
}

// The index is a sibling of the layer themes, not a member of one.
TEST(WorldLayout, IndexSitsAtTheCollectionRoot)
{
  EXPECT_EQ("/w/survey_index.db", worldIndexPath("/w").string());
}

TEST(WorldLayout, DefaultLayerIsTheProcessedDepthProduct)
{
  EXPECT_EQ("/w/depths/processed", defaultStoresDir("/w").string());
}

// The opening layer must be the first thing discovery offers, or the combo
// would present one layer while the basemap showed another.
TEST(WorldLayout, DefaultLayerLeadsThePreferredOrder)
{
  ASSERT_FALSE(preferredLayerPaths().empty());
  EXPECT_EQ("depths/processed", preferredLayerPaths().front());
}

// uma-ADR-0010 D3 names the depth provenance layers; none of the pre-#40
// paths existed in any collection.
TEST(WorldLayout, PreferredPathsAreTheTaxonomyNotTheOldGuesses)
{
  const auto & paths = preferredLayerPaths();
  const auto has = [&paths](const std::string & p) {
      return std::find(paths.begin(), paths.end(), p) != paths.end();
    };
  EXPECT_TRUE(has("depths/processed"));
  EXPECT_TRUE(has("depths/reference"));
  EXPECT_TRUE(has("depths/chart"));
  EXPECT_TRUE(has("imagery/sidescan/processed"));
  EXPECT_FALSE(has("bathymetry/survey"));
  EXPECT_FALSE(has("bathymetry/reference"));
  EXPECT_FALSE(has("backscatter/survey"));
  EXPECT_FALSE(has("sidescan/processed"));
}

// `survey` is the MBES backscatter store's current layer name — uma-ADR-0007
// A.2 collapsed its `draft` / `processed` overlay to that single layer, and
// uma-ADR-0010 D3 scopes the provenance names to `depths/`. So `survey` is
// preferred, and `processed` stays listed below it only for stores written
// before that collapse.
TEST(WorldLayout, BackscatterPrefersTheCurrentSurveyNameOverThePreCollapseOne)
{
  const auto & paths = preferredLayerPaths();
  const auto at = [&paths](const std::string & p) {
      return static_cast<std::size_t>(
        std::find(paths.begin(), paths.end(), p) - paths.begin());
    };
  ASSERT_LT(at("imagery/backscatter/processed"), paths.size());
  ASSERT_LT(at("imagery/backscatter/survey"), paths.size());
  EXPECT_LT(
    at("imagery/backscatter/survey"), at("imagery/backscatter/processed"));
}

// No duplicates: a repeated path would list the same layer twice in the combo.
TEST(WorldLayout, PreferredPathsAreUnique)
{
  auto paths = preferredLayerPaths();
  std::sort(paths.begin(), paths.end());
  EXPECT_EQ(paths.end(), std::unique(paths.begin(), paths.end()));
}
