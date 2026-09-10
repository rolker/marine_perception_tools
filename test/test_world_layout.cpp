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
#include <filesystem>
#include <string>

#include "world_layout.hpp"

using marine_perception_tools::defaultStoresDir;
using marine_perception_tools::defaultWorldRoot;
using marine_perception_tools::preferredLayerPaths;
using marine_perception_tools::worldIndexPath;

// HOME is process-wide, so a test that changes it and walks away leaks into
// every test that runs afterwards — including code paths like session_index_io
// that read it — and makes the suite order-dependent (flagged by Copilot). This
// guard restores whatever was there, including "it was unset".
class ScopedHome
{
public:
  explicit ScopedHome(const char * value)
  {
    const char * previous = std::getenv("HOME");
    had_home_ = previous != nullptr;
    if (had_home_) {
      previous_ = previous;   // copy before any mutation invalidates it
    }
    if (value == nullptr) {
      ::unsetenv("HOME");
    } else {
      ::setenv("HOME", value, 1);
    }
  }

  ~ScopedHome()
  {
    if (had_home_) {
      ::setenv("HOME", previous_.c_str(), 1);
    } else {
      ::unsetenv("HOME");
    }
  }

  ScopedHome(const ScopedHome &) = delete;
  ScopedHome & operator=(const ScopedHome &) = delete;

private:
  bool had_home_ = false;
  std::string previous_;
};

TEST(WorldLayout, RootIsUnderHome)
{
  const ScopedHome home("/home/tester");
  EXPECT_EQ("/home/tester/data/world", defaultWorldRoot().string());
}

// An unset HOME must not degrade to a relative path: "data/world" would
// resolve against whatever directory the app happened to be launched from.
TEST(WorldLayout, RootIsEmptyWithoutHome)
{
  const ScopedHome home(nullptr);
  EXPECT_TRUE(defaultWorldRoot().empty());
  EXPECT_TRUE(worldIndexPath(defaultWorldRoot()).empty());
  EXPECT_TRUE(defaultStoresDir(defaultWorldRoot()).empty());
}

// The guard is the thing the other two rely on, so it is worth one test of its
// own: HOME must come back exactly as it was, unset included.
TEST(WorldLayout, ScopedHomeRestoresWhatItFound)
{
  const std::string before = std::getenv("HOME") ? std::getenv("HOME") : "";
  const bool had = std::getenv("HOME") != nullptr;
  {
    const ScopedHome home("/home/somewhere-else");
    ASSERT_STREQ("/home/somewhere-else", std::getenv("HOME"));
    {
      const ScopedHome nested(nullptr);
      ASSERT_EQ(nullptr, std::getenv("HOME"));
    }
    EXPECT_STREQ("/home/somewhere-else", std::getenv("HOME"));
  }
  EXPECT_EQ(had, std::getenv("HOME") != nullptr);
  if (had) {
    EXPECT_EQ(before, std::getenv("HOME"));
  }
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

// A BARE RELATIVE index path must still yield a stores directory (#42 review,
// independently flagged by Copilot). `path("survey_index.db").parent_path()` is
// empty and defaultStoresDir({}) is {} by design, so the unresolved form opened
// the explorer with no basemap and no explanation.
TEST(WorldLayout, ABareRelativeIndexPathStillResolvesAStoresDir)
{
  const auto stores =
    marine_perception_tools::defaultStoresDirForIndex("survey_index.db");

  EXPECT_FALSE(stores.empty()) << "a bare filename lost its stores directory";
  EXPECT_TRUE(stores.is_absolute());
  EXPECT_EQ(std::filesystem::current_path() / "depths" / "processed", stores);
}

// An absolute index path is unchanged by the resolution.
TEST(WorldLayout, AnAbsoluteIndexPathIsUnaffected)
{
  const auto stores = marine_perception_tools::defaultStoresDirForIndex(
    "/data/world/survey_index.db");

  EXPECT_EQ(std::filesystem::path("/data/world/depths/processed"), stores);
}

// A relative path that already names a directory resolves against the cwd
// rather than being treated as rootless.
TEST(WorldLayout, ARelativeIndexPathWithAParentResolvesAgainstTheCwd)
{
  const auto stores = marine_perception_tools::defaultStoresDirForIndex(
    "sub/dir/survey_index.db");

  EXPECT_EQ(
    std::filesystem::current_path() / "sub" / "dir" / "depths" / "processed",
    stores);
}
