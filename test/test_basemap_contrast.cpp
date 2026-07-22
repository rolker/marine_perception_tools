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

#include "basemap_contrast.hpp"

namespace
{

using marine_perception_tools::robust_range;

// The bug this helper exists for: Massabesic's bathy store holds real depths
// of ~38-48 m plus a handful of residual outlier cells reaching -4305 m. A
// min/max range mapped every real depth to the top 0.2% of the colour scale
// (a flat, unreadable basemap); the percentile range must stay on the data.
TEST(BasemapContrast, OutlierCellsDoNotOwnTheRange)
{
  std::vector<double> values;
  for (int i = 0; i < 10000; ++i) {
    values.push_back(38.0 + 10.0 * (i % 100) / 100.0);   // real depths 38..48
  }
  values.push_back(-4305.3);   // the junk
  values.push_back(-1069.2);
  const auto [lo, hi] = robust_range(values);
  EXPECT_GE(lo, 38.0);
  EXPECT_LE(hi, 48.0);
  EXPECT_GT(hi - lo, 5.0);   // most of the real span survives the clip
}

TEST(BasemapContrast, EmptyYieldsUnitRange)
{
  std::vector<double> values;
  const auto [lo, hi] = robust_range(values);
  EXPECT_EQ(lo, 0.0);
  EXPECT_EQ(hi, 1.0);
}

TEST(BasemapContrast, NearConstantWidensToMinMax)
{
  // 97% identical values: the 2/98 percentiles coincide, so the range must
  // widen to min/max rather than collapse to zero span.
  std::vector<double> values(970, 42.0);
  for (int i = 0; i < 30; ++i) {
    values.push_back(40.0 + 0.1 * i);
  }
  const auto [lo, hi] = robust_range(values);
  EXPECT_LT(lo, hi);
  EXPECT_EQ(lo, 40.0);
}

TEST(BasemapContrast, AllEqualStaysDegenerate)
{
  std::vector<double> values(100, 7.0);
  const auto [lo, hi] = robust_range(values);
  EXPECT_EQ(lo, 7.0);
  EXPECT_EQ(hi, 7.0);   // caller's span<=0 convention takes over
}

TEST(BasemapContrast, SmallSampleExactPercentiles)
{
  std::vector<double> values{1.0, 2.0, 3.0, 4.0, 5.0};
  const auto [lo, hi] = robust_range(values, 0.0, 1.0);
  EXPECT_EQ(lo, 1.0);
  EXPECT_EQ(hi, 5.0);
}

}  // namespace
