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

#include "pass_timeline_model.hpp"

namespace
{

using marine_perception_tools::TimelinePass;
using marine_perception_tools::hitPass;
using marine_perception_tools::layoutTimeline;

constexpr std::int64_t kSec = 1000000000LL;

TEST(LayoutTimeline, EmptyInputYieldsEmptyLayout)
{
  const auto layout = layoutTimeline({});
  EXPECT_TRUE(layout.spans.empty());
  EXPECT_TRUE(layout.pass_x.empty());
  EXPECT_DOUBLE_EQ(layout.xOf(123), 0.0);
}

TEST(LayoutTimeline, SinglePassFillsTheAxis)
{
  const std::vector<TimelinePass> passes = {{0, 100 * kSec, 0}};
  const auto layout = layoutTimeline(passes);
  ASSERT_EQ(layout.spans.size(), 1u);
  EXPECT_DOUBLE_EQ(layout.spans[0].x0, 0.0);
  EXPECT_DOUBLE_EQ(layout.spans[0].x1, 1.0);
  EXPECT_DOUBLE_EQ(layout.pass_x[0].first, 0.0);
  EXPECT_DOUBLE_EQ(layout.pass_x[0].second, 1.0);
  // Linear inside the span.
  EXPECT_NEAR(layout.xOf(50 * kSec), 0.5, 1e-12);
}

TEST(LayoutTimeline, GapCompressesToFixedFraction)
{
  // Two 100 s passes a week apart: without compression the bars would be
  // ~0.017 % of the axis each; with it they share ~98 % equally.
  const std::vector<TimelinePass> passes = {
    {0, 100 * kSec, 0},
    {7 * 24 * 3600 * kSec, 7 * 24 * 3600 * kSec + 100 * kSec, 0}};
  const auto layout = layoutTimeline(passes, 0, 0.02);
  ASSERT_EQ(layout.spans.size(), 2u);
  const double w0 = layout.spans[0].x1 - layout.spans[0].x0;
  const double w1 = layout.spans[1].x1 - layout.spans[1].x0;
  EXPECT_NEAR(w0, 0.49, 1e-9);
  EXPECT_NEAR(w1, 0.49, 1e-9);
  EXPECT_NEAR(layout.spans[1].x0 - layout.spans[0].x1, 0.02, 1e-9);
  EXPECT_LE(layout.spans[1].x1, 1.0);
}

TEST(LayoutTimeline, OverlappingPassesShareOneSpan)
{
  const std::vector<TimelinePass> passes = {
    {0, 60 * kSec, 0},
    {30 * kSec, 90 * kSec, 1}};
  const auto layout = layoutTimeline(passes);
  ASSERT_EQ(layout.spans.size(), 1u);
  EXPECT_EQ(layout.spans[0].t0, 0);
  EXPECT_EQ(layout.spans[0].t1, 90 * kSec);
}

TEST(LayoutTimeline, PaddingMergesNearbyPasses)
{
  // 8 s apart: separate spans unpadded, one span with a 5 s pad.
  const std::vector<TimelinePass> passes = {
    {0, 10 * kSec, 0},
    {18 * kSec, 30 * kSec, 0}};
  EXPECT_EQ(layoutTimeline(passes, 0).spans.size(), 2u);
  EXPECT_EQ(layoutTimeline(passes, 5 * kSec).spans.size(), 1u);
}

TEST(LayoutTimeline, ManyGapsShrinkToHalfTheAxis)
{
  // 60 spans at 2 % per gap would eat 118 % of the axis; the cap leaves half
  // for the data.
  std::vector<TimelinePass> passes;
  for (int i = 0; i < 60; ++i) {
    passes.push_back({i * 1000 * kSec, i * 1000 * kSec + kSec, 0});
  }
  const auto layout = layoutTimeline(passes, 0, 0.02);
  ASSERT_EQ(layout.spans.size(), 60u);
  double data_width = 0.0;
  for (const auto & span : layout.spans) {
    data_width += span.x1 - span.x0;
    EXPECT_GT(span.x1, span.x0);
  }
  EXPECT_NEAR(data_width, 0.5, 1e-9);
  EXPECT_LE(layout.spans.back().x1, 1.0);
}

TEST(LayoutTimeline, XOfPinsGapsAndOutOfRangeToSpanEdges)
{
  const std::vector<TimelinePass> passes = {
    {0, 10 * kSec, 0},
    {100 * kSec, 110 * kSec, 0}};
  const auto layout = layoutTimeline(passes, 0, 0.1);
  EXPECT_DOUBLE_EQ(layout.xOf(-5 * kSec), layout.spans[0].x0);
  // In a compressed gap: pinned to the following span's left edge.
  EXPECT_DOUBLE_EQ(layout.xOf(50 * kSec), layout.spans[1].x0);
  EXPECT_DOUBLE_EQ(layout.xOf(500 * kSec), layout.spans[1].x1);
}

TEST(HitPass, RowFilteredAndNarrowestWins)
{
  // Pass 1 nests inside pass 0's interval on the same row; a click over the
  // small bar must pick the small bar. Row 1 holds an unrelated pass.
  const std::vector<TimelinePass> passes = {
    {0, 100 * kSec, 0},
    {40 * kSec, 50 * kSec, 0},
    {0, 100 * kSec, 1}};
  const auto layout = layoutTimeline(passes);
  const double x_mid = layout.xOf(45 * kSec);
  EXPECT_EQ(hitPass(passes, layout, x_mid, 0), 1);
  EXPECT_EQ(hitPass(passes, layout, layout.xOf(10 * kSec), 0), 0);
  EXPECT_EQ(hitPass(passes, layout, x_mid, 1), 2);
  EXPECT_EQ(hitPass(passes, layout, x_mid, 5), -1);
}

}  // namespace
