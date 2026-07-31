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

#include <cmath>
#include <cstdint>

#include "time_bar_model.hpp"

namespace
{

using marine_perception_tools::computeTickLadder;
using marine_perception_tools::offsetTimeNs;
using marine_perception_tools::TickRow;

constexpr std::int64_t kNsPerS = 1000000000LL;

// 1970-01-05 00:00:00 UTC — a Monday at an exact day boundary, so calendar
// expectations are hand-checkable (1970-01-01 was a Thursday).
constexpr std::int64_t kMonday = 4LL * 24 * 3600 * kNsPerS;

const TickRow * rowFor(const std::vector<TickRow> & rows, double interval_s)
{
  for (const auto & r : rows) {
    if (std::abs(r.interval_s - interval_s) < 1e-9) {
      return &r;
    }
  }
  return nullptr;
}

TEST(TimeBarModel, SecondTicksLandOnWholeSeconds)
{
  // 0.1 s/px over 600 px starting 0.25 s before a whole second: second ticks
  // are 10 px apart, the first one 2.5 px in.
  const std::int64_t left = kMonday - kNsPerS / 4;
  const auto rows = computeTickLadder(left, 0.1, 600.0);
  const auto * sec = rowFor(rows, 1.0);
  ASSERT_NE(sec, nullptr);
  ASSERT_FALSE(sec->ticks.empty());
  EXPECT_NEAR(sec->ticks.front().x_px, 2.5, 1e-6);
  ASSERT_GT(sec->ticks.size(), 2u);
  EXPECT_NEAR(sec->ticks[1].x_px - sec->ticks[0].x_px, 10.0, 1e-9);
}

TEST(TimeBarModel, LevelsOutsideThePixelWindowAreEmpty)
{
  // At 1 s/px: milliseconds are 0.001 px apart (< 3 px -> hidden) and days
  // are 86400 px apart (>= 100k px would hide; 86400 is visible).
  const auto rows = computeTickLadder(kMonday, 1.0, 600.0);
  const auto * ms = rowFor(rows, 0.001);
  ASSERT_NE(ms, nullptr);
  EXPECT_TRUE(ms->ticks.empty());
  // At 10000 s/px, minute ticks (0.006 px) hide; days (8.64 px) show.
  const auto zoomed_out = computeTickLadder(kMonday, 10000.0, 600.0);
  EXPECT_TRUE(rowFor(zoomed_out, 60.0)->ticks.empty());
  EXPECT_FALSE(rowFor(zoomed_out, 24.0 * 3600.0)->ticks.empty());
}

TEST(TimeBarModel, MinuteRolloverWrapsToZero)
{
  // Window starting at 58 min past the hour, 6 s/px over 600 px spans ~1 h:
  // the minute values must wrap 59 -> 0 at the hour, not run to 60+.
  const std::int64_t left = kMonday + (58 * 60) * kNsPerS;
  const auto rows = computeTickLadder(left, 6.0, 600.0);
  const auto * minutes = rowFor(rows, 60.0);
  ASSERT_NE(minutes, nullptr);
  bool saw_zero = false;
  for (const auto & t : minutes->ticks) {
    if (!t.label.empty()) {
      EXPECT_NE(t.label, "60m");
      if (t.label == "0m") {
        saw_zero = true;
      }
    }
  }
  EXPECT_TRUE(saw_zero);
}

TEST(TimeBarModel, WeekdayNamesMatchTheCalendar)
{
  // Days at ~1000 s/px: day ticks are 86.4 px apart (labels need > 50 px).
  // Left edge Monday 00:00 -> first day tick is TUESDAY's start.
  const auto rows = computeTickLadder(kMonday + 1, 1000.0, 600.0);
  const auto * days = rowFor(rows, 24.0 * 3600.0);
  ASSERT_NE(days, nullptr);
  ASSERT_FALSE(days->ticks.empty());
  EXPECT_EQ(days->ticks.front().label, "Tue");
}

TEST(TimeBarModel, TickHeightGrowsWithSpacing)
{
  const auto rows = computeTickLadder(kMonday, 1.0, 600.0);
  const auto * sec = rowFor(rows, 1.0);      // 1 px apart -> hidden? 1 < 3 hidden!
  const auto * minutes = rowFor(rows, 60.0);   // 60 px apart
  const auto * hours = rowFor(rows, 3600.0);   // 3600 px apart
  ASSERT_NE(minutes, nullptr);
  ASSERT_NE(hours, nullptr);
  EXPECT_TRUE(sec->ticks.empty());   // 1 px spacing is below the 3 px floor
  EXPECT_GT(hours->tick_frac, minutes->tick_frac);
  EXPECT_LE(hours->tick_frac, 1.0);
}

TEST(TimeBarModel, DegenerateInputsYieldNoTicks)
{
  EXPECT_TRUE(computeTickLadder(kMonday, 0.0, 600.0).empty());
  EXPECT_TRUE(computeTickLadder(kMonday, -1.0, 600.0).empty());
  for (const auto & row : computeTickLadder(kMonday, 1.0, 0.0)) {
    EXPECT_TRUE(row.ticks.empty());
  }
}

TEST(TimeBarModel, OffsetTimeNsIsExactInRangeAndSaturatesBeyond)
{
  // Normal interactive spans stay exact.
  EXPECT_EQ(offsetTimeNs(kMonday, 25.0, 2.0), kMonday + 50 * kNsPerS);
  EXPECT_EQ(offsetTimeNs(kMonday, -25.0, 2.0), kMonday - 50 * kNsPerS);
  // A 4096 px page at the kMaxSpp zoom clamp is ~1.1e19 ns — past int64
  // range; the raw cast was UB. Both directions saturate instead.
  constexpr std::int64_t kSatNs = static_cast<std::int64_t>(9.2e18);
  EXPECT_EQ(offsetTimeNs(0, 4096.0, 2.7e6), kSatNs);
  EXPECT_EQ(offsetTimeNs(0, -4096.0, 2.7e6), -kSatNs);
  // The addition saturates too, even when the span alone is representable.
  EXPECT_EQ(offsetTimeNs(kSatNs, 4096.0, 2.7e6), kSatNs);
}

TEST(TimeBarModel, LabelsAppearOnlyWithRoom)
{
  // Minute ticks at 60 px spacing (1 s/px): full labels (> 50 px rule).
  const auto rows = computeTickLadder(kMonday, 1.0, 600.0);
  const auto * minutes = rowFor(rows, 60.0);
  ASSERT_NE(minutes, nullptr);
  for (const auto & t : minutes->ticks) {
    EXPECT_FALSE(t.label.empty());
    EXPECT_FALSE(t.minor_label);
  }
  // At 2 s/px minute ticks are 30 px apart: only every-15th minor labels.
  const auto crowded = computeTickLadder(kMonday, 2.0, 600.0);
  const auto * crowded_min = rowFor(crowded, 60.0);
  ASSERT_NE(crowded_min, nullptr);
  for (const auto & t : crowded_min->ticks) {
    if (!t.label.empty()) {
      EXPECT_TRUE(t.minor_label);
    }
  }
}

}  // namespace
