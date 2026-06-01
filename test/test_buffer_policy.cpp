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

// Pure unit tests for the windowed-buffer span policy (Milestone D, R9). No bag,
// no Qt — exercises plan_buffer() arithmetic and its boundary cases directly.

#include <gtest/gtest.h>

#include "buffer_policy.hpp"

namespace mpt = marine_perception_tools;

namespace
{

// Defaults mirroring the CLI: integration 90s (3 x 30s half-life), margin 10s,
// retention 120s, over a 600s bag. Guaranteed window length = integration +
// 2*margin = 110s; max cache span = 110 + 120 = 230s.
mpt::BufferParams default_params(double bag_hi = 600.0)
{
  mpt::BufferParams p;
  p.integration_s = 90.0;
  p.margin_s = 10.0;
  p.retention_s = 120.0;
  p.bag_lo = 0.0;
  p.bag_hi = bag_hi;
  return p;
}

mpt::BufferState no_cache()
{
  return mpt::BufferState{};  // has_cache = false
}

mpt::BufferState cache(double lo, double hi)
{
  mpt::BufferState s;
  s.has_cache = true;
  s.cache_lo = lo;
  s.cache_hi = hi;
  return s;
}

constexpr double kEps = 1e-6;

}  // namespace

// --- Fresh load: first File->Open materializes the guaranteed window. ---------

TEST(BufferPolicy, FreshLoadMaterializesGuaranteedWindow)
{
  const auto p = default_params();
  const auto plan = mpt::plan_buffer(300.0, p, no_cache());

  EXPECT_EQ(plan.action, mpt::BufferAction::FreshLoad);
  EXPECT_TRUE(plan.read);
  // guaranteed = [t - integration - margin, t + margin] = [200, 310]
  EXPECT_NEAR(plan.cache_lo, 200.0, kEps);
  EXPECT_NEAR(plan.cache_hi, 310.0, kEps);
  EXPECT_NEAR(plan.read_lo, 200.0, kEps);
  EXPECT_NEAR(plan.read_hi, 310.0, kEps);
  // engine replays exactly [t - integration, t] = [210, 300]
  EXPECT_NEAR(plan.replay_lo, 210.0, kEps);
  EXPECT_NEAR(plan.replay_hi, 300.0, kEps);
  EXPECT_NEAR(plan.warmup_s(), 90.0, kEps);
}

// --- R9a: near bag start, lo clamps and warm-up is shorter than integration. --

TEST(BufferPolicy, NearBagStartClampsAndUnderWarms)
{
  const auto p = default_params();
  const auto plan = mpt::plan_buffer(30.0, p, no_cache());

  EXPECT_EQ(plan.action, mpt::BufferAction::FreshLoad);
  // replay_lo clamps to bag start (0), not t - integration = -60.
  EXPECT_NEAR(plan.replay_lo, 0.0, kEps);
  EXPECT_NEAR(plan.replay_hi, 30.0, kEps);
  // warm-up is the clamped 30s, NOT the full 90s — the caller surfaces this.
  EXPECT_NEAR(plan.warmup_s(), 30.0, kEps);
  EXPECT_LT(plan.warmup_s(), p.integration_s);
  // guaranteed window also clamps at the bottom.
  EXPECT_NEAR(plan.cache_lo, 0.0, kEps);
  EXPECT_NEAR(plan.cache_hi, 40.0, kEps);
}

// --- No reload: scrubbing within the cached replay span just re-slices. -------

TEST(BufferPolicy, ScrubInsideCachedSpanNoReload)
{
  const auto p = default_params();
  // Cache spans [200, 310] from a fresh load at t=300. Nudge to t=305: replay
  // [215, 305] is still inside the cache → no disk read.
  const auto plan = mpt::plan_buffer(305.0, p, cache(200.0, 310.0));

  EXPECT_EQ(plan.action, mpt::BufferAction::NoReload);
  EXPECT_FALSE(plan.read);
  EXPECT_NEAR(plan.replay_lo, 215.0, kEps);
  EXPECT_NEAR(plan.replay_hi, 305.0, kEps);
}

TEST(BufferPolicy, SmallRewindInsideMarginNoReload)
{
  const auto p = default_params();
  // The margin exists so a small rewind stays cached. Cache [200,310] built at
  // t=300; rewind to t=295 → replay [205,295], still >= cache_lo 200 → no read.
  const auto plan = mpt::plan_buffer(295.0, p, cache(200.0, 310.0));

  EXPECT_EQ(plan.action, mpt::BufferAction::NoReload);
  EXPECT_FALSE(plan.read);
  EXPECT_GE(plan.replay_lo, 200.0 - kEps);
}

// --- Extend: scrubbing just past the cache reads only the new slice. ----------

TEST(BufferPolicy, ScrubForwardExtendsReadsOnlyNewSlice)
{
  const auto p = default_params();
  // Cache [200, 310]. Scrub forward to t=340: guaranteed = [240, 350], replay
  // [250, 340] — replay_hi 340 > cache_hi 310, so extend upward.
  const auto plan = mpt::plan_buffer(340.0, p, cache(200.0, 310.0));

  EXPECT_EQ(plan.action, mpt::BufferAction::Extend);
  EXPECT_TRUE(plan.read);
  // Only the newly-exposed slice [old cache_hi, new cache_hi] is read.
  EXPECT_NEAR(plan.read_lo, 310.0, kEps);
  EXPECT_NEAR(plan.read_hi, 350.0, kEps);
  EXPECT_NEAR(plan.cache_hi, 350.0, kEps);
  // Total span 200..350 = 150s <= max_span 230s, so no trim yet; lo unchanged.
  EXPECT_NEAR(plan.cache_lo, 200.0, kEps);
}

// --- R9c: retention trim removes the far end, never the guaranteed window. ----

TEST(BufferPolicy, RetentionTrimDropsFarEndNotGuaranteedWindow)
{
  const auto p = default_params();
  // A large existing cache [50, 320] (270s > max_span 230s). View t=300:
  // guaranteed = [200, 310]. Forward extend to cache_hi 320 is already there;
  // the span must trim to 230s by dropping the low end (farthest from t=300),
  // but must not cut above lo of the guaranteed window (200).
  const auto plan = mpt::plan_buffer(300.0, p, cache(50.0, 320.0));

  // replay [210,300] is within [50,320] → no read needed.
  EXPECT_EQ(plan.action, mpt::BufferAction::NoReload);
  EXPECT_FALSE(plan.read);
  // Trimmed to 230s span ending at 320: lo := 90. But the guaranteed lower bound
  // is 200, and 90 < 200, so trimming to 90 does NOT cut into [200,310]. Good.
  EXPECT_NEAR(plan.cache_hi, 320.0, kEps);
  EXPECT_NEAR(plan.cache_lo, 90.0, kEps);
  EXPECT_NEAR(plan.cache_hi - plan.cache_lo, 230.0, kEps);
  // Guaranteed window still fully inside the trimmed cache.
  EXPECT_LE(plan.cache_lo, 200.0 + kEps);
  EXPECT_GE(plan.cache_hi, 310.0 - kEps);
}

TEST(BufferPolicy, RetentionFloorIsGuaranteedWindowWhenRetentionTiny)
{
  auto p = default_params();
  p.retention_s = 0.0;  // R9c: floor is the guaranteed-window length regardless.
  // Big cache, tight retention: the cache must shrink to the guaranteed window
  // (110s) but no further — never cut inside [g_lo, g_hi].
  const auto plan = mpt::plan_buffer(300.0, p, cache(50.0, 500.0));

  // guaranteed = [200, 310]; with retention 0, max_span == guaranteed length.
  EXPECT_LE(plan.cache_lo, 200.0 + kEps);
  EXPECT_GE(plan.cache_hi, 310.0 - kEps);
  EXPECT_NEAR(plan.cache_hi - plan.cache_lo, 110.0, kEps);
}

// --- Far jump: non-overlapping guaranteed window drops the cache, reloads. ----

TEST(BufferPolicy, FarJumpDropsCacheAndReloadsFresh)
{
  const auto p = default_params();
  // Cache [200, 310]. Jump far forward to t=550: guaranteed = [450, 560], which
  // does not overlap [200, 310] → drop + reload fresh.
  const auto plan = mpt::plan_buffer(550.0, p, cache(200.0, 310.0));

  EXPECT_EQ(plan.action, mpt::BufferAction::FarReload);
  EXPECT_TRUE(plan.read);
  EXPECT_NEAR(plan.cache_lo, 450.0, kEps);
  EXPECT_NEAR(plan.cache_hi, 560.0, kEps);
  EXPECT_NEAR(plan.read_lo, 450.0, kEps);
  EXPECT_NEAR(plan.read_hi, 560.0, kEps);
  EXPECT_NEAR(plan.replay_lo, 460.0, kEps);
  EXPECT_NEAR(plan.replay_hi, 550.0, kEps);
}

TEST(BufferPolicy, FarJumpBackwardDropsCacheAndReloadsFresh)
{
  const auto p = default_params();
  // Cache [400, 510] (from viewing t=500). Jump back to t=100: guaranteed =
  // [0, 110] (lo = 100-90-10 = 0), no overlap with [400,510] → far reload.
  const auto plan = mpt::plan_buffer(100.0, p, cache(400.0, 510.0));

  EXPECT_EQ(plan.action, mpt::BufferAction::FarReload);
  EXPECT_NEAR(plan.cache_lo, 0.0, kEps);
  EXPECT_NEAR(plan.cache_hi, 110.0, kEps);
}

// --- R9b: half-life raised so integration exceeds the bag → clamp to whole. ---

TEST(BufferPolicy, IntegrationExceedingBagClampsToWholeBag)
{
  auto p = default_params(/*bag_hi=*/200.0);
  p.integration_s = 600.0;  // e.g. decay_half_life_s bumped to 200 -> 3x = 600
  const auto plan = mpt::plan_buffer(150.0, p, no_cache());

  // replay_lo clamps to bag start; replay spans the whole bag up to t.
  EXPECT_NEAR(plan.replay_lo, 0.0, kEps);
  EXPECT_NEAR(plan.replay_hi, 150.0, kEps);
  // guaranteed window clamps to [0, 160] (t+margin), well inside the bag.
  EXPECT_NEAR(plan.cache_lo, 0.0, kEps);
  EXPECT_NEAR(plan.cache_hi, 160.0, kEps);
}

TEST(BufferPolicy, IntegrationGrowthForcesReloadWhenCacheTooSmall)
{
  auto p = default_params();
  // Cache built when integration was small: [290, 315] around t=300. Now the
  // half-life knob raised integration to 90: replay needs [210,300], which is
  // not inside [290,315], and growth is needed on the LOW side only (315 already
  // covers t+margin=310 minus... actually g_hi=310 < 315) → one-sided extend low.
  const auto plan = mpt::plan_buffer(300.0, p, cache(290.0, 315.0));

  EXPECT_TRUE(plan.read);
  // replay_lo 210 must be covered after the operation.
  EXPECT_LE(plan.cache_lo, 210.0 + kEps);
  EXPECT_GE(plan.cache_hi, 300.0 - kEps);
}

// --- t exactly at bag end. ----------------------------------------------------

TEST(BufferPolicy, ViewAtBagEndClampsUpper)
{
  const auto p = default_params(/*bag_hi=*/600.0);
  const auto plan = mpt::plan_buffer(600.0, p, no_cache());

  EXPECT_NEAR(plan.replay_hi, 600.0, kEps);
  EXPECT_NEAR(plan.replay_lo, 510.0, kEps);
  // t + margin clamps to bag_hi.
  EXPECT_NEAR(plan.cache_hi, 600.0, kEps);
  EXPECT_NEAR(plan.cache_lo, 500.0, kEps);
}

// --- t passed out of range is clamped defensively. ----------------------------

TEST(BufferPolicy, OutOfRangeViewIsClamped)
{
  const auto p = default_params(/*bag_hi=*/600.0);
  const auto over = mpt::plan_buffer(999.0, p, no_cache());
  EXPECT_NEAR(over.replay_hi, 600.0, kEps);

  const auto under = mpt::plan_buffer(-50.0, p, no_cache());
  EXPECT_NEAR(under.replay_hi, 0.0, kEps);
  EXPECT_NEAR(under.replay_lo, 0.0, kEps);
}
