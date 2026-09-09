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

// The eased glide behind the middle-click recentre (#42): exact endpoints,
// monotonic in between, and an instant path when the duration is zero.

#include <gtest/gtest.h>

#include <cmath>

#include "view_animation.hpp"

using marine_perception_tools::animationProgress;
using marine_perception_tools::easeInOutCubic;
using marine_perception_tools::easedInterpolate;
using marine_perception_tools::kRecenterDurationMs;

TEST(ViewAnimation, EndpointsAreExact)
{
  EXPECT_EQ(easeInOutCubic(0.0), 0.0);
  EXPECT_EQ(easeInOutCubic(1.0), 1.0);
}

// The curve is the only thing between the start and the target, so a dip
// anywhere in it would show as the map backing up mid-flight.
TEST(ViewAnimation, IsMonotonicAcrossTheWholeCurve)
{
  double previous = easeInOutCubic(0.0);
  for (int i = 1; i <= 1000; ++i) {
    const double v = easeInOutCubic(i / 1000.0);
    EXPECT_GE(v, previous) << "at t = " << i / 1000.0;
    previous = v;
  }
  EXPECT_EQ(previous, 1.0);
}

// Ease IN and ease OUT, not one or the other: the first half is behind a
// linear ramp (still accelerating) and the second half is ahead of it (already
// decelerating), meeting exactly at the midpoint.
TEST(ViewAnimation, AcceleratesThenDecelerates)
{
  EXPECT_DOUBLE_EQ(easeInOutCubic(0.5), 0.5);
  EXPECT_LT(easeInOutCubic(0.25), 0.25);
  EXPECT_GT(easeInOutCubic(0.75), 0.75);
  // Symmetric about the midpoint: the deceleration mirrors the acceleration.
  for (const double t : {0.1, 0.2, 0.3, 0.4}) {
    EXPECT_NEAR(easeInOutCubic(t), 1.0 - easeInOutCubic(1.0 - t), 1e-12);
  }
}

TEST(ViewAnimation, ProgressOutsideTheRunIsClamped)
{
  EXPECT_EQ(easeInOutCubic(-0.5), 0.0);
  EXPECT_EQ(easeInOutCubic(1.5), 1.0);
}

// The canvas compares its settled centre against the cached one for equality,
// so the landing has to be the target itself, not a value 1e-16 away from it.
TEST(ViewAnimation, InterpolationLandsExactlyOnBothEnds)
{
  const double from = 4302191.375;
  const double to = -1875.0625;
  EXPECT_EQ(easedInterpolate(from, to, 0.0), from);
  EXPECT_EQ(easedInterpolate(from, to, 1.0), to);
  EXPECT_EQ(easedInterpolate(from, to, -1.0), from);
  EXPECT_EQ(easedInterpolate(from, to, 2.0), to);
}

TEST(ViewAnimation, InterpolationTravelsMonotonicallyInBothDirections)
{
  double rising = easedInterpolate(-100.0, 100.0, 0.0);
  double falling = easedInterpolate(100.0, -100.0, 0.0);
  for (int i = 1; i <= 200; ++i) {
    const double t = i / 200.0;
    const double up = easedInterpolate(-100.0, 100.0, t);
    const double down = easedInterpolate(100.0, -100.0, t);
    EXPECT_GE(up, rising);
    EXPECT_LE(down, falling);
    rising = up;
    falling = down;
  }
  EXPECT_EQ(rising, 100.0);
  EXPECT_EQ(falling, -100.0);
}

TEST(ViewAnimation, ProgressIsTheElapsedFractionClampedToTheRun)
{
  EXPECT_DOUBLE_EQ(animationProgress(0.0, 750.0), 0.0);
  EXPECT_DOUBLE_EQ(animationProgress(375.0, 750.0), 0.5);
  EXPECT_DOUBLE_EQ(animationProgress(750.0, 750.0), 1.0);
  // A frame that lands late (a stalled repaint, a slow rebuild) settles the
  // animation rather than overshooting past the target.
  EXPECT_DOUBLE_EQ(animationProgress(4000.0, 750.0), 1.0);
  EXPECT_DOUBLE_EQ(animationProgress(-10.0, 750.0), 0.0);
}

// Zero duration is the instant path: headless snapshots and the widget tests
// use it so no capture can land on an intermediate frame.
TEST(ViewAnimation, ZeroOrNegativeDurationCompletesImmediately)
{
  EXPECT_DOUBLE_EQ(animationProgress(0.0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(animationProgress(0.0, -50.0), 1.0);
  EXPECT_EQ(easedInterpolate(7.0, 9.0, animationProgress(0.0, 0.0)), 9.0);
}

TEST(ViewAnimation, TheDefaultRunIsAboutThreeQuartersOfASecond)
{
  EXPECT_EQ(kRecenterDurationMs, 750);
}
