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

// The click-versus-drag rule that decides which of two meanings each mouse
// button carries on the index map (#42).

#include <gtest/gtest.h>

#include "map_gesture.hpp"

using marine_perception_tools::gestureIsClick;
using marine_perception_tools::kClickSlopPx;

TEST(MapGesture, NoMovementIsAClick)
{
  EXPECT_TRUE(gestureIsClick(0, 0));
}

// The boundary is inclusive, so a hand that wobbles exactly to the limit gets
// the gesture it aimed at rather than a one-pixel region.
TEST(MapGesture, TravelExactlyAtTheThresholdIsStillAClick)
{
  EXPECT_TRUE(gestureIsClick(kClickSlopPx, 0));
  EXPECT_TRUE(gestureIsClick(0, kClickSlopPx));
  EXPECT_TRUE(gestureIsClick(kClickSlopPx - 1, 1));
}

TEST(MapGesture, OnePixelPastTheThresholdIsADrag)
{
  EXPECT_FALSE(gestureIsClick(kClickSlopPx + 1, 0));
  EXPECT_FALSE(gestureIsClick(0, kClickSlopPx + 1));
  EXPECT_FALSE(gestureIsClick(kClickSlopPx, 1));
}

// Manhattan distance, and direction never matters: dragging up-left is the
// same gesture as dragging down-right.
TEST(MapGesture, DirectionIsIrrelevant)
{
  EXPECT_EQ(gestureIsClick(3, 3), gestureIsClick(-3, -3));
  EXPECT_EQ(gestureIsClick(3, -3), gestureIsClick(-3, 3));
  EXPECT_FALSE(gestureIsClick(-5, 0));
}

TEST(MapGesture, SlopIsOverridable)
{
  EXPECT_TRUE(gestureIsClick(20, 0, 20));
  EXPECT_FALSE(gestureIsClick(20, 0, 19));
  EXPECT_FALSE(gestureIsClick(1, 0, 0));
  EXPECT_TRUE(gestureIsClick(0, 0, 0));
}
