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

// The 3D pane's one colour vocabulary (#36). These are the rules the two
// selectors are built from, so they are pinned here rather than only in the
// widget harness: the entries and their order, and — the part that is a
// decision rather than a list — which channels a layer cannot carry, and that
// each of those still says why.

#include <gtest/gtest.h>

#include <string>

#include "color_vocabulary.hpp"

using marine_perception_tools::ColorChannel;
using marine_perception_tools::color_channel_name;
using marine_perception_tools::kColorVocabulary;
using marine_perception_tools::point_channel_unavailable_reason;
using marine_perception_tools::surface_channel_unavailable_reason;

// One vocabulary, one order. The points and the surface are two layers of one
// picture; a selector that reordered or dropped an entry would make them read
// as unrelated controls again.
TEST(ColorVocabulary, IsTheFiveChannelsInPresentationOrder)
{
  ASSERT_EQ(kColorVocabulary.size(), 5u);
  EXPECT_EQ(kColorVocabulary[0], ColorChannel::Depth);
  EXPECT_EQ(kColorVocabulary[1], ColorChannel::Uncertainty);
  EXPECT_EQ(kColorVocabulary[2], ColorChannel::Backscatter);
  EXPECT_EQ(kColorVocabulary[3], ColorChannel::Pass);
  EXPECT_EQ(kColorVocabulary[4], ColorChannel::Sidescan);
  EXPECT_EQ(std::string(color_channel_name(ColorChannel::Pass)), "Pass");
  EXPECT_EQ(std::string(color_channel_name(ColorChannel::Sidescan)), "Sidescan");
}

// Pass is an ORDINARY entry for the points whenever the cloud was loaded as
// passes — the operator's complaint was that selecting a multi-pass region
// took the control away rather than offering pass colouring alongside the
// scalar ramps.
TEST(ColorVocabulary, PointsOfferDepthBackscatterAndPassOnAMultiPassCloud)
{
  EXPECT_EQ(
    point_channel_unavailable_reason(ColorChannel::Depth, true), nullptr);
  EXPECT_EQ(
    point_channel_unavailable_reason(ColorChannel::Backscatter, true), nullptr);
  EXPECT_EQ(
    point_channel_unavailable_reason(ColorChannel::Pass, true), nullptr);
}

// A cloud with no pass identity (the scrub window, a CUBE run's own gather)
// has nothing to tell apart, so Pass is greyed — and says so, rather than
// silently colouring every point the same hue.
TEST(ColorVocabulary, PassIsUnavailableWithoutPassIdentityAndSaysWhy)
{
  const char * reason =
    point_channel_unavailable_reason(ColorChannel::Pass, false);
  ASSERT_NE(reason, nullptr);
  EXPECT_FALSE(std::string(reason).empty());
}

// The verified gap (#36): MbesSounding carries x/y/z, intensity, beam angle
// and slant range — no uncertainty. The per-beam errors CUBE consumes are a
// placeholder derived from depth INSIDE run_cube, so colouring points by
// "uncertainty" would be colouring them by depth under another name. The
// entry stays listed and greyed; it must never be filled with an invented
// value. Sidescan is likewise a drape onto CUBE nodes, not a sounding field.
TEST(ColorVocabulary, PointsCannotCarryUncertaintyOrSidescanButStillExplainWhy)
{
  for (const auto channel : {ColorChannel::Uncertainty, ColorChannel::Sidescan}) {
    const char * reason = point_channel_unavailable_reason(channel, true);
    ASSERT_NE(reason, nullptr) << color_channel_name(channel);
    EXPECT_FALSE(std::string(reason).empty()) << color_channel_name(channel);
  }
}

// The surface carries every scalar the estimate produces, and the drape; it
// cannot carry pass identity, because a node merges every pass that touched
// it. That entry is present and greyed too, so the two lists match.
TEST(ColorVocabulary, SurfaceOffersEverythingButPass)
{
  EXPECT_EQ(surface_channel_unavailable_reason(ColorChannel::Depth), nullptr);
  EXPECT_EQ(
    surface_channel_unavailable_reason(ColorChannel::Uncertainty), nullptr);
  EXPECT_EQ(
    surface_channel_unavailable_reason(ColorChannel::Backscatter), nullptr);
  EXPECT_EQ(surface_channel_unavailable_reason(ColorChannel::Sidescan), nullptr);
  const char * reason = surface_channel_unavailable_reason(ColorChannel::Pass);
  ASSERT_NE(reason, nullptr);
  EXPECT_FALSE(std::string(reason).empty());
}

// Every channel is offered by at least one of the two layers: an entry no
// layer can ever colour by would be vocabulary for its own sake.
TEST(ColorVocabulary, EveryChannelIsColourableSomewhere)
{
  for (const auto channel : kColorVocabulary) {
    const bool points = point_channel_unavailable_reason(channel, true) == nullptr;
    const bool surface = surface_channel_unavailable_reason(channel) == nullptr;
    EXPECT_TRUE(points || surface) << color_channel_name(channel);
  }
}
