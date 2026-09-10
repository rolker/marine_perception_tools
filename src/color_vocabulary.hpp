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

#ifndef COLOR_VOCABULARY_HPP_
#define COLOR_VOCABULARY_HPP_

// One colour vocabulary for the 3D pane (#36). The point cloud and the CUBE
// surface are two layers of one picture, so they offer the SAME entries in the
// SAME order — a channel a layer cannot carry stays in the list, disabled, with
// the reason on its tooltip. A silently shorter list reads as two unrelated
// controls, which is how the pane came to have two disagreeing dropdowns.
//
// Pure (no Qt, no GL): the vocabulary and the availability rules are pinned in
// test_color_vocabulary without a display.

#include <array>

namespace marine_perception_tools
{

// The vocabulary, in presentation order. Depth first (the default everywhere),
// then the two other estimated scalars, then the two identity/imagery channels.
enum class ColorChannel
{
  Depth,
  Uncertainty,
  Backscatter,
  Pass,
  Sidescan,
};

inline constexpr std::array<ColorChannel, 5> kColorVocabulary{
  ColorChannel::Depth,
  ColorChannel::Uncertainty,
  ColorChannel::Backscatter,
  ColorChannel::Pass,
  ColorChannel::Sidescan,
};

inline const char * color_channel_name(ColorChannel channel)
{
  switch (channel) {
    case ColorChannel::Depth: return "Depth";
    case ColorChannel::Uncertainty: return "Uncertainty";
    case ColorChannel::Backscatter: return "Backscatter";
    case ColorChannel::Pass: return "Pass";
    case ColorChannel::Sidescan: return "Sidescan";
  }
  return "Depth";
}

// Why the POINT cloud cannot offer a channel; nullptr when it can.
//
// Uncertainty is the load-bearing one: MbesSounding carries x/y/z, intensity,
// beam angle and slant range — no uncertainty. The per-sounding vertical and
// horizontal errors CUBE consumes are a placeholder model computed inside
// run_cube from that beam angle and slant range (#49), never a measured field
// of the sounding, so colouring points by "uncertainty" would be colouring
// them by the model rather than by anything the sonar reported. Uncertainty is
// a property of the CUBE estimate, not of a beam.
//
// `has_pass_identity` is false for a cloud loaded as one set of points (the
// scrub window, a CUBE run's own gather) — there are no passes to tell apart.
inline const char * point_channel_unavailable_reason(
  ColorChannel channel, bool has_pass_identity)
{
  switch (channel) {
    case ColorChannel::Uncertainty:
      return "A sounding carries no uncertainty — the per-beam errors CUBE "
             "uses are a placeholder computed inside the estimator from the "
             "beam's angle and slant range, not something the sonar reported. "
             "Uncertainty is a property of the CUBE surface; colour the "
             "surface by it instead.";
    case ColorChannel::Sidescan:
      return "Sidescan is a drape: amplitude is painted onto CUBE nodes by "
             "marching the terrain, not carried by the soundings. Colour the "
             "surface by it instead.";
    case ColorChannel::Pass:
      return has_pass_identity ?
             nullptr :
             "This cloud holds one set of points with no pass identity — "
             "select a region to load its passes.";
    default:
      return nullptr;
  }
}

// Why the CUBE SURFACE cannot offer a channel; nullptr when it can.
inline const char * surface_channel_unavailable_reason(ColorChannel channel)
{
  if (channel == ColorChannel::Pass) {
    return "A CUBE node merges every pass that touched it — pass identity is "
           "a per-sounding property and does not survive the estimate. Colour "
           "the points by it instead.";
  }
  return nullptr;
}

}  // namespace marine_perception_tools

#endif  // COLOR_VOCABULARY_HPP_
