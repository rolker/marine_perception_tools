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

// The CUBE-lab path's offline projection (#55). What is worth pinning here is
// not the error model's arithmetic — that is cube_bathymetry's own test
// surface — but the three things THIS package owns: the offline parameter
// defaults, the single cube::Sounding -> MbesSounding rename site, and the
// per-ping/per-beam validity guard that cube's SQUARED range gate cannot
// provide.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "mbes_projection.hpp"

namespace
{

using marine_perception_tools::kOfflineMinimumRangeM;
using marine_perception_tools::offline_projection_caveat;
using marine_perception_tools::offline_projector_params;
using marine_perception_tools::project_ping;

constexpr const char * kBaseLink = "bizzy/base_link";
constexpr const char * kLevel = "bizzy/base_link_north_up";
constexpr const char * kTide = "bizzy/map_tide";
constexpr const char * kMap = "bizzy/map";
constexpr const char * kOdom = "bizzy/odom";
constexpr std::int32_t kStampSec = 1000;

geometry_msgs::msg::TransformStamped identityTransform(
  const std::string & parent, const std::string & child)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp.sec = kStampSec;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.rotation.w = 1.0;
  return t;
}

// Populate a TF buffer with the chains the projector looks up: attitude
// (level <- base_link) and heave (tide <- base_link). The shape mirrors the
// verified BizzyBoat tree rather than being two ad-hoc edges — TF is a TREE,
// so `base_link` may have exactly one parent, and parenting it to both the
// level and the tide frame would silently leave one of the two lookups
// unresolvable (which reads as missing attitude: NaN uncertainty on every
// sounding, with only a diagnostics counter to say so).
//
//   map ── base_link_north_up ── base_link
//    └──── odom ──── map_tide
//
// Every edge identity, so roll/pitch/heave are zero and the uncertainty comes
// out finite. (tf2::BufferCore holds a mutex and so is neither copyable nor
// movable — it is filled in place rather than returned.)
void fillTf(tf2::BufferCore & tf)
{
  tf.setTransform(identityTransform(kMap, kLevel), "test", true);
  tf.setTransform(identityTransform(kLevel, kBaseLink), "test", true);
  tf.setTransform(identityTransform(kMap, kOdom), "test", true);
  tf.setTransform(identityTransform(kOdom, kTide), "test", true);
}

// A ping of `n` beams, all at 1500 m/s, each with a distinct travel time so a
// swapped or dropped beam is visible, and a distinct intensity and rx angle.
marine_acoustic_msgs::msg::SonarDetections makePing(std::size_t n = 3)
{
  marine_acoustic_msgs::msg::SonarDetections d;
  d.header.stamp.sec = kStampSec;
  d.header.frame_id = "bizzy/m3";
  d.ping_info.sound_speed = 1500.0f;
  for (std::size_t i = 0; i < n; ++i) {
    d.two_way_travel_times.push_back(static_cast<float>(0.01 + 0.002 * i));
    d.tx_angles.push_back(0.0f);
    d.rx_angles.push_back(static_cast<float>(-0.4 + 0.4 * i));
    d.intensities.push_back(static_cast<float>(-30.0 - i));
  }
  return d;
}

}  // namespace

// --- offline_projector_params ----------------------------------------------

// The frame names are the caller's, verbatim: a projector whose attitude
// lookup silently used a library-default unprefixed `base_link` would resolve
// through the bag's `base_link -> base_link_frd` alias to SOMETHING, without
// throwing, and place every sounding off a frame that is not the boat.
TEST(OfflineProjectorParams, CarriesTheCallersFrameNames)
{
  const auto p = offline_projector_params(kBaseLink, kLevel, kTide);
  EXPECT_EQ(p.base_link_frame, kBaseLink);
  EXPECT_EQ(p.level_frame, kLevel);
  EXPECT_EQ(p.tide_frame, kTide);
}

// The near-field gate is this package's own choice (the library default is
// 0.0, which admits a zero-range beam as a 0 m depth at the sonar head), and
// it is a named constant so there is one place to revisit it.
TEST(OfflineProjectorParams, UsesTheOfflineNearFieldGate)
{
  const auto p = offline_projector_params(kBaseLink, kLevel, kTide);
  EXPECT_DOUBLE_EQ(p.minimum_range, kOfflineMinimumRangeM);
  EXPECT_DOUBLE_EQ(kOfflineMinimumRangeM, 0.05);
  // The far gate is not ours to set; it stays the library's.
  EXPECT_DOUBLE_EQ(p.maximum_range, cube::ProjectorParams{}.maximum_range);
}

// Vessel and device are LIBRARY DEFAULTS, deliberately (there is no offline
// configuration record — unh_marine_autonomy#385). Overriding them here would
// invent a number and make the lab disagree with the store, which runs the
// same defaults. Pinned so a later "improvement" that quietly tightens
// gps_drms has to argue with a test.
TEST(OfflineProjectorParams, RunsOnLibraryDefaultVesselAndDevice)
{
  const auto p = offline_projector_params(kBaseLink, kLevel, kTide);
  const cube::Vessel default_vessel;
  const cube::Device default_device;
  EXPECT_DOUBLE_EQ(p.vessel.gps_drms, default_vessel.gps_drms);
  EXPECT_DOUBLE_EQ(p.vessel.gps_x, default_vessel.gps_x);
  EXPECT_DOUBLE_EQ(p.vessel.gps_z, default_vessel.gps_z);
  EXPECT_DOUBLE_EQ(p.vessel.draft, default_vessel.draft);
  EXPECT_EQ(p.vessel.ellipsoidal_referenced, default_vessel.ellipsoidal_referenced);
  EXPECT_DOUBLE_EQ(
    p.device.across_track_beamwidth, default_device.across_track_beamwidth);
  EXPECT_DOUBLE_EQ(p.device.range_error_percent, default_device.range_error_percent);
  EXPECT_DOUBLE_EQ(p.device.range_error_floor_m, default_device.range_error_floor_m);
}

// The caveat is one string shared by the CUBE-tuning dialog and the load note.
// Its ORDER is the point (decision 7): the 2 m GPS assumption is the one that
// sets a floor under every sounding's horizontal variance, so it is stated
// first, then the generic device, then the optimistic-horizontal consequence
// of the missing speed.
TEST(OfflineProjectionCaveat, NamesTheGpsAssumptionFirstAndTheSpeedConsequence)
{
  const std::string caveat = offline_projection_caveat();
  ASSERT_FALSE(caveat.empty());
  const auto gps = caveat.find("2 m");
  const auto device = caveat.find("generic");
  const auto speed = caveat.find("NaN");
  ASSERT_NE(gps, std::string::npos) << caveat;
  ASSERT_NE(device, std::string::npos) << caveat;
  ASSERT_NE(speed, std::string::npos) << caveat;
  EXPECT_LT(gps, speed) << caveat;
  EXPECT_NE(caveat.find("OPTIMISTIC"), std::string::npos) << caveat;
  EXPECT_NE(caveat.find("unh_marine_autonomy#385"), std::string::npos) << caveat;
}

// The 4 m² horizontal floor is real; the smeared surface it was once said to
// cause is not. Parameters::influenceRadius subtracts the 99% horizontal term
// from the depth-budget term, caps the result at that same term and floors it
// at the cell size, so below ~0.75 m cells the radius is one cell at any IHO
// budget and the >= ~5.15 m cap is only a ceiling, reached around 2 m cells.
// An operator who reads the caveat must not come away expecting a
// 5 m smear, so the text is pinned both ways: the ceiling is named as a
// ceiling, and the old claim that CUBE "spreads each sounding over an
// influence radius of about 5 m" may not come back.
TEST(OfflineProjectionCaveat, CallsTheHorizontalFloorACeilingNotASmear)
{
  const std::string caveat = offline_projection_caveat();
  EXPECT_NE(caveat.find("4 m²"), std::string::npos) << caveat;
  EXPECT_NE(caveat.find("CEILING"), std::string::npos) << caveat;
  EXPECT_NE(caveat.find("one cell"), std::string::npos) << caveat;
  EXPECT_EQ(caveat.find("influence radius of about 5 m"), std::string::npos)
    << caveat;
}

// --- project_ping -----------------------------------------------------------

// THE RENAME SITE. cube::Sounding spells both variances `*_error`;
// MbesSounding names them for what they are. Every other field is a
// straight copy, and the position comes from sonar_relative_position.
TEST(ProjectPing, MapsEveryFieldIncludingTheVarianceRename)
{
  tf2::BufferCore tf(std::chrono::seconds(600));
  fillTf(tf);
  const cube::DetectionsProjector projector(
    offline_projector_params(kBaseLink, kLevel, kTide));
  const auto ping = makePing();

  const auto mine = project_ping(projector, ping, tf, std::nanf(""));
  const auto theirs = projector.project(ping, tf, std::nanf(""));

  ASSERT_EQ(mine.soundings.size(), theirs.soundings.size());
  ASSERT_FALSE(mine.soundings.empty());
  for (std::size_t i = 0; i < mine.soundings.size(); ++i) {
    const auto & got = mine.soundings[i];
    const auto & cs = theirs.soundings[i];
    EXPECT_DOUBLE_EQ(got.x, cs.sonar_relative_position.x);
    EXPECT_DOUBLE_EQ(got.y, cs.sonar_relative_position.y);
    EXPECT_DOUBLE_EQ(got.z, cs.sonar_relative_position.z);
    EXPECT_FLOAT_EQ(got.intensity, cs.intensity);
    EXPECT_FLOAT_EQ(got.beam_angle, cs.beam_angle);
    EXPECT_FLOAT_EQ(got.slant_range, cs.slant_range);
    EXPECT_FLOAT_EQ(got.vertical_variance, cs.vertical_error);
    EXPECT_FLOAT_EQ(got.horizontal_variance, cs.horizontal_error);
    // Not a fabricated confidence: with attitude resolved, the real model
    // produces finite, positive variances, and the horizontal one carries the
    // 2 m generic GPS drms (4 m^2) the caveat names.
    EXPECT_TRUE(std::isfinite(got.vertical_variance));
    EXPECT_GT(got.vertical_variance, 0.0f);
    EXPECT_GE(got.horizontal_variance, 4.0f);
  }
}

// The projector's own per-ping counters are the caller's only view of how many
// beams the model saw and why they were dropped, so they pass through
// untouched — project_ping adds counters, it does not edit these.
TEST(ProjectPing, PassesTheProjectorsDiagnosticsThroughUnchanged)
{
  tf2::BufferCore tf(std::chrono::seconds(600));
  fillTf(tf);
  const cube::DetectionsProjector projector(
    offline_projector_params(kBaseLink, kLevel, kTide));
  const auto ping = makePing();

  const auto mine = project_ping(projector, ping, tf, std::nanf(""));
  const auto theirs = projector.project(ping, tf, std::nanf(""));

  EXPECT_EQ(mine.diagnostics.total, theirs.diagnostics.total);
  EXPECT_EQ(mine.diagnostics.filtered_range, theirs.diagnostics.filtered_range);
  EXPECT_EQ(mine.diagnostics.missing_attitude, theirs.diagnostics.missing_attitude);
  EXPECT_EQ(mine.diagnostics.missing_heave, theirs.diagnostics.missing_heave);
  EXPECT_EQ(
    mine.diagnostics.default_beamwidth_beams,
    theirs.diagnostics.default_beamwidth_beams);
  EXPECT_EQ(
    mine.diagnostics.missing_rx_angle_beams,
    theirs.diagnostics.missing_rx_angle_beams);
  // One sounding per beam before filtering, so the load note's arithmetic
  // (beams = soundings + filtered_range + invalid_beams) closes.
  EXPECT_EQ(mine.diagnostics.total, ping.two_way_travel_times.size());
  EXPECT_EQ(
    mine.diagnostics.total,
    mine.soundings.size() + mine.diagnostics.filtered_range + mine.invalid_beams);
}

// A ping whose sound speed is non-positive scales EVERY beam wrongly, and
// cube's gate cannot see it: a NEGATIVE sound speed mirrors the sounding and
// its SQUARED range still passes. The whole ping is dropped, and counted.
TEST(ProjectPing, DropsAWholePingWithANonPositiveSoundSpeed)
{
  tf2::BufferCore tf(std::chrono::seconds(600));
  fillTf(tf);
  const cube::DetectionsProjector projector(
    offline_projector_params(kBaseLink, kLevel, kTide));

  for (const float bad_speed : {0.0f, -1500.0f}) {
    auto ping = makePing();
    ping.ping_info.sound_speed = bad_speed;
    const auto out = project_ping(projector, ping, tf, std::nanf(""));
    EXPECT_TRUE(out.soundings.empty()) << bad_speed;
    EXPECT_EQ(out.invalid_pings, 1u) << bad_speed;
    EXPECT_EQ(out.invalid_beams, 0u) << bad_speed;
    // The ping never reached the model, so it contributes no beams either.
    EXPECT_EQ(out.diagnostics.total, 0u) << bad_speed;
  }
  // A good ping is not counted as invalid.
  const auto good = project_ping(projector, makePing(), tf, std::nanf(""));
  EXPECT_EQ(good.invalid_pings, 0u);
}

// A NEGATIVE travel time is the case cube's squared range gate admits: the
// position is mirrored through the sonar head but range^2 is unchanged, so the
// gate passes it and only a signed test rejects it. Its siblings survive —
// the drop is by VALUE (the returned sounding's slant_range), never by index,
// because the range gate has already shortened the vector.
TEST(ProjectPing, DropsANegativeTravelTimeBeamAndKeepsItsSiblings)
{
  tf2::BufferCore tf(std::chrono::seconds(600));
  fillTf(tf);
  const cube::DetectionsProjector projector(
    offline_projector_params(kBaseLink, kLevel, kTide));

  auto ping = makePing();
  const auto clean = project_ping(projector, ping, tf, std::nanf(""));
  ASSERT_EQ(clean.soundings.size(), 3u);
  ASSERT_EQ(clean.invalid_beams, 0u);

  ping.two_way_travel_times[1] = -0.012f;
  const auto out = project_ping(projector, ping, tf, std::nanf(""));

  EXPECT_EQ(out.invalid_beams, 1u);
  EXPECT_EQ(out.invalid_pings, 0u);
  ASSERT_EQ(out.soundings.size(), 2u);
  for (const auto & s : out.soundings) {
    EXPECT_GT(s.slant_range, 0.0f);
  }
  // The two survivors are beams 0 and 2, unchanged.
  EXPECT_FLOAT_EQ(out.soundings[0].slant_range, clean.soundings[0].slant_range);
  EXPECT_FLOAT_EQ(out.soundings[1].slant_range, clean.soundings[2].slant_range);
}

// A ZERO travel time is dropped too, but by cube's range gate rather than by
// the signed test: its range is 0, which falls below kOfflineMinimumRangeM, so
// the projector filters it before project_ping ever sees it and the drop lands
// in `filtered_range`. Both counters reach the load note, and the arithmetic
// still closes — what matters is that no beam at the sonar head survives with
// a spurious 0 m depth.
TEST(ProjectPing, DropsAZeroTravelTimeBeamThroughTheNearFieldGate)
{
  tf2::BufferCore tf(std::chrono::seconds(600));
  fillTf(tf);
  const cube::DetectionsProjector projector(
    offline_projector_params(kBaseLink, kLevel, kTide));

  auto ping = makePing();
  ping.two_way_travel_times[1] = 0.0f;
  const auto out = project_ping(projector, ping, tf, std::nanf(""));

  ASSERT_EQ(out.soundings.size(), 2u);
  EXPECT_EQ(out.diagnostics.filtered_range, 1u);
  EXPECT_EQ(out.invalid_beams, 0u);
  EXPECT_EQ(
    out.diagnostics.total,
    out.soundings.size() + out.diagnostics.filtered_range + out.invalid_beams);
  for (const auto & s : out.soundings) {
    EXPECT_GT(s.slant_range, 0.0f);
  }
}

// Without the attitude chain the error model cannot produce an uncertainty:
// the variances come back NaN while the POSITION stays finite, so the range
// gate keeps the sounding. That is why the count is surfaced in the load note
// and why run_cube has its own gate — nothing here may quietly substitute a
// number.
TEST(ProjectPing, MissingAttitudeYieldsNaNVariancesAndIsCounted)
{
  tf2::BufferCore tf(std::chrono::seconds(600));
  fillTf(tf);
  // A level frame the bag does not carry — the shape a namespace mismatch in
  // the frame configuration takes.
  const cube::DetectionsProjector projector(
    offline_projector_params(kBaseLink, "bizzy/not_a_frame", kTide));

  const auto out = project_ping(projector, makePing(), tf, std::nanf(""));

  EXPECT_EQ(out.diagnostics.missing_attitude, 1u);
  ASSERT_FALSE(out.soundings.empty());
  for (const auto & s : out.soundings) {
    EXPECT_TRUE(std::isfinite(s.slant_range));
    EXPECT_TRUE(std::isnan(s.vertical_variance));
    EXPECT_TRUE(std::isnan(s.horizontal_variance));
  }
}
