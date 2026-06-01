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
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "sensor_msgs/msg/camera_info.hpp"

#include "bag_loader.hpp"
#include "resim_engine.hpp"

namespace mpt = marine_perception_tools;

namespace
{

constexpr int kW = 64;
constexpr int kH = 48;
constexpr double kWindowM = 4.0;
constexpr double kRes = 0.1;
constexpr double kMaxRange = 100.0;

// A simple pinhole looking straight down at the z=0 plane from 2 m: fx=fy=50,
// principal point centred. The down-look rotation maps optical +z (forward) to
// world -z, so cells on the plane project in front of the camera.
sensor_msgs::msg::CameraInfo make_camera_info()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = kW;
  info.height = kH;
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {50.0, 0.0, 32.0, 0.0, 50.0, 24.0, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {50.0, 0.0, 32.0, 0.0, 0.0, 50.0, 24.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

// rgb8 mask: green water everywhere, a red obstacle square in the middle. The
// red square's bottom row is the waterline contact (water below) → hits; the
// green cells → misses. Guarantees the buffer actually accumulates.
cv::Mat make_mask()
{
  cv::Mat m(kH, kW, CV_8UC3, cv::Vec3b(0, 200, 0));  // rgb8 green = water
  for (int r = 20; r <= 28; ++r) {
    for (int c = 28; c <= 36; ++c) {
      m.at<cv::Vec3b>(r, c) = cv::Vec3b(220, 0, 0);  // rgb8 red = obstacle
    }
  }
  return m;
}

// Down-look rotation_cam_to_target: optical (x,y,z) -> world (x,-y,-z).
cv::Matx33d down_look_rotation()
{
  return cv::Matx33d(1, 0, 0, 0, -1, 0, 0, 0, -1);
}

mpt::LoadedBag make_bag(int n_frames)
{
  mpt::LoadedBag bag;
  bag.camera_models.resize(1);  // single synthetic camera (index 0)
  bag.camera_models[0].fromCameraInfo(make_camera_info());
  const cv::Mat mask = make_mask();
  for (int i = 0; i < n_frames; ++i) {
    mpt::PreparedFrame f;
    f.cam = 0;
    f.stamp_s = 0.5 * i;
    f.mask_rgb8 = mask;
    f.camera_origin = cv::Vec3d(0.0, 0.0, 2.0);
    f.rotation_cam_to_target = down_look_rotation();
    f.boat_x = 0.0;
    f.boat_y = 0.0;
    bag.frames.push_back(f);
  }
  return bag;
}

// A multi-camera bag: n_per frames per camera, interleaved in time (ascending
// stamps), each camera's mask tagged with a distinct obstacle column so
// latestMask is checkable per camera.
mpt::LoadedBag make_multicam_bag(int n_cams, int n_per)
{
  mpt::LoadedBag bag;
  bag.camera_models.resize(n_cams);
  std::vector<cv::Mat> masks;
  for (int c = 0; c < n_cams; ++c) {
    bag.camera_models[c].fromCameraInfo(make_camera_info());
    cv::Mat m(kH, kW, CV_8UC3, cv::Vec3b(0, 200, 0));  // green water
    for (int r = 20; r <= 28; ++r) {
      for (int col = 24 + 2 * c; col <= 30 + 2 * c; ++col) {
        m.at<cv::Vec3b>(r, col) = cv::Vec3b(220, 0, 0);  // distinct obstacle block
      }
    }
    masks.push_back(m);
  }
  double t = 0.0;
  for (int i = 0; i < n_per; ++i) {
    for (int c = 0; c < n_cams; ++c) {
      mpt::PreparedFrame f;
      f.cam = c;
      f.stamp_s = t;
      t += 0.1;
      f.mask_rgb8 = masks[c];
      f.camera_origin = cv::Vec3d(0.0, 0.0, 2.0);
      f.rotation_cam_to_target = down_look_rotation();
      f.boat_x = 0.0;
      f.boat_y = 0.0;
      bag.frames.push_back(f);
    }
  }
  return bag;
}

// Sample the buffer's log-odds on a grid spanning the window.
std::vector<double> sample_grid(const mpt::ReSimEngine & e)
{
  std::vector<double> out;
  for (double wx = -1.5; wx <= 1.5; wx += 0.1) {
    for (double wy = -1.5; wy <= 1.5; wy += 0.1) {
      out.push_back(e.logOddsAt(wx, wy));
    }
  }
  return out;
}

// Two grids equal iff each pair is both-NaN or within tol.
::testing::AssertionResult grids_equal(
  const std::vector<double> & a, const std::vector<double> & b, double tol = 1e-9)
{
  if (a.size() != b.size()) {
    return ::testing::AssertionFailure() << "size " << a.size() << " != " << b.size();
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    const bool na = std::isnan(a[i]), nb = std::isnan(b[i]);
    if (na != nb || (!na && std::abs(a[i] - b[i]) > tol)) {
      return ::testing::AssertionFailure()
        << "differ at " << i << ": " << a[i] << " vs " << b[i];
    }
  }
  return ::testing::AssertionSuccess();
}

std::size_t finite_count(const std::vector<double> & v)
{
  std::size_t n = 0;
  for (double x : v) {
    if (std::isfinite(x)) {
      ++n;
    }
  }
  return n;
}

}  // namespace

// The synthetic geometry must actually accumulate evidence, otherwise every
// other test passes vacuously on an all-NaN buffer.
TEST(ReSimEngine, GeometryAccumulatesEvidence)
{
  const auto bag = make_bag(3);
  mpt::ReSimEngine engine(bag, kWindowM, kRes, kMaxRange);
  engine.seekTo(2);
  EXPECT_GT(finite_count(sample_grid(engine)), 0u)
    << "no cells observed — the test geometry is degenerate, downstream tests are vacuous";
}

TEST(ReSimEngine, Deterministic)
{
  const auto bag = make_bag(4);
  mpt::ReSimEngine a(bag, kWindowM, kRes, kMaxRange);
  mpt::ReSimEngine b(bag, kWindowM, kRes, kMaxRange);
  a.seekTo(3);
  b.seekTo(3);
  EXPECT_TRUE(grids_equal(sample_grid(a), sample_grid(b)));
}

// Reaching a frame by incremental forward stepping must equal reaching it by a
// rewind (clear + full replay) — guards the seekTo branches.
TEST(ReSimEngine, IncrementalEqualsReplay)
{
  const auto bag = make_bag(4);
  mpt::ReSimEngine incremental(bag, kWindowM, kRes, kMaxRange);  // ctor accumulates 0
  incremental.seekTo(1);  // forward 0->1

  mpt::ReSimEngine replay(bag, kWindowM, kRes, kMaxRange);
  replay.seekTo(3);       // forward to 3
  replay.seekTo(1);       // rewind: clear + replay [0,1]

  EXPECT_EQ(incremental.currentIndex(), 1u);
  EXPECT_EQ(replay.currentIndex(), 1u);
  EXPECT_TRUE(grids_equal(sample_grid(incremental), sample_grid(replay)));
}

// A param change re-sims to the current frame; the result must equal an engine
// constructed with that param from the start (no residue — the clear() works).
TEST(ReSimEngine, ParamChangeReSimEqualsFreshConstruction)
{
  const auto bag = make_bag(3);
  sea_surface_segmentation::OccupancyParams changed;  // defaults
  changed.decay_half_life_s = 5.0;  // differs from default 30 → affects accumulation

  mpt::ReSimEngine fresh(bag, kWindowM, kRes, kMaxRange, changed);
  fresh.seekTo(2);

  mpt::ReSimEngine mutated(bag, kWindowM, kRes, kMaxRange);  // default decay
  mutated.seekTo(2);
  std::string why;
  ASSERT_TRUE(mutated.setOccupancyParams(changed, why)) << why;

  EXPECT_TRUE(grids_equal(sample_grid(fresh), sample_grid(mutated)));
}

TEST(ReSimEngine, RejectsInvalidOccupancyParamsWithoutStateChange)
{
  const auto bag = make_bag(3);
  mpt::ReSimEngine engine(bag, kWindowM, kRes, kMaxRange);
  engine.seekTo(2);
  const auto before = sample_grid(engine);

  sea_surface_segmentation::OccupancyParams bad;
  bad.obstacle_clamp = -1.0;  // must be finite and > 0
  std::string why;
  EXPECT_FALSE(engine.setOccupancyParams(bad, why));
  EXPECT_FALSE(why.empty());
  EXPECT_DOUBLE_EQ(engine.occupancyParams().obstacle_clamp, 5.0);  // unchanged default
  EXPECT_TRUE(grids_equal(before, sample_grid(engine)));
}

TEST(ReSimEngine, RejectsInvalidAccumulateParamsWithoutStateChange)
{
  const auto bag = make_bag(3);
  mpt::ReSimEngine engine(bag, kWindowM, kRes, kMaxRange);
  engine.seekTo(2);
  const auto before = sample_grid(engine);
  std::string why;

  auto bad = engine.accumulateParams();
  bad.max_range = -5.0;  // <= 0
  EXPECT_FALSE(engine.setAccumulateParams(bad, why));
  bad = engine.accumulateParams();
  bad.min_grazing_angle_deg = 95.0;  // >= 90
  EXPECT_FALSE(engine.setAccumulateParams(bad, why));
  bad = engine.accumulateParams();
  bad.obstacle_prob_min = 1.5;  // outside [0, 1]
  EXPECT_FALSE(engine.setAccumulateParams(bad, why));
  EXPECT_DOUBLE_EQ(engine.accumulateParams().max_range, kMaxRange);
  EXPECT_TRUE(grids_equal(before, sample_grid(engine)));

  auto good = engine.accumulateParams();
  good.max_range = 80.0;
  good.min_grazing_angle_deg = 5.0;
  EXPECT_TRUE(engine.setAccumulateParams(good, why)) << why;  // valid path works
  EXPECT_DOUBLE_EQ(engine.accumulateParams().max_range, 80.0);
}

// All four (here two) cameras feed one shared buffer, and the per-camera latest
// lookup returns each camera's own mask — the multi-camera fusion + lookup is the
// logic added for the 4-camera tuner, so it gets its own coverage.
TEST(ReSimEngine, FusesMultipleCamerasAndTracksLatestMask)
{
  const auto bag = make_multicam_bag(/*n_cams=*/2, /*n_per=*/3);
  mpt::ReSimEngine e(bag, kWindowM, kRes, kMaxRange);
  e.seekTo(e.frameCount() - 1);

  EXPECT_GT(finite_count(sample_grid(e)), 0u)
    << "no cells observed — neither camera fused into the shared buffer";

  const cv::Mat m0 = e.latestMask(0);
  const cv::Mat m1 = e.latestMask(1);
  ASSERT_FALSE(m0.empty());
  ASSERT_FALSE(m1.empty());
  EXPECT_GT(cv::norm(m0, m1, cv::NORM_L1), 0.0)
    << "latestMask returned the same mask for two different cameras";
}

// A camera with no frames in the bag yields an empty mask (UI shows a
// placeholder) rather than indexing out of range.
TEST(ReSimEngine, LatestMaskEmptyForAbsentCamera)
{
  const auto bag = make_multicam_bag(/*n_cams=*/2, /*n_per=*/2);  // cams 0,1 only
  mpt::ReSimEngine e(bag, kWindowM, kRes, kMaxRange);
  e.seekTo(e.frameCount() - 1);
  EXPECT_TRUE(e.latestMask(3).empty());  // aft camera not present in this bag
}

// Build a bag holding exactly the frames whose stamp lies in [lo, hi] — the
// engine's view of one buffered span. Frame i has stamp 0.5*i (see make_bag).
mpt::LoadedBag slice_bag(int n_frames, double lo, double hi)
{
  const auto full = make_bag(n_frames);
  mpt::LoadedBag out;
  out.camera_models = full.camera_models;
  for (const auto & f : full.frames) {
    if (f.stamp_s >= lo && f.stamp_s <= hi) {
      out.frames.push_back(f);
    }
  }
  return out;
}

// Sample the buffer at an absolute stamp `t` over a bag that contains AT LEAST
// the replay span [t-integration, t]. Seeks to the frame at/just before `t`.
std::vector<double> grid_at(mpt::ReSimEngine & e, double t)
{
  // Seek to the last frame whose stamp <= t (frames are stamp-sorted).
  std::size_t target = 0;
  for (std::size_t i = 0; i < e.frameCount(); ++i) {
    e.seekTo(i);
    if (e.currentStamp() <= t + 1e-9) {
      target = i;
    } else {
      break;
    }
  }
  e.seekTo(target);
  return sample_grid(e);
}

// R1 (determinism): the costmap at a fixed view time `t` must not depend on
// retained frames the buffer happens to hold BEYOND `t`. The retention cache
// (D-series) keeps future frames in memory to avoid re-reads; this test proves
// those un-replayed future frames never leak into the rendering at `t`.
//
// Three bags share the same replay span start (replay_lo = 9) but differ in how
// far past `t` they extend — modelling fresh-load, forward-extend (future frames
// retained ahead of t), and trim-and-reload provenances. Each `grid_at` seeks to
// the frame at/just before t=15, so all three accumulate exactly [9..15]; the
// rendered grids must match. (The complementary property — that the engine's
// START frame is load-bearing, so the manager must hand it [replay_lo, t] — is a
// D4 buffer-manager concern: with this fixed-boat synthetic geometry obstacle
// cells saturate at obstacle_clamp regardless of warm-up length, so the spatial
// start-dependence of R5 can only be exercised at the D4 integration level with a
// moving-boat bag, not here.)
TEST(ReSimEngine, CostmapAtFixedTimeIgnoresRetainedFutureFrames)
{
  // 40 frames at 0.5s spacing → stamps [0, 19.5]. t=15, integration 6s → replay
  // span [9, 15].
  const int n = 40;
  const double t = 15.0;
  const double integration = 6.0;
  const double replay_lo = t - integration;  // 9.0

  // Provenance A — fresh load: bag is exactly the replay span [9, 15].
  auto bag_fresh = slice_bag(n, replay_lo, t);
  // Provenance B — forward extend: a wider cache [9, 17] with retained future
  // frames ahead of t. They must not be accumulated when viewing t.
  auto bag_extend = slice_bag(n, replay_lo, 17.0);
  // Provenance C — trim-and-reload: a different upper extent [9, 15.5].
  auto bag_trim = slice_bag(n, replay_lo, 15.5);

  mpt::ReSimEngine a(bag_fresh, kWindowM, kRes, kMaxRange);
  mpt::ReSimEngine b(bag_extend, kWindowM, kRes, kMaxRange);
  mpt::ReSimEngine c(bag_trim, kWindowM, kRes, kMaxRange);

  const auto ga = grid_at(a, t);
  const auto gb = grid_at(b, t);
  const auto gc = grid_at(c, t);

  EXPECT_GT(finite_count(ga), 0u) << "replay span observed nothing — vacuous";
  EXPECT_TRUE(grids_equal(ga, gb)) << "retained future frames leaked into t";
  EXPECT_TRUE(grids_equal(ga, gc)) << "trim provenance changed the costmap at t";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
