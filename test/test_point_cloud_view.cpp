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

// Offscreen smoke test for the MBES 3D point-cloud viewport: a cloud of synthetic
// soundings must render some points (non-background pixels) and clear() must blank
// it. Mode/Z-exaggeration setters must not crash. A real GL context is required;
// self-skips when none is available, matching the shared-widget tests.

#include <gtest/gtest.h>

#include <QApplication>
#include <QColor>
#include <QCoreApplication>
#include <QImage>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QSurfaceFormat>

#include <cmath>
#include <memory>
#include <vector>

#include "mbes_geometry.hpp"
#include "point_cloud_view.hpp"

using marine_perception_tools::MbesSounding;
using marine_perception_tools::PointCloudView;

namespace
{
constexpr int kW = 300;
constexpr int kH = 300;

bool gl_available()
{
  QSurfaceFormat fmt;
  fmt.setRenderableType(QSurfaceFormat::OpenGL);
  fmt.setVersion(3, 3);
  QOffscreenSurface surface;
  surface.setFormat(fmt);
  surface.create();
  if (!surface.isValid()) {
    return false;
  }
  QOpenGLContext ctx;
  ctx.setFormat(fmt);
  if (!ctx.create() || !ctx.makeCurrent(&surface)) {
    return false;
  }
  const QSurfaceFormat got = ctx.format();
  const bool ok =
    got.majorVersion() > 3 || (got.majorVersion() == 3 && got.minorVersion() >= 3);
  ctx.doneCurrent();
  return ok;
}

// A small grid of soundings around a world point, with a depth ramp.
std::vector<MbesSounding> make_cloud()
{
  std::vector<MbesSounding> pts;
  for (int i = -10; i <= 10; ++i) {
    for (int j = -10; j <= 10; ++j) {
      MbesSounding s;
      s.x = 500.0 + i;
      s.y = 1000.0 + j;
      s.z = 45.0 + 0.05 * (i + j);
      s.intensity = -20.0f - static_cast<float>(i);
      pts.push_back(s);
    }
  }
  return pts;
}

// Count pixels that differ from the dark background (a rendered point).
int non_background(const QImage & img)
{
  int n = 0;
  for (int y = 0; y < img.height(); ++y) {
    for (int x = 0; x < img.width(); ++x) {
      const QColor c = img.pixelColor(x, y);
      if (c.red() > 40 || c.green() > 40 || c.blue() > 50) {
        ++n;
      }
    }
  }
  return n;
}

class PointCloudViewTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (QCoreApplication::instance() == nullptr) {
      static int argc = 1;
      static char arg0[] = "test_point_cloud_view";
      static char * argv[] = {arg0, nullptr};
      app_ = std::make_unique<QApplication>(argc, argv);
    }
    if (!gl_available()) {
      GTEST_SKIP() << "No OpenGL 3.3 context available (headless without software GL).";
    }
  }
  std::unique_ptr<QApplication> app_;
};

// A cloud renders some points; clearing blanks the view.
TEST_F(PointCloudViewTest, RendersAndClears)
{
  PointCloudView view;
  view.resize(kW, kH);
  view.setPoints(make_cloud());
  const QImage with = view.grabFramebuffer();
  ASSERT_FALSE(with.isNull());
  EXPECT_GT(non_background(with), 0) << "a non-empty cloud should render points";

  view.clear();
  const QImage empty = view.grabFramebuffer();
  EXPECT_EQ(non_background(empty), 0) << "clear() should blank the view";
}

// The mode / Z-exaggeration / colormap setters must not crash and still render.
TEST_F(PointCloudViewTest, SettersAreSafe)
{
  PointCloudView view;
  view.resize(kW, kH);
  view.setPoints(make_cloud());
  view.setColorMode(PointCloudView::ColorMode::Backscatter);
  view.setZExaggeration(8.0f);
  view.setColorMap(0);
  const QImage img = view.grabFramebuffer();
  ASSERT_FALSE(img.isNull());
  EXPECT_GT(non_background(img), 0);
}

// Setting an empty cloud is safe and renders nothing.
TEST_F(PointCloudViewTest, EmptyIsSafe)
{
  PointCloudView view;
  view.resize(kW, kH);
  view.setPoints({});
  const QImage img = view.grabFramebuffer();
  ASSERT_FALSE(img.isNull());
  EXPECT_EQ(non_background(img), 0);
}

// Two passes in ColorMode::Pass render with DISTINCT per-pass hues — pass 0 is
// red-dominant, pass 1 green-dominant (golden-angle palette). Asserting both
// hue families appear catches a regression where Pass mode is ignored or every
// point lands in pass 0 (#21).
TEST_F(PointCloudViewTest, MultiPassColour)
{
  PointCloudView view;
  view.resize(kW, kH);
  view.setColorMode(PointCloudView::ColorMode::Pass);
  auto pass_a = make_cloud();
  auto pass_b = make_cloud();
  for (auto & s : pass_b) {
    s.y += 25.0;   // place the repeat pass beside the first so both are visible
  }
  view.setMultiPassPoints({pass_a, pass_b});
  const QImage img = view.grabFramebuffer();
  ASSERT_FALSE(img.isNull());
  int red_dominant = 0;
  int green_dominant = 0;
  for (int y = 0; y < img.height(); ++y) {
    for (int x = 0; x < img.width(); ++x) {
      const QColor c = img.pixelColor(x, y);
      if (c.red() > 100 && c.red() > 2 * c.green() && c.red() > 2 * c.blue()) {
        ++red_dominant;    // pass_color(0) ≈ (0.90, 0.14, 0.14)
      } else if (c.green() > 100 && c.green() > 2 * c.red()) {
        ++green_dominant;  // pass_color(1) ≈ (0.14, 0.90, 0.36)
      }
    }
  }
  EXPECT_GT(red_dominant, 0) << "pass 0 (red-dominant) pixels should render";
  EXPECT_GT(green_dominant, 0) << "pass 1 (green-dominant) pixels should render";
}

// Switching to Pass mode on a single-pass cloud is safe (everything is pass 0).
TEST_F(PointCloudViewTest, ColorModePassOnSinglePassIsSafe)
{
  PointCloudView view;
  view.resize(kW, kH);
  view.setPoints(make_cloud());
  view.setColorMode(PointCloudView::ColorMode::Pass);
  const QImage img = view.grabFramebuffer();
  ASSERT_FALSE(img.isNull());
  EXPECT_GT(non_background(img), 0);
}

// The golden-angle pass palette is pure and stable: pinned values + distinct
// leading hues (legend colours must not drift between releases).
TEST(PassColor, StableAndDistinct)
{
  float r0, g0, b0;
  marine_perception_tools::pass_color(0, r0, g0, b0);
  // Index 0 is hue 0 (red-ish) at s=0.85, v=0.9: rgb = (0.9, 0.135, 0.135).
  EXPECT_NEAR(r0, 0.9f, 1e-4);
  EXPECT_NEAR(g0, 0.135f, 1e-4);
  EXPECT_NEAR(b0, 0.135f, 1e-4);
  // The first ten pass colours are pairwise distinguishable.
  for (int i = 0; i < 10; ++i) {
    for (int j = i + 1; j < 10; ++j) {
      float ri, gi, bi, rj, gj, bj;
      marine_perception_tools::pass_color(i, ri, gi, bi);
      marine_perception_tools::pass_color(j, rj, gj, bj);
      const float d =
        std::abs(ri - rj) + std::abs(gi - gj) + std::abs(bi - bj);
      EXPECT_GT(d, 0.1f) << "passes " << i << " and " << j << " too similar";
    }
  }
}
}  // namespace
