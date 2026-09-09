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
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "coastline_data.hpp"

namespace
{

using marine_perception_tools::Coastline;
using marine_perception_tools::coastlineFadeAlpha;
using marine_perception_tools::decimatePolyline;
using marine_perception_tools::kCoastlineFadeFloorMpp;
using marine_perception_tools::kCoastlineFullScaleMpp;
using marine_perception_tools::loadCoastline;
using marine_perception_tools::parseCoastline;

Coastline parse(const std::string & text, double tolerance_deg = 0.0)
{
  std::istringstream in(text);
  return parseCoastline(in, tolerance_deg);
}

// --- format -----------------------------------------------------------------

TEST(ParseCoastline, ReadsAbsoluteStartAndDeltas)
{
  const auto c = parse(
    "# a comment\n"
    "\n"
    "> 43000 -70750\n"
    "10 -20\n"
    "-5 30\n");
  ASSERT_EQ(c.lines.size(), 1u);
  const auto & pts = c.lines[0].points;
  ASSERT_EQ(pts.size(), 3u);
  EXPECT_DOUBLE_EQ(pts[0].first, 43.0);
  EXPECT_DOUBLE_EQ(pts[0].second, -70.75);
  EXPECT_DOUBLE_EQ(pts[1].first, 43.01);
  EXPECT_DOUBLE_EQ(pts[1].second, -70.77);
  EXPECT_DOUBLE_EQ(pts[2].first, 43.005);
  EXPECT_DOUBLE_EQ(pts[2].second, -70.74);
  EXPECT_EQ(c.pointCount(), 3u);
  EXPECT_FALSE(c.empty());
}

TEST(ParseCoastline, ComputesBoundingBoxPerLine)
{
  const auto c = parse(
    "> 43000 -70750\n"
    "10 -20\n"
    "-5 30\n");
  ASSERT_EQ(c.lines.size(), 1u);
  const auto & line = c.lines[0];
  EXPECT_DOUBLE_EQ(line.south, 43.0);
  EXPECT_DOUBLE_EQ(line.north, 43.01);
  EXPECT_DOUBLE_EQ(line.west, -70.77);
  EXPECT_DOUBLE_EQ(line.east, -70.74);
}

TEST(ParseCoastline, SeparatesPolylinesAndToleratesCarriageReturns)
{
  const auto c = parse(
    "> 10000 20000\r\n"
    "100 0\r\n"
    "> -10000 -20000\r\n"
    "0 100\r\n");
  ASSERT_EQ(c.lines.size(), 2u);
  EXPECT_DOUBLE_EQ(c.lines[0].points[1].first, 10.1);
  EXPECT_DOUBLE_EQ(c.lines[1].points[1].second, -19.9);
}

// --- degradation ------------------------------------------------------------

TEST(ParseCoastline, EmptyInputYieldsNoCoastline)
{
  const auto c = parse("");
  EXPECT_TRUE(c.empty());
  EXPECT_EQ(c.pointCount(), 0u);
}

TEST(ParseCoastline, HeaderOnlyInputYieldsNoCoastline)
{
  EXPECT_TRUE(parse("# marine_perception_tools coastline v1\n# nothing else\n").empty());
}

TEST(ParseCoastline, SinglePointPolylineIsDropped)
{
  EXPECT_TRUE(parse("> 43000 -70750\n").empty());
}

TEST(ParseCoastline, JunkYieldsNoCoastlineRatherThanACrash)
{
  EXPECT_TRUE(parse("this is not a coastline\nnor is this\n\x01\x02\n").empty());
  EXPECT_TRUE(parse("> not numbers\n1 2\n").empty());
}

TEST(ParseCoastline, PointsBeforeAnyPolylineStartAreIgnored)
{
  const auto c = parse(
    "10 10\n"
    "> 1000 2000\n"
    "5 5\n");
  ASSERT_EQ(c.lines.size(), 1u);
  EXPECT_EQ(c.lines[0].points.size(), 2u);
}

TEST(ParseCoastline, MalformedRecordDropsOnlyItsOwnPolyline)
{
  // A lost delta shifts every later point of that polyline, so the whole
  // polyline goes — but the file keeps loading at the next ">".
  const auto c = parse(
    "> 1000 2000\n"
    "5 5\n"
    "5 oops\n"
    "5 5\n"
    "> 30000 40000\n"
    "1 1\n");
  ASSERT_EQ(c.lines.size(), 1u);
  EXPECT_DOUBLE_EQ(c.lines[0].points[0].first, 30.0);
}

TEST(ParseCoastline, TrailingGarbageOnARecordIsMalformed)
{
  EXPECT_TRUE(parse("> 1000 2000 3000\n5 5\n").empty());
  EXPECT_TRUE(parse("> 1000 2000\n5 5 5\n").empty());
}

TEST(ParseCoastline, OutOfRangePositionsAreRejected)
{
  EXPECT_TRUE(parse("> 91000 0\n1 1\n").empty());
  EXPECT_TRUE(parse("> 0 181000\n1 1\n").empty());
  // A delta that walks off the earth drops the polyline it belongs to.
  const auto c = parse(
    "> 89000 0\n"
    "2000 0\n"
    "> 10000 10000\n"
    "1 1\n");
  ASSERT_EQ(c.lines.size(), 1u);
  EXPECT_DOUBLE_EQ(c.lines[0].points[0].first, 10.0);
}

TEST(LoadCoastline, MissingFileYieldsNoCoastline)
{
  EXPECT_TRUE(loadCoastline("/nonexistent/coastline/file.txt").empty());
}

TEST(LoadCoastline, ReadsAFileFromDisk)
{
  const auto path = std::filesystem::temp_directory_path() /
    "mpt_test_coastline.txt";
  {
    std::ofstream out(path);
    out << "# header\n> 43000 -70750\n10 10\n";
  }
  const auto c = loadCoastline(path.string());
  ASSERT_EQ(c.lines.size(), 1u);
  EXPECT_EQ(c.lines[0].points.size(), 2u);
  std::filesystem::remove(path);
}

// --- decimation -------------------------------------------------------------

TEST(DecimatePolyline, DropsCollinearInteriorPoints)
{
  const std::vector<std::pair<double, double>> line{
    {0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}, {0.0, 3.0}};
  const auto out = decimatePolyline(line, 0.001);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_DOUBLE_EQ(out.front().second, 0.0);
  EXPECT_DOUBLE_EQ(out.back().second, 3.0);
}

TEST(DecimatePolyline, KeepsDeviationsLargerThanTheTolerance)
{
  const std::vector<std::pair<double, double>> line{
    {0.0, 0.0}, {0.5, 1.0}, {0.0, 2.0}};
  const auto kept = decimatePolyline(line, 0.1);
  EXPECT_EQ(kept.size(), 3u);
  const auto dropped = decimatePolyline(line, 1.0);
  EXPECT_EQ(dropped.size(), 2u);
}

TEST(DecimatePolyline, AlwaysKeepsEndpointsAndOrder)
{
  const std::vector<std::pair<double, double>> line{
    {10.0, 20.0}, {10.1, 20.05}, {10.2, 20.1}, {11.0, 21.0}};
  const auto out = decimatePolyline(line, 5.0);   // absurdly coarse
  ASSERT_EQ(out.size(), 2u);
  EXPECT_DOUBLE_EQ(out.front().first, 10.0);
  EXPECT_DOUBLE_EQ(out.back().first, 11.0);
}

TEST(DecimatePolyline, ZeroToleranceAndShortInputsAreIdentity)
{
  const std::vector<std::pair<double, double>> line{
    {0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}};
  EXPECT_EQ(decimatePolyline(line, 0.0).size(), 3u);
  const std::vector<std::pair<double, double>> two{{0.0, 0.0}, {1.0, 1.0}};
  EXPECT_EQ(decimatePolyline(two, 10.0).size(), 2u);
  EXPECT_TRUE(decimatePolyline({}, 1.0).empty());
}

TEST(DecimatePolyline, ClosedRingKeepsItsShape)
{
  // First and last point coincide: the split cannot use a segment direction,
  // and must not collapse the ring to its (degenerate) chord.
  const std::vector<std::pair<double, double>> ring{
    {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, {0.0, 0.0}};
  const auto out = decimatePolyline(ring, 0.1);
  EXPECT_EQ(out.size(), 5u);
}

TEST(ParseCoastline, AppliesTheDecimationTolerance)
{
  const std::string text =
    "> 0 0\n"
    "0 1\n"
    "0 1\n"
    "0 1\n";   // four collinear points, 1 milli-degree apart
  EXPECT_EQ(parse(text, 0.0).lines[0].points.size(), 4u);
  EXPECT_EQ(parse(text, 0.001).lines[0].points.size(), 2u);
}

// --- the scale rule ---------------------------------------------------------

TEST(CoastlineFadeAlpha, IsFullAtCollectionZoom)
{
  EXPECT_DOUBLE_EQ(coastlineFadeAlpha(1000.0), 1.0);
  EXPECT_DOUBLE_EQ(coastlineFadeAlpha(kCoastlineFullScaleMpp), 1.0);
}

TEST(CoastlineFadeAlpha, IsZeroAtSurveyZoom)
{
  // The contract that keeps this layer from reading as chart detail: at the
  // scales where the real data layers answer the question, it is not drawn.
  EXPECT_DOUBLE_EQ(coastlineFadeAlpha(0.1), 0.0);
  EXPECT_DOUBLE_EQ(coastlineFadeAlpha(1.0), 0.0);
  EXPECT_DOUBLE_EQ(coastlineFadeAlpha(10.0), 0.0);
  EXPECT_DOUBLE_EQ(coastlineFadeAlpha(kCoastlineFadeFloorMpp), 0.0);
}

TEST(CoastlineFadeAlpha, FadesMonotonicallyBetween)
{
  double previous = 0.0;
  for (double mpp = kCoastlineFadeFloorMpp; mpp <= kCoastlineFullScaleMpp; mpp += 1.0) {
    const double a = coastlineFadeAlpha(mpp);
    EXPECT_GE(a, previous);
    EXPECT_GE(a, 0.0);
    EXPECT_LE(a, 1.0);
    previous = a;
  }
  EXPECT_GT(coastlineFadeAlpha(60.0), 0.0);
  EXPECT_LT(coastlineFadeAlpha(60.0), 1.0);
}

TEST(CoastlineFadeAlpha, RejectsNonsenseScales)
{
  EXPECT_DOUBLE_EQ(coastlineFadeAlpha(0.0), 0.0);
  EXPECT_DOUBLE_EQ(coastlineFadeAlpha(-5.0), 0.0);
  // Not-a-number and infinity are broken view state, not a zoom level: the
  // layer stays off rather than painting a world coastline over whatever the
  // operator is actually looking at.
  EXPECT_DOUBLE_EQ(coastlineFadeAlpha(std::nan("")), 0.0);
  EXPECT_DOUBLE_EQ(coastlineFadeAlpha(std::numeric_limits<double>::infinity()), 0.0);
}

}  // namespace
