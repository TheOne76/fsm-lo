// Copyright 2022 Alexandros Filotheou
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*
 * Where a ray leaving a pose meets the walls of a room.
 *
 * Everything the matcher does rests on this: a scan is turned into points, the
 * points are turned back into ranges from a hypothesised pose, and the two are
 * compared. If the intersection is wrong then every stage above it is working
 * from fiction, and nothing further up would say so clearly.
 *
 * The expected answers here are worked out by hand rather than recorded from a
 * previous run, so they check the geometry rather than check that it has not
 * changed.
 *
 * Ray i of a scan of n rays leaves at i * 2*pi/n + pose orientation - pi. Ray
 * zero therefore points backwards along the pose's own axis, not forwards, and
 * the cases below are indexed with that in mind.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "fsm_lo/fsm_lo.hpp"

namespace
{

/* Intersections are exact for these rooms, so the bar is machine precision. */
const double kExact = 1e-12;

/*
 * Except when a ray meets a wall perpendicular to the x-axis at an angle. That
 * case is computed from a point placed a hundred million metres along the ray,
 * and the final subtraction cancels almost all of it away, costing about eight
 * digits. The answer is good to a few parts in a thousand million, no better.
 * The equivalence bar is the same order, which is close enough to be worth
 * knowing about.
 */
const double kFarPointCancellation = 1e-8;

using Point = std::pair<double, double>;
using Room = std::vector<Point>;

/* Counter-clockwise, which is what the ray casting assumes. */
Room rectangle(const double half_width, const double half_height)
{
  return Room{
    {-half_width, -half_height},
    {half_width, -half_height},
    {half_width, half_height},
    {-half_width, half_height}};
}

void expectPoint(const Point& got, const double x, const double y,
  const std::string& which)
{
  EXPECT_NEAR(got.first, x, kExact) << which << ", x";
  EXPECT_NEAR(got.second, y, kExact) << which << ", y";
}

double distance(const Point& from, const Point& to)
{
  const double dx = to.first - from.first;
  const double dy = to.second - from.second;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

/*
 * The four rays of a four ray scan from the centre of a square leave along the
 * axes, so each meets the middle of one wall. This is also the case that
 * exercises the branch for a ray perpendicular to the x-axis, where the
 * gradient of the ray is infinite and the general formula cannot be used.
 */
TEST(Geometry, AxisAlignedRaysFromTheCentreOfASquare)
{
  const Room room = rectangle(4.0, 4.0);
  const std::vector<Point> hits = FSM::X::find(FSM::Pose{}, room, 4);

  ASSERT_EQ(hits.size(), 4u);
  expectPoint(hits[0], -4.0, 0.0, "ray at -pi");
  expectPoint(hits[1], 0.0, -4.0, "ray at -pi/2");
  expectPoint(hits[2], 4.0, 0.0, "ray at 0");
  expectPoint(hits[3], 0.0, 4.0, "ray at pi/2");
}

/*
 * A pose close to a wall sees that wall very near and the opposite wall very
 * far. Both distances are still exact, and the near one is where a sign error
 * or an off by one in the segment search would show up first.
 */
TEST(Geometry, APoseCloseToAWall)
{
  const Room room = rectangle(4.0, 4.0);
  const FSM::Pose pose{3.5, 0.0, 0.0};
  const std::vector<Point> hits = FSM::X::find(pose, room, 4);

  ASSERT_EQ(hits.size(), 4u);
  expectPoint(hits[0], -4.0, 0.0, "away from the near wall");
  expectPoint(hits[2], 4.0, 0.0, "into the near wall");

  EXPECT_NEAR(distance({pose.x, pose.y}, hits[2]), 0.5, kExact);
  EXPECT_NEAR(distance({pose.x, pose.y}, hits[0]), 7.5, kExact);
}

/*
 * A ray aimed exactly at a corner meets two wall segments at the same point.
 * Either may be reported, and the point must be the corner in both cases: this
 * is the case where a search that stops at the first segment it likes and one
 * that keeps the nearest can disagree.
 */
TEST(Geometry, ARayThroughACorner)
{
  const Room room = rectangle(4.0, 4.0);
  const std::vector<Point> hits = FSM::X::find(FSM::Pose{}, room, 8);

  ASSERT_EQ(hits.size(), 8u);
  expectPoint(hits[1], -4.0, -4.0, "ray at -3pi/4");
  expectPoint(hits[3], 4.0, -4.0, "ray at -pi/4");
  expectPoint(hits[5], 4.0, 4.0, "ray at pi/4");
  expectPoint(hits[7], -4.0, 4.0, "ray at 3pi/4");
}

/*
 * A room that is not square and a ray that is not aligned with anything. The
 * expected point is where the line y = x*tan(pi/6) leaves a room eight wide and
 * five tall: it reaches x = 4 while y is still under 2.5, so it meets the right
 * hand wall rather than the top one.
 */
TEST(Geometry, AnObliqueRayInARectangularRoom)
{
  const Room room = rectangle(4.0, 2.5);
  const std::vector<Point> hits = FSM::X::find(FSM::Pose{}, room, 12);

  ASSERT_EQ(hits.size(), 12u);

  const double expected_y = 4.0 * std::tan(M_PI / 6);
  ASSERT_LT(expected_y, 2.5) << "the case is only oblique if it clears the top";

  EXPECT_NEAR(hits[7].first, 4.0, kExact) << "ray at pi/6, x";
  EXPECT_NEAR(hits[7].second, expected_y, kFarPointCancellation)
    << "ray at pi/6, y";
}

/*
 * The pose's orientation turns the whole fan of rays with it. Turning the pose
 * by exactly one ray's worth must give the same set of points, shifted by one
 * place, since the rays then leave along the same directions as before.
 */
TEST(Geometry, TurningThePoseByOneRayShiftsTheIntersections)
{
  const Room room = rectangle(4.0, 2.5);
  const unsigned int rays = 16;

  const std::vector<Point> straight = FSM::X::find(FSM::Pose{}, room, rays);
  const std::vector<Point> turned =
    FSM::X::find(FSM::Pose{0.0, 0.0, 2 * M_PI / rays}, room, rays);

  ASSERT_EQ(straight.size(), rays);
  ASSERT_EQ(turned.size(), rays);

  for (unsigned int i = 0; i < rays - 1; i++)
    expectPoint(turned[i], straight[i + 1].first, straight[i + 1].second,
      "ray " + std::to_string(i));
}

/*
 * Two implementations of the same thing live side by side: one walks every
 * segment for every ray, the other narrows the search using the segment the
 * previous ray met. Only the second runs.
 *
 * In a room whose walls turn only one way they agree exactly, from anywhere
 * inside and at any orientation. In a room with a corner that turns back on
 * itself they do not always, because the segment a ray meets stops advancing
 * with the ray, and the narrowed search can settle for a wall that stands
 * behind the nearest one. That is recorded as known and unfixed rather than
 * asserted here, since fixing it would move the numbers.
 */
TEST(Geometry, TheWindowedSearchAgreesWithTheExhaustiveOneInAConvexRoom)
{
  const Room room = rectangle(4.0, 2.5);

  for (const FSM::Pose& pose : {FSM::Pose{}, FSM::Pose{1.5, -0.5, 0.7},
      FSM::Pose{-2.0, 1.0, -2.2}, FSM::Pose{3.9, 2.4, 3.0}})
  {
    const std::vector<Point> windowed = FSM::X::findExact2(pose, room, 90);
    const std::vector<Point> exhaustive = FSM::X::findExact(pose, room, 90);

    ASSERT_EQ(windowed.size(), exhaustive.size());
    for (std::size_t i = 0; i < windowed.size(); i++)
      expectPoint(windowed[i], exhaustive[i].first, exhaustive[i].second,
        "ray " + std::to_string(i));
  }
}

/*
 * Ranges and points are two spellings of the same thing, so converting one way
 * and back must return what went in. This is the pair of conversions every
 * iteration of the matcher performs, twice.
 */
TEST(Geometry, PointsAndRangesRoundTrip)
{
  const Room room = rectangle(4.0, 2.5);
  const FSM::Pose pose{1.25, -0.75, 0.4};

  const std::vector<double> ranges =
    FSM::Utils::points2scan(FSM::X::find(pose, room, 180), pose);

  ASSERT_EQ(ranges.size(), 180u);

  const std::vector<Point> back = FSM::Utils::scan2points(ranges, pose);
  const std::vector<double> again = FSM::Utils::points2scan(back, pose);

  ASSERT_EQ(again.size(), ranges.size());
  for (std::size_t i = 0; i < ranges.size(); i++)
    EXPECT_NEAR(again[i], ranges[i], kExact) << "ray " << i;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
