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
 * Scans carrying rays the sensor could not measure.
 *
 * A driver reports such a ray as zero, as infinity, or as not-a-number,
 * depending on its conventions. Only zero was recognised before; the other two
 * went into the frequency transform as though they were distances, and a
 * single one of them contaminates every coefficient the transform produces.
 *
 * The property these tests pin is that all three forms mean the same thing.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "fsm_lo/fsm_lo.hpp"

namespace
{

const std::size_t kSize = 360;
const double kInfinity = std::numeric_limits<double>::infinity();
const double kNotANumber = std::numeric_limits<double>::quiet_NaN();

std::vector<double> roomScan(const double x, const double y)
{
  std::vector<double> ranges(kSize);
  for (std::size_t i = 0; i < kSize; i++)
  {
    const double angle = i * 2.0 * M_PI / kSize - M_PI;
    double nearest = 1e9;
    const double walls[4][3] = {
      {x, -1.0, 0.0}, {8.0 - x, 1.0, 0.0},
      {y, 0.0, -1.0}, {5.0 - y, 0.0, 1.0}};

    for (const auto& wall : walls)
    {
      const double projection =
        std::cos(angle) * wall[1] + std::sin(angle) * wall[2];
      if (projection > 1e-9)
        nearest = std::min(nearest, wall[0] / projection);
    }
    ranges[i] = nearest;
  }
  return ranges;
}

fsm_lo::Parameters parameters()
{
  fsm_lo::Parameters p;
  p.size_scan = kSize;
  return p;
}

fsm_lo::Pose matchWith(std::vector<double> second)
{
  fsm_lo::Matcher matcher(parameters());
  const auto first = matcher.process(roomScan(3.0, 2.5));
  EXPECT_FALSE(first.has_value());

  const auto result = matcher.process(second);
  EXPECT_TRUE(result.has_value());
  return result.has_value() ? result->increment : fsm_lo::Pose{};
}

}  // namespace

TEST(InvalidRanges, ZeroInfinityAndNotANumberAreAllTreatedAsNoReading)
{
  std::vector<double> zeros = roomScan(3.05, 2.5);
  std::vector<double> infinities = zeros;
  std::vector<double> nans = zeros;

  for (std::size_t i = 100; i < 112; i++)
  {
    zeros[i] = 0.0;
    infinities[i] = kInfinity;
    nans[i] = kNotANumber;
  }

  const fsm_lo::Pose from_zeros = matchWith(zeros);
  const fsm_lo::Pose from_infinities = matchWith(infinities);
  const fsm_lo::Pose from_nans = matchWith(nans);

  EXPECT_DOUBLE_EQ(from_infinities.x, from_zeros.x);
  EXPECT_DOUBLE_EQ(from_infinities.y, from_zeros.y);
  EXPECT_DOUBLE_EQ(from_infinities.t, from_zeros.t);

  EXPECT_DOUBLE_EQ(from_nans.x, from_zeros.x);
  EXPECT_DOUBLE_EQ(from_nans.y, from_zeros.y);
  EXPECT_DOUBLE_EQ(from_nans.t, from_zeros.t);
}

TEST(InvalidRanges, AnInfiniteRayDoesNotWreckTheMatch)
{
  const std::vector<double> clean = roomScan(3.05, 2.5);

  std::vector<double> with_infinity = clean;
  with_infinity[200] = kInfinity;

  const fsm_lo::Pose expected = matchWith(clean);
  const fsm_lo::Pose actual = matchWith(with_infinity);

  EXPECT_TRUE(std::isfinite(actual.x));
  EXPECT_TRUE(std::isfinite(actual.y));
  EXPECT_TRUE(std::isfinite(actual.t));

  EXPECT_NEAR(actual.x, expected.x, 1e-3);
  EXPECT_NEAR(actual.y, expected.y, 1e-3);
  EXPECT_NEAR(actual.t, expected.t, 1e-3);
}

TEST(InvalidRanges, ANegativeRangeIsTreatedAsNoReading)
{
  std::vector<double> zeros = roomScan(3.05, 2.5);
  std::vector<double> negatives = zeros;

  for (std::size_t i = 40; i < 48; i++)
  {
    zeros[i] = 0.0;
    negatives[i] = -1.0;
  }

  const fsm_lo::Pose from_zeros = matchWith(zeros);
  const fsm_lo::Pose from_negatives = matchWith(negatives);

  EXPECT_DOUBLE_EQ(from_negatives.x, from_zeros.x);
  EXPECT_DOUBLE_EQ(from_negatives.y, from_zeros.y);
}

TEST(InvalidRanges, AnEntirelyInvalidScanIsRefused)
{
  fsm_lo::Matcher matcher(parameters());

  const std::vector<double> all_infinite(kSize, kInfinity);
  const auto result = matcher.process(all_infinite);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), fsm_lo::MatchError::scan_entirely_invalid);
}

TEST(InvalidRanges, AnEntirelyInvalidScanDoesNotBecomeTheReference)
{
  fsm_lo::Matcher matcher(parameters());

  ASSERT_FALSE(matcher.process(roomScan(3.0, 2.5)).has_value());

  const std::vector<double> all_zero(kSize, 0.0);
  ASSERT_FALSE(matcher.process(all_zero).has_value());

  const auto result = matcher.process(roomScan(3.05, 2.5));
  ASSERT_TRUE(result.has_value())
    << "the good scan before the invalid one should still be the reference";
}

TEST(InvalidRanges, IsValidRangeAgreesWithItsDocumentation)
{
  EXPECT_TRUE(fsm_lo::isValidRange(1.0));
  EXPECT_FALSE(fsm_lo::isValidRange(0.0));
  EXPECT_FALSE(fsm_lo::isValidRange(-1.0));
  EXPECT_FALSE(fsm_lo::isValidRange(kInfinity));
  EXPECT_FALSE(fsm_lo::isValidRange(-kInfinity));
  EXPECT_FALSE(fsm_lo::isValidRange(kNotANumber));
}
