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
#include "fsm_lo/fsm_lo.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <ranges>
#include <utility>

namespace fsm_lo
{

namespace
{

FSM::input_params asInputParams(const Parameters& parameters)
{
  FSM::input_params ip;
  ip.num_iterations = parameters.num_iterations;
  ip.xy_bound = parameters.xy_bound;
  ip.t_bound = parameters.t_bound;
  ip.max_counter = parameters.max_counter;
  ip.min_magnification_size = parameters.min_magnification_size;
  ip.max_magnification_size = parameters.max_magnification_size;
  ip.max_recoveries = parameters.max_recoveries;
  ip.rng_seed = parameters.rng_seed;
  return ip;
}

}  // namespace

/*******************************************************************************
*/
bool isValidRange(const double range)
{
  /*
   * The exponent is inspected directly rather than asking std::isfinite.
   * This package ships compiled with -Ofast, which implies -ffinite-math-only,
   * under which the compiler is entitled to assume no infinity or NaN ever
   * exists and folds std::isfinite to a constant true. The check would then
   * silently do nothing in exactly the build that ships, which is the worst
   * possible outcome for a guard. Reading the bits cannot be assumed away.
   */
  const std::uint64_t bits = std::bit_cast<std::uint64_t>(range);
  const std::uint64_t exponent = (bits >> 52) & 0x7FFU;

  if (exponent == 0x7FFU)
    return false;

  return range > 0.0;
}

/*******************************************************************************
*/
std::string validate(const Parameters& parameters)
{
  if (parameters.size_scan == 0)
    return "size_scan must be greater than zero";

  if (parameters.num_iterations == 0)
    return "num_iterations must be greater than zero";

  if (parameters.xy_bound < 0.0)
    return "xy_bound must not be negative, got "
      + std::to_string(parameters.xy_bound);

  if (parameters.t_bound < 0.0)
    return "t_bound must not be negative, got "
      + std::to_string(parameters.t_bound);

  if (parameters.max_counter == 0)
    return "max_counter must be greater than zero";

  if (parameters.max_magnification_size < parameters.min_magnification_size)
    return "max_magnification_size must not be smaller than "
      "min_magnification_size";

  return {};
}

/*******************************************************************************
*/
Matcher::Matcher(const Parameters& parameters)
: parameters_(parameters),
  forward_plan_(FSM::DFTUtils::forwardPlan(parameters.size_scan)),
  inverse_plan_(FSM::DFTUtils::inversePlan(parameters.size_scan))
{
}

/*******************************************************************************
*/
Pose Matcher::accumulatedPose() const
{
  return Pose{
    accumulated_(0, 2),
    accumulated_(1, 2),
    std::atan2(accumulated_(1, 0), accumulated_(0, 0))};
}

/*******************************************************************************
*/
void Matcher::clearTrajectory()
{
  trajectory_.clear();
  accumulated_ = Eigen::Matrix3d::Identity();
}

/*******************************************************************************
*/
std::expected<MatchResult, MatchError>
Matcher::process(std::span<const double> ranges)
{
  if (ranges.size() < parameters_.size_scan)
    return std::unexpected(MatchError::scan_too_short);

  /*
   * Infinity and not-a-number both mean "no reading" and both destroy a
   * frequency transform outright: one such ray contaminates every coefficient.
   * They are normalised to zero here, which is the form the gap filling below
   * already understands, so all three ways a driver can say "nothing here" are
   * treated alike.
   */
  std::vector<double> scan(ranges.begin(), ranges.end());
  std::ranges::replace_if(scan,
    [](const double range) { return !isValidRange(range); }, 0.0);

  if (std::ranges::none_of(scan, isValidRange))
    return std::unexpected(MatchError::scan_entirely_invalid);

  scan = FSM::DatasetUtils::interpolateRanges(scan);
  scan = FSM::Utils::subsampleScan(scan, parameters_.size_scan);

  scans_seen_++;

  if (reference_scan_.empty())
  {
    reference_scan_ = std::move(scan);
    return std::unexpected(MatchError::no_reference_yet);
  }

  const Pose origin;

  const std::vector<std::pair<double, double>> reference_points =
    FSM::Utils::scan2points(reference_scan_, origin);

  FSM::output_params op;
  Pose increment;
  FSM::Match::fmtdbh(scan, origin, reference_points, forward_plan_,
    inverse_plan_, asInputParams(parameters_), &op, &increment);

  accumulated_ = FSM::Utils::computeTransform(increment, accumulated_);
  trajectory_.push_back(accumulatedPose());

  reference_scan_ = std::move(scan);

  return MatchResult{
    increment,
    accumulatedPose(),
    op.exec_time,
    op.num_recoveries};
}

/*******************************************************************************
*/
void Matcher::setInitialPose(const Pose& pose)
{
  accumulated_ =
    FSM::Utils::computeTransform(pose, Eigen::Matrix3d::Identity());
}

}  // namespace fsm_lo
