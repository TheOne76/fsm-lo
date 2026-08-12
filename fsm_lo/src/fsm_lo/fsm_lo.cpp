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

#include <cmath>
#include <tuple>
#include <utility>

namespace fsm_lo
{

namespace
{

std::tuple<double, double, double> asTuple(const Pose& pose)
{
  return std::make_tuple(pose.x, pose.y, pose.t);
}

Pose asPose(const std::tuple<double, double, double>& tuple)
{
  return Pose{std::get<0>(tuple), std::get<1>(tuple), std::get<2>(tuple)};
}

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
  return ip;
}

}  // namespace

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
: parameters_(parameters)
{
  const std::size_t size = parameters_.size_scan;

  double* forward_in = static_cast<double*>(fftw_malloc(size * sizeof(double)));
  double* forward_out = static_cast<double*>(fftw_malloc(size * sizeof(double)));
  forward_plan_ = fftw_plan_r2r_1d(static_cast<int>(size), forward_in,
    forward_out, FFTW_R2HC, FSM_LO_FFTW_PLAN_FLAG);

  fftw_complex* inverse_in =
    static_cast<fftw_complex*>(fftw_malloc(size * sizeof(fftw_complex)));
  double* inverse_out = static_cast<double*>(fftw_malloc(size * sizeof(double)));
  inverse_plan_ = fftw_plan_dft_c2r_1d(static_cast<int>(size), inverse_in,
    inverse_out, FSM_LO_FFTW_PLAN_FLAG);

  fftw_free(forward_in);
  fftw_free(forward_out);
  fftw_free(inverse_in);
  fftw_free(inverse_out);
}

/*******************************************************************************
*/
Matcher::~Matcher()
{
  if (forward_plan_ != nullptr)
    fftw_destroy_plan(forward_plan_);

  if (inverse_plan_ != nullptr)
    fftw_destroy_plan(inverse_plan_);

  fftw_cleanup();
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

  std::vector<double> scan =
    FSM::DatasetUtils::interpolateRanges(
      std::vector<double>(ranges.begin(), ranges.end()));
  scan = FSM::Utils::subsampleScan(scan, parameters_.size_scan);

  scans_seen_++;

  if (reference_scan_.empty())
  {
    reference_scan_ = std::move(scan);
    return std::unexpected(MatchError::no_reference_yet);
  }

  const std::tuple<double, double, double> origin(0.0, 0.0, 0.0);

  std::vector<std::pair<double, double>> reference_points;
  FSM::Utils::scan2points(reference_scan_, origin, &reference_points);

  FSM::output_params op;
  std::tuple<double, double, double> increment;
  FSM::Match::fmtdbh(scan, origin, reference_points, forward_plan_,
    inverse_plan_, asInputParams(parameters_), &op, &increment);

  accumulated_ = FSM::Utils::computeTransform(increment, accumulated_);
  trajectory_.push_back(accumulatedPose());

  reference_scan_ = std::move(scan);

  return MatchResult{
    asPose(increment),
    accumulatedPose(),
    op.exec_time,
    op.num_recoveries};
}

/*******************************************************************************
*/
void Matcher::setInitialPose(const Pose& pose)
{
  accumulated_ =
    FSM::Utils::computeTransform(asTuple(pose), Eigen::Matrix3d::Identity());
}

}  // namespace fsm_lo
