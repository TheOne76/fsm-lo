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
#ifndef FSM_LO__FSM_LO_HPP_
#define FSM_LO__FSM_LO_HPP_

#include <cstddef>
#include <expected>
#include <span>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "fsm_lo/fsm_core.hpp"

namespace fsm_lo
{

/**
 * @brief A planar pose, or a planar displacement between two poses.
 *
 * The core carries the same type, so this is an alias rather than a second
 * declaration: the two layers hand poses to each other unconverted.
 */
using Pose = FSM::Pose;

/**
 * @brief Everything the matcher needs to know, and nothing about ROS.
 */
struct Parameters
{
  std::size_t size_scan{360};
  unsigned int num_iterations{2};
  double xy_bound{0.2};
  double t_bound{M_PI / 4};
  unsigned int max_counter{200};
  unsigned int min_magnification_size{0};
  unsigned int max_magnification_size{3};
  unsigned int max_recoveries{10};

  /** Zero draws from hardware entropy, as the algorithm has always done. Any
   *  other value pins the recovery search so a run can be reproduced. */
  unsigned int rng_seed{0};

  /** How each ray of a scan is matched to the wall it meets, either
   *  "angular" or "windowed". The first returns the nearest wall in front of
   *  every ray whatever shape the room is, and costs time in proportion to the
   *  ray count. The second is what this algorithm shipped with: it costs time
   *  in proportion to the square of the ray count, and where a room turns back
   *  on itself it can return a wall standing behind the nearest one. */
  std::string ray_search{"angular"};
};

/**
 * @brief What one successful match yields.
 */
struct MatchResult
{
  /** The displacement from the previous scan's pose to this one's. */
  Pose increment;

  /** The pose accumulated from every increment so far. */
  Pose accumulated;

  double execution_time{0.0};
  unsigned int num_recoveries{0};
};

/**
 * @brief Why a scan produced no match.
 */
enum class MatchError
{
  /** The first scan has nothing to be matched against. */
  no_reference_yet,

  /** The scan carried fewer ranges than the configured scan size. */
  scan_too_short,

  /** Every range in the scan was invalid, so there is nothing to match. */
  scan_entirely_invalid,
};

/**
 * @brief Whether a range reading carries a usable distance.
 *
 * A lidar reports a ray that struck nothing, or a reading it could not trust,
 * in one of three ways: zero, infinity, or not-a-number. Which of the three it
 * picks is a matter of driver convention. All three mean the same thing here,
 * and are filled in from the rays around them before matching.
 */
bool isValidRange(double range);

/**
 * @brief Validate a parameter set.
 * @return an empty string when the parameters are usable, otherwise a sentence
 *         naming the offending parameter and its value.
 */
std::string validate(const Parameters& parameters);

/**
 * @brief Lidar odometry by correspondenceless scan matching.
 *
 * Holds the odometry state that is not a ROS concept: the cached transform
 * plans, the previous scan, the accumulating pose, and the trajectory. Feed it
 * scans and it yields the displacement between each pair.
 */
class Matcher
{
public:
  explicit Matcher(const Parameters& parameters);

  Matcher(const Matcher&) = delete;
  Matcher& operator=(const Matcher&) = delete;
  Matcher(Matcher&&) = delete;
  Matcher& operator=(Matcher&&) = delete;

  /**
   * @brief Match a scan against the one before it.
   *
   * The first scan after construction or after a reset is kept as the
   * reference and yields no result.
   */
  std::expected<MatchResult, MatchError> process(std::span<const double> ranges);

  /**
   * @brief Forget the trajectory and return the accumulated pose to the origin.
   *
   * The reference scan is kept, so the next scan still produces an increment.
   */
  void clearTrajectory();

  /**
   * @brief Place the accumulated pose at an arbitrary starting point.
   */
  void setInitialPose(const Pose& pose);

  const Parameters& parameters() const { return parameters_; }
  Pose accumulatedPose() const;
  const std::vector<Pose>& trajectory() const { return trajectory_; }
  unsigned int scansSeen() const { return scans_seen_; }

private:
  Parameters parameters_;

  /*
   * Owned by the shared plan cache, which keeps them for the life of the
   * process. Creating one is expensive and, under FFTW_MEASURE, involves
   * timing trials, so they are not per matcher resources.
   */
  fftw_plan forward_plan_;
  fftw_plan inverse_plan_;

  std::vector<double> reference_scan_;
  std::vector<Pose> trajectory_;
  Eigen::Matrix3d accumulated_{Eigen::Matrix3d::Identity()};
  unsigned int scans_seen_{0};
};

}  // namespace fsm_lo

#endif  // FSM_LO__FSM_LO_HPP_
