// Copyright (c) 2026 Space Robotics Lab -- Tohoku University
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

#ifndef RAMP__LRST__LOW_REACTION_SWING_TRAJECTORY_HPP_
#define RAMP__LRST__LOW_REACTION_SWING_TRAJECTORY_HPP_

#include <Eigen/Dense>
#include <functional>
#include <vector>

#include <nlopt.hpp>

#include "ramp/visibility_control.h"

namespace ramp
{
namespace lrst
{

struct SolverParams
{
  double dt = 0.01;  // [s]

  double step_duration = 10.0;  // [s]
  double step_height = 0.05;    // [m]

  double relative_tol = 1e-4;
  int max_iter = 1000;
};

struct WeightParams
{
  double force_max = 1.0;
  double moment_max = 1.0;

  double step_height_max = 1.0;
  double step_height_ave = 1.0;
};

using IKSolverCallback =
  std::function<bool(Eigen::VectorXd & q, const Eigen::Isometry3d & pose_des)>;

using CouplingInertiaCallback =
  std::function<void(const Eigen::VectorXd & q, Eigen::MatrixXd & H_bm)>;

class RAMP_PUBLIC LowReactionSwingTrajectory
{
public:
  explicit LowReactionSwingTrajectory(int num_joints);
  virtual ~LowReactionSwingTrajectory() = default;

  void setBoundaryConditions(const Eigen::Vector3d & start_pos, const Eigen::Vector3d & end_pos);

  void setIKSolverCallback(IKSolverCallback ik_cb);

  void setCouplingInertiaCallback(CouplingInertiaCallback coupling_inertia_cb);

  void setRobotState(
    const Eigen::VectorXd & q_init,
    const Eigen::Matrix3d & initial_swing_ee_orientation = Eigen::Matrix3d::Identity());

  Eigen::MatrixXd optimizeTrajectory(
    const SolverParams & solver_params, const WeightParams & weight_params);

  Eigen::Vector3d computeBezierPosition(double t, const Eigen::MatrixXd & P) const;

  Eigen::Vector3d computeBezierVelocity(double t, const Eigen::MatrixXd & P) const;

private:
  double computeCost(const std::vector<double> & x);

  static double objectiveWrapper(
    const std::vector<double> & x, std::vector<double> & grad, void * data);

  static constexpr int kBezierOrder_ = 7;
  Eigen::Matrix<double, 3, kBezierOrder_ + 1> bezier_base_matrix_;

  SolverParams solver_params_;
  WeightParams weight_params_;

  const int kNumJoints_;

  IKSolverCallback ik_callback_;
  CouplingInertiaCallback coupling_inertia_callback_;

  Eigen::VectorXd q_init_;
  Eigen::Matrix3d init_sw_ee_ori_;

  bool is_boundary_set_ = false;
  bool is_robot_state_set_ = false;
};

}  // namespace lrst
}  // namespace ramp

#endif  // RAMP__LRST__LOW_REACTION_SWING_TRAJECTORY_HPP_
