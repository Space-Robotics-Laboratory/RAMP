// Copyright (c) 2026 Masazumi Imai
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

#ifndef RAMP__MD__MOMENTUM_DISTRIBUTION_HPP_
#define RAMP__MD__MOMENTUM_DISTRIBUTION_HPP_

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "ramp/visibility_control.h"

namespace ramp
{
namespace md
{

struct VelocityCommand
{
  Eigen::VectorXd base_velocity;     // Base spatial velocity (6x1)
  Eigen::VectorXd joint_velocities;  // Whole-body joint velocities (num_joints x 1)
};

struct AdaptiveDLSParams
{
  double max_lambda = 0.1;
  double epsilon = 0.05;
};

class RAMP_PUBLIC MomentumDistribution
{
public:
  explicit MomentumDistribution(int num_joints, int num_limbs);
  virtual ~MomentumDistribution() = default;

  /**
   * @brief Compute whole-body desired joint velocities considering momentum distribution
   *        based on whole-body Jacobian and swing limb end-effector velocity
   *
   * @param H_b Base inertia matrix (6x6)
   * @param H_bm Coupling inertia matrix between base and joints (6 x num_joints)
   * @param J_b_support Base jacobian of support limbs (6k1 x 6)
   * @param J_m_support Joint jacobian of support limbs (6k1 x num_joints)
   * @param J_b_swing Base jacobian of swing limbs (6k2 x 6)
   * @param J_m_swing Joint jacobian of swing limbs (6k2 x num_joints)
   * @param v_swing_ee_des Desired end-effector spatial velocity (6k2 x 1)
   * @param alpha Momentum distribution factor (0.0 ~ 1.0)
   * @param dls_params Stabilization parameters for the pseudo-inverse matrix
   * @return VelocityCommand Base velocity and joint angular velocities
   */
  VelocityCommand computeVelocities(
    const Eigen::MatrixXd & H_b, const Eigen::MatrixXd & H_bm, const Eigen::MatrixXd & J_b_support,
    const Eigen::MatrixXd & J_m_support, const Eigen::MatrixXd & J_b_swing,
    const Eigen::MatrixXd & J_m_swing, const Eigen::VectorXd & v_swing_ee_des, double alpha = 1.0,
    const AdaptiveDLSParams & dls_params = AdaptiveDLSParams());

private:
  Eigen::MatrixXd computePseudoInverseAdaptiveDLS(
    const Eigen::MatrixXd & J, double max_lambda, double epsilon);

  const int kNumJoints_;
  const int kNumLimbs_;

  Eigen::VectorXd S_inv_buffer_;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
};

}  // namespace md
}  // namespace ramp

#endif  // RAMP__MD__MOMENTUM_DISTRIBUTION_HPP_
