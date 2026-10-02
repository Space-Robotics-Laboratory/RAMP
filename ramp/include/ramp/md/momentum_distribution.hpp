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

#ifndef RAMP__MD__MOMENTUM_DISTRIBUTION_HPP_
#define RAMP__MD__MOMENTUM_DISTRIBUTION_HPP_

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "ramp/visibility_control.h"

namespace ramp
{
namespace md
{

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
   * @param base_velocity Output base spatial velocity (6x1)
   * @param joint_velocities Output whole-body joint velocities (num_joints x 1)
   * @param dls_params Stabilization parameters for the pseudo-inverse matrix
   *
   * Allocation-free once called with the same input sizes, for real-time loops.
   */
  void computeVelocities(
    const Eigen::Ref<const Eigen::MatrixXd> & H_b, const Eigen::Ref<const Eigen::MatrixXd> & H_bm,
    const Eigen::Ref<const Eigen::MatrixXd> & J_b_support,
    const Eigen::Ref<const Eigen::MatrixXd> & J_m_support,
    const Eigen::Ref<const Eigen::MatrixXd> & J_b_swing,
    const Eigen::Ref<const Eigen::MatrixXd> & J_m_swing,
    const Eigen::Ref<const Eigen::VectorXd> & v_swing_ee_des, double alpha,
    Eigen::Ref<Eigen::VectorXd> base_velocity, Eigen::Ref<Eigen::VectorXd> joint_velocities,
    const AdaptiveDLSParams & dls_params = AdaptiveDLSParams());

private:
  // Writes J^+ into J_pinv (cols x rows of J).
  void computePseudoInverseAdaptiveDLS(
    const Eigen::Ref<const Eigen::MatrixXd> & J, double max_lambda, double epsilon,
    Eigen::Ref<Eigen::MatrixXd> J_pinv);

  const int kNumJoints_;
  const int kNumLimbs_;

  // --- Preallocated workspaces (sized for 6 * num_limbs rows) ---
  Eigen::VectorXd S_inv_buffer_;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
  Eigen::MatrixXd svd_input_;           // copy of J: JacobiSVD::compute takes a MatrixXd
  Eigen::MatrixXd scaled_u_;            // U * S_inv
  Eigen::MatrixXd support_pinv_;        // J_m_support^+
  Eigen::MatrixXd swing_pinv_;          // J_m_swing^+
  Eigen::VectorXd swing_nominal_;       // dq_sw,nom
  Eigen::VectorXd swing_residual_;      // v_sw,des - J_b_sw * dx_b
  Eigen::VectorXd support_motion_;      // J_b_sup * dx_b
  Eigen::MatrixXd pinv_base_jacobian_;  // J_m^+ * J_b, num_joints x 6
};

}  // namespace md
}  // namespace ramp

#endif  // RAMP__MD__MOMENTUM_DISTRIBUTION_HPP_
