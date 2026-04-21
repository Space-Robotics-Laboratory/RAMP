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

#include "ramp/md/momentum_distribution.hpp"

#include <iostream>

namespace ramp
{
namespace md
{

MomentumDistribution::MomentumDistribution(int num_joints, int num_limbs)
: kNumJoints_(num_joints), kNumLimbs_(num_limbs)
{
  int max_rows = 6 * kNumLimbs_;

  A_matrix_.resize(6, 6);

  int max_singular_values = std::min(kNumJoints_, max_rows);
  S_inv_buffer_.resize(max_singular_values);

  // HACK: Force the SVD solver to reserve internal buffers using dummy matrices
  Eigen::MatrixXd dummy_J = Eigen::MatrixXd::Zero(max_rows, kNumJoints_);
  svd_.compute(dummy_J, Eigen::ComputeThinU | Eigen::ComputeThinV);
}

VelocityCommand MomentumDistribution::computeVelocities(
  const Eigen::MatrixXd & H_b, const Eigen::MatrixXd & H_bm_sup, const Eigen::MatrixXd & J_b_sup,
  const Eigen::MatrixXd & J_m_sup, const Eigen::VectorXd & L_swing, double alpha,
  const AdaptiveDLSParams & dls_params)
{
  VelocityCommand cmd;

  // Eigen::MatrixXd J_m_pinv = J_m_sup.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::MatrixXd J_m_pinv =
    computePseudoInverseAdaptiveDLS(J_m_sup, dls_params.max_lambda, dls_params.epsilon);

  // A = H_b - H_{bm_sup} * J_{m_sup}^+ * J_{b_sup}
  A_matrix_ = H_b - H_bm_sup * J_m_pinv * J_b_sup;
  // b = -alpha * L_{swing}
  Eigen::VectorXd b = L_swing;
  // (A * d{x}_b = b)
  // d{x}_b = A^(-1) * b
  cmd.base_velocity = -alpha * A_matrix_.colPivHouseholderQr().solve(b);

  /// d{q}_{sup} = -J_{m_sup}^+ * J_{b_sup} * d{x}_b
  cmd.support_limb_joint_velocities = -J_m_pinv * J_b_sup * cmd.base_velocity;

  return cmd;
}

Eigen::MatrixXd MomentumDistribution::computePseudoInverseAdaptiveDLS(
  const Eigen::MatrixXd & J, double max_lambda, double epsilon)
{
  // Compute the SVD of the current Jacobian
  svd_.compute(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

  const Eigen::VectorXd & singular_values = svd_.singularValues();
  int k = singular_values.size();

  for (int i = 0; i < k; ++i) {
    double sigma = singular_values(i);
    double lambda = 0.0;

    if (sigma < epsilon) {
      double ratio = sigma / epsilon;
      lambda = max_lambda * (1.0 - ratio * ratio);
    }

    // sigma / (sigma^2 + lambda^2)
    S_inv_buffer_(i) = sigma / (sigma * sigma + lambda * lambda);
  }

  // J^+ = V * S_{inv} * U^T
  return svd_.matrixV() * S_inv_buffer_.head(k).asDiagonal() * svd_.matrixU().transpose();
}

}  // namespace md
}  // namespace ramp
