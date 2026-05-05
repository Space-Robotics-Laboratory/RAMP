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

#define DEBUG false

namespace ramp
{
namespace md
{

MomentumDistribution::MomentumDistribution(int num_joints, int num_limbs)
: kNumJoints_(num_joints), kNumLimbs_(num_limbs)
{
  int max_rows = 6 * kNumLimbs_;

  int max_singular_values = std::min(kNumJoints_, max_rows);
  S_inv_buffer_.resize(max_singular_values);

  // HACK: Force the SVD solver to reserve internal buffers using dummy matrices
  Eigen::MatrixXd dummy_J = Eigen::MatrixXd::Zero(max_rows, kNumJoints_);
  svd_.compute(dummy_J, Eigen::ComputeThinU | Eigen::ComputeThinV);
}

VelocityCommand MomentumDistribution::computeVelocities(
  const Eigen::MatrixXd & H_b, const Eigen::MatrixXd & H_bm, const Eigen::MatrixXd & J_b_support,
  const Eigen::MatrixXd & J_m_support, const Eigen::MatrixXd & J_b_swing,
  const Eigen::MatrixXd & J_m_swing, const Eigen::VectorXd & v_swing_ee_des, double alpha,
  const AdaptiveDLSParams & dls_params)
{
  VelocityCommand cmd;

  double max_lambda = dls_params.max_lambda;
  double epsilon = dls_params.epsilon;

  Eigen::MatrixXd J_m_sup_pinv = computePseudoInverseAdaptiveDLS(J_m_support, max_lambda, epsilon);
  Eigen::MatrixXd J_m_sw_pinv = computePseudoInverseAdaptiveDLS(J_m_swing, max_lambda, epsilon);

  // Nominal joint velocities and momenta of the swing limb
  // HACK: Assuming the base is fixed
  Eigen::VectorXd dq_sw_nom = J_m_sw_pinv * v_swing_ee_des;
  Eigen::Matrix<double, 6, 1> L_sw_nom = H_bm * dq_sw_nom;

  // A = H_{b} - H_{bm} * J_{m_sup}^+ * J_{b_sup}
  Eigen::Matrix<double, 6, 6> A = H_b - H_bm * J_m_sup_pinv * J_b_support;

  // B = A - alpha H_{bm} * J_{m_sw}^+ * J_{b_sw}
  Eigen::Matrix<double, 6, 6> B = A - alpha * H_bm * J_m_sw_pinv * J_b_swing;

  // d{x}_b = -alpha * B^(-1) * L_{sw, nom}
  cmd.base_velocity = -alpha * B.colPivHouseholderQr().solve(L_sw_nom);

  // d{q}_{sup} = -J_{m_sup}^+ * J_{b_sup} * d{x}_b
  Eigen::VectorXd dq_sup = -J_m_sup_pinv * J_b_support * cmd.base_velocity;

  // Actual joint angular velocity of the swing limbs
  Eigen::VectorXd dq_sw = J_m_sw_pinv * (v_swing_ee_des - J_b_swing * cmd.base_velocity);

  // Whole-body joint angular velocity vector
  cmd.joint_velocities = dq_sup + dq_sw;

#if DEBUG
  Eigen::VectorXd L = H_b * cmd.base_velocity + H_bm * cmd.joint_velocities;
  std::cout << "L = " << L.transpose() << std::endl;
#endif  // DEBUG

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
