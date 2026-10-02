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
  svd_input_ = Eigen::MatrixXd::Zero(6, kNumJoints_);
  scaled_u_ = Eigen::MatrixXd::Zero(max_rows, max_singular_values);
  support_pinv_ = Eigen::MatrixXd::Zero(kNumJoints_, max_rows);
  swing_pinv_ = Eigen::MatrixXd::Zero(kNumJoints_, max_rows);
  swing_nominal_ = Eigen::VectorXd::Zero(kNumJoints_);
  swing_residual_ = Eigen::VectorXd::Zero(max_rows);
  support_motion_ = Eigen::VectorXd::Zero(max_rows);
  pinv_base_jacobian_ = Eigen::MatrixXd::Zero(kNumJoints_, 6);

  // HACK: Force the SVD solver to reserve internal buffers using dummy matrices
  svd_.compute(svd_input_, Eigen::ComputeThinU | Eigen::ComputeThinV);
}

VelocityCommand MomentumDistribution::computeVelocities(
  const Eigen::MatrixXd & H_b, const Eigen::MatrixXd & H_bm, const Eigen::MatrixXd & J_b_support,
  const Eigen::MatrixXd & J_m_support, const Eigen::MatrixXd & J_b_swing,
  const Eigen::MatrixXd & J_m_swing, const Eigen::VectorXd & v_swing_ee_des, double alpha,
  const AdaptiveDLSParams & dls_params)
{
  VelocityCommand cmd;
  cmd.base_velocity = Eigen::VectorXd::Zero(6);
  cmd.joint_velocities = Eigen::VectorXd::Zero(kNumJoints_);
  computeVelocities(
    H_b, H_bm, J_b_support, J_m_support, J_b_swing, J_m_swing, v_swing_ee_des, alpha,
    cmd.base_velocity, cmd.joint_velocities, dls_params);
  return cmd;
}

void MomentumDistribution::computeVelocities(
  const Eigen::Ref<const Eigen::MatrixXd> & H_b, const Eigen::Ref<const Eigen::MatrixXd> & H_bm,
  const Eigen::Ref<const Eigen::MatrixXd> & J_b_support,
  const Eigen::Ref<const Eigen::MatrixXd> & J_m_support,
  const Eigen::Ref<const Eigen::MatrixXd> & J_b_swing,
  const Eigen::Ref<const Eigen::MatrixXd> & J_m_swing,
  const Eigen::Ref<const Eigen::VectorXd> & v_swing_ee_des, double alpha,
  Eigen::Ref<Eigen::VectorXd> base_velocity, Eigen::Ref<Eigen::VectorXd> joint_velocities,
  const AdaptiveDLSParams & dls_params)
{
  const double max_lambda = dls_params.max_lambda;
  const double epsilon = dls_params.epsilon;
  const Eigen::Index support_rows = J_m_support.rows();
  const Eigen::Index swing_rows = J_m_swing.rows();

  auto J_m_sup_pinv = support_pinv_.leftCols(support_rows);
  auto J_m_sw_pinv = swing_pinv_.leftCols(swing_rows);
  computePseudoInverseAdaptiveDLS(J_m_support, max_lambda, epsilon, J_m_sup_pinv);
  computePseudoInverseAdaptiveDLS(J_m_swing, max_lambda, epsilon, J_m_sw_pinv);

  // Nominal joint velocities and momenta of the swing limb
  // HACK: Assuming the base is fixed
  swing_nominal_.noalias() = J_m_sw_pinv * v_swing_ee_des;
  const Eigen::Matrix<double, 6, 1> L_sw_nom = H_bm * swing_nominal_;

  // A = H_{b} - H_{bm} * J_{m_sup}^+ * J_{b_sup}
  // B = A - alpha H_{bm} * J_{m_sw}^+ * J_{b_sw}
  Eigen::Matrix<double, 6, 6> B = H_b;
  pinv_base_jacobian_.noalias() = J_m_sup_pinv * J_b_support;
  B.noalias() -= H_bm * pinv_base_jacobian_;
  pinv_base_jacobian_.noalias() = J_m_sw_pinv * J_b_swing;
  B.noalias() -= alpha * H_bm * pinv_base_jacobian_;

  // d{x}_b = -alpha * B^(-1) * L_{sw, nom}
  const Eigen::Matrix<double, 6, 1> base = -alpha * B.colPivHouseholderQr().solve(L_sw_nom);
  base_velocity = base;

  // d{q}_{sup} = -J_{m_sup}^+ * J_{b_sup} * d{x}_b
  // d{q}_{sw} = J_{m_sw}^+ * (v_{sw,des} - J_{b_sw} * d{x}_b)
  auto residual = swing_residual_.head(swing_rows);
  residual = v_swing_ee_des;
  residual.noalias() -= J_b_swing * base;
  joint_velocities.noalias() = J_m_sw_pinv * residual;
  auto support_motion = support_motion_.head(support_rows);
  support_motion.noalias() = J_b_support * base;
  joint_velocities.noalias() -= J_m_sup_pinv * support_motion;

#if DEBUG
  Eigen::VectorXd L = H_b * base + H_bm * joint_velocities;
  std::cout << "L = " << L.transpose() << std::endl;
#endif  // DEBUG
}

void MomentumDistribution::computePseudoInverseAdaptiveDLS(
  const Eigen::Ref<const Eigen::MatrixXd> & J, double max_lambda, double epsilon,
  Eigen::Ref<Eigen::MatrixXd> J_pinv)
{
  // Compute the SVD of the current Jacobian
  svd_input_ = J;
  svd_.compute(svd_input_, Eigen::ComputeThinU | Eigen::ComputeThinV);

  const Eigen::VectorXd & singular_values = svd_.singularValues();
  int k = singular_values.size();

  for (int i = 0; i < k; ++i) {
    double sigma = singular_values(i);
    double lambda = 0.0;

    if (sigma < epsilon) {
      double ratio = sigma / epsilon;
      lambda = max_lambda * (1.0 - ratio * ratio);
    }

    // sigma / (sigma^2 + lambda^2); zero for a null direction with no damping
    const double denominator = sigma * sigma + lambda * lambda;
    S_inv_buffer_(i) = denominator > 0.0 ? sigma / denominator : 0.0;
  }

  // J^+ = V * S_{inv} * U^T
  auto U_scaled = scaled_u_.topLeftCorner(J.rows(), k);
  U_scaled.noalias() = svd_.matrixU() * S_inv_buffer_.head(k).asDiagonal();
  J_pinv.noalias() = svd_.matrixV() * U_scaled.transpose();
}

}  // namespace md
}  // namespace ramp
