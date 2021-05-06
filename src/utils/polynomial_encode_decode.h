/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Dexai Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
#include <Eigen/Core>
#include <drake/common/polynomial.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

#include <robot_msgs/lcmt_piecewise_polynomial.hpp>
#include <robot_msgs/lcmt_polynomial.hpp>
#include <robot_msgs/lcmt_polynomial_matrix.hpp>

void encodePolynomial(const drake::Polynomiald& polynomial,
                      // NOLINTNEXTLINE(runtime/references)
                      robot_msgs::lcmt_polynomial& msg);
drake::Polynomiald decodePolynomial(const robot_msgs::lcmt_polynomial& msg);
template <int RowsAtCompileTime, int ColsAtCompileTime>
void encodePolynomialMatrix(
    const Eigen::Matrix<drake::Polynomiald, RowsAtCompileTime,
                        ColsAtCompileTime>& polynomial_matrix,
    // NOLINTNEXTLINE(runtime/references)
    robot_msgs::lcmt_polynomial_matrix& msg) {
  msg.polynomials.clear();
  msg.polynomials.resize(polynomial_matrix.rows());
  for (int row = 0; row < polynomial_matrix.rows(); ++row) {
    auto& polynomial_msg_row = msg.polynomials[row];
    polynomial_msg_row.resize(polynomial_matrix.cols());
    for (int col = 0; col < polynomial_matrix.cols(); ++col) {
      encodePolynomial(polynomial_matrix(row, col), polynomial_msg_row[col]);
    }
  }
  msg.rows = polynomial_matrix.rows();
  msg.cols = polynomial_matrix.cols();
}
template <int RowsAtCompileTime, int ColsAtCompileTime>
Eigen::Matrix<drake::Polynomiald, RowsAtCompileTime, ColsAtCompileTime>
decodePolynomialMatrix(const robot_msgs::lcmt_polynomial_matrix& msg) {
  Eigen::Matrix<drake::Polynomiald, RowsAtCompileTime, ColsAtCompileTime> ret(
      msg.rows, msg.cols);
  for (int row = 0; row < msg.rows; ++row) {
    for (int col = 0; col < msg.cols; ++col) {
      ret(row, col) = decodePolynomial(msg.polynomials[row][col]);
    }
  }
  return ret;
}
/**
 * See issue https://github.com/DexaiRobotics/drake-franka-driver/issues/54
 * TODO: remove and use new drake method
 */
void encodePiecewisePolynomial(const drake::trajectories::PiecewisePolynomial<
                                   double>& piecewise_polynomial,
                               robot_msgs::lcmt_piecewise_polynomial& msg);
drake::trajectories::PiecewisePolynomial<double> decodePiecewisePolynomial(
    const robot_msgs::lcmt_piecewise_polynomial& msg);
