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

#include "utils/polynomial_encode_decode.h"

#include <vector>

using drake::trajectories::PiecewisePolynomial;
using Eigen::Dynamic;
using Eigen::Map;
using Eigen::VectorXd;

void encodePolynomial(const drake::Polynomiald& polynomial,
                      // NOLINTNEXTLINE(runtime/references)
                      robot_msgs::lcmt_polynomial& msg) {
  // convert eigen vector to std vector
  std::vector<double> polyvec(polynomial.GetNumberOfCoefficients());
  Eigen::VectorXd::Map(&polyvec[0], polynomial.GetNumberOfCoefficients()) =
      polynomial.GetCoefficients();
  msg.coefficients = polyvec;
  msg.num_coefficients = polynomial.GetNumberOfCoefficients();
}
drake::Polynomiald decodePolynomial(const robot_msgs::lcmt_polynomial& msg) {
  Map<const VectorXd> coefficients(msg.coefficients.data(),
                                   msg.coefficients.size());
  return drake::Polynomiald(coefficients);
}
void encodePiecewisePolynomial(const drake::trajectories::PiecewisePolynomial<
                                   double>& piecewise_polynomial,
                               // NOLINTNEXTLINE(runtime/references)
                               robot_msgs::lcmt_piecewise_polynomial& msg) {
  msg.num_segments = piecewise_polynomial.get_number_of_segments();
  msg.num_breaks = piecewise_polynomial.get_number_of_segments() + 1;
  msg.breaks = piecewise_polynomial.get_segment_times();
  msg.polynomial_matrices.resize(piecewise_polynomial.get_number_of_segments());
  for (int i = 0; i < piecewise_polynomial.get_number_of_segments(); ++i) {
    encodePolynomialMatrix<Eigen::Dynamic, Eigen::Dynamic>(
        piecewise_polynomial.getPolynomialMatrix(i),
        msg.polynomial_matrices[i]);
  }
}
drake::trajectories::PiecewisePolynomial<double> decodePiecewisePolynomial(
    const robot_msgs::lcmt_piecewise_polynomial& msg) {
  using PolyMat =
      drake::trajectories::PiecewisePolynomial<double>::PolynomialMatrix;
  std::vector<PolyMat> polynomial_matrices;
  for (size_t i {}; i < msg.polynomial_matrices.size(); ++i) {
    polynomial_matrices.push_back(
        decodePolynomialMatrix<Dynamic, Dynamic>(msg.polynomial_matrices[i]));
  }
  return drake::trajectories::PiecewisePolynomial<double>(polynomial_matrices,
                                                          msg.breaks);
}
