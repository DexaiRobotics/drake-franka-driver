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

/// @file: util_math.h
#pragma once

#include <drake/math/rigid_transform.h>
#include "drake/common/trajectories/piecewise_polynomial.h"

typedef drake::trajectories::PiecewisePolynomial<double> PPType;

namespace std {
string to_string(const drake::Isometry3<double>& pose);
ostream& operator<<(ostream& os, drake::Isometry3<double> const& pose);
}  // namespace std

namespace utils {
// Vector Conversions:
template <typename T, std::size_t SIZE>
std::vector<T> ArrayToVector(const std::array<T, SIZE>& a) {
  std::vector<T> v(a.begin(), a.end());
  return v;
}

template <typename T, std::size_t SIZE>
void VectorToArray(const std::vector<T>& v, std::array<T, SIZE>& a) {
  for (size_t i {}; i < SIZE; i++) {
    a[i] = v[i];
  }
}

std::array<double, 7> EigenToArray(const Eigen::VectorXd& input);

std::vector<double> e_to_v(Eigen::VectorXd e);

template <typename T>
Eigen::VectorXd v_to_e(std::vector<T> v);

template <typename T>
double max_angular_distance(T t1, T t2) {
  T delta = t1 - t2;
  double max = delta.cwiseAbs().maxCoeff();
  return max;
}

/// TODO: Move EpsEq and VectorEpsEq to a separate header.
/// Determines if two values are 'close enough' based on a scaled tolerance
template <typename T>
bool EpsEq(
    T a, T b, T rel_epsilon = std::numeric_limits<T>::epsilon(),
    typename std::enable_if<std::is_floating_point<T>::value, T>::type* = 0) {
  T value_range[] = {T(1.0), a, b};
  return (std::abs(a - b)
          <= rel_epsilon * *std::max_element(value_range, value_range + 3));
}

/// TODO: change to use iterators so can handle more than just Eigen types.
/// NOTE: Container of type ContainerT should contain values of type ValueT.
/// template<typename ContainerT, typename ValueT>
/// bool VectorEpsEq(ContainerT a, ContainerT b, ValueT relTol =
///         std::numeric_limits<ValueT>::epsilon()) {
template <typename T>
bool VectorEpsEq(T a, T b,
                 double relTol = std::numeric_limits<double>::epsilon()) {
  if (a.size() != b.size()) {
    return false;
  }
  for (int i = 0; i < a.size(); i++) {
    if (!EpsEq(double(a(i)), double(b(i)), relTol)) {
      return false;
    }
  }
  return true;
}

inline drake::math::RigidTransformd LinearInterpPose(
    const drake::math::RigidTransformd& X_i,
    const drake::math::RigidTransformd& X_f, const double frac) {
  const Eigen::Vector3d &x_i {X_i.translation()}, &x_f {X_f.translation()};
  const Eigen::Quaterniond q_i {X_i.rotation().ToQuaternion()},
      q_f {X_f.rotation().ToQuaternion()};
  return drake::math::RigidTransformd(q_i.slerp(frac, q_f),
                                      (1 - frac) * x_i + frac * x_f);
}

}  // namespace utils
