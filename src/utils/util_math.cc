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

/// @file: util_math.cc
#include "utils/util_math.h"

#include <memory>  // for unique_ptr
#include <string>  // for string
#include <vector>  // for vector

#include "utils/dexai_log.h"

using drake::trajectories::PiecewisePolynomial;

namespace std {
string to_string(const drake::Isometry3<double>& pose) {
  Eigen::Vector3d vec3(pose.translation());
  Eigen::Quaterniond quat(pose.linear());
  return fmt::format(
      "iso3[v_xyz({:5.4} {:5.4} {:5.4}) q_wxyz({:.4} {:.4} {:.4} {:.4})]",
      vec3(0), vec3(1), vec3(2), quat.w(), quat.x(), quat.y(), quat.z());
}

ostream& operator<<(ostream& os, drake::Isometry3<double> const& pose) {
  return os << to_string(pose);
}
}  // namespace std

namespace utils {

std::array<double, 7> EigenToArray(const Eigen::VectorXd& input) {
  std::array<double, 7> output = {
      {input[0], input[1], input[2], input[3], input[4], input[5], input[6]}};
  return output;
}

std::vector<double> e_to_v(Eigen::VectorXd e) {
  std::vector<double> v;
  v.resize(e.size());
  Eigen::VectorXd::Map(&v[0], e.size()) = e;
  return v;
}

template <>
Eigen::VectorXd v_to_e(std::vector<double> v) {
  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v.data(), v.size());
}

template <>
Eigen::VectorXd v_to_e(std::vector<float> v) {
  Eigen::VectorXf evf =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(v.data(), v.size());
  return evf.template cast<double>();
}

template <>
Eigen::VectorXd v_to_e(std::vector<int> v) {
  Eigen::VectorXi evi =
      Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(v.data(), v.size());
  return evi.template cast<double>();
}

bool is_continuous(const std::unique_ptr<PPType>& old_plan,
                   const std::unique_ptr<PPType>& new_plan, double franka_time,
                   const Eigen::VectorXd& pos_tolerance,
                   const Eigen::VectorXd& vel_tolerance,
                   const Eigen::VectorXd& acc_tolerance) {
  std::function is_tolerated {[&](int d, Eigen::VectorXd tolerance) -> bool {
    const auto old_plan_derivative {old_plan->derivative(d).value(franka_time)};
    const auto new_plan_derivative {new_plan->derivative(d).value(franka_time)};
    const auto err {(new_plan_derivative - old_plan_derivative).cwiseAbs()};
    return (err.array() < tolerance.array()).all();
  }};

  return (is_tolerated(0, pos_tolerance) && is_tolerated(1, vel_tolerance)
          && is_tolerated(2, acc_tolerance));
}

}  //  namespace utils
