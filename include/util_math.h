/// @file: util_math.h
#pragma once

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
  for (int i = 0; i < SIZE; i++) {
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

}  // namespace utils
