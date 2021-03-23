#include "utils/polynomial_encode_decode.h"

#include <vector>

using drake::trajectories::PiecewisePolynomial;
using Eigen::Dynamic;
using Eigen::Map;
using Eigen::VectorXd;

void encodePolynomial(const drake::Polynomiald& polynomial,
                      // NOLINTNEXTLINE(runtime/references)
                      drake::lcmt_polynomial& msg) {
  // convert eigen vector to std vector
  std::vector<double> polyvec(polynomial.GetNumberOfCoefficients());
  Eigen::VectorXd::Map(&polyvec[0], polynomial.GetNumberOfCoefficients()) =
      polynomial.GetCoefficients();
  msg.coefficients = polyvec;
  msg.num_coefficients = polynomial.GetNumberOfCoefficients();
}
drake::Polynomiald decodePolynomial(const drake::lcmt_polynomial& msg) {
  Map<const VectorXd> coefficients(msg.coefficients.data(),
                                   msg.coefficients.size());
  return drake::Polynomiald(coefficients);
}
void encodePiecewisePolynomial(const drake::trajectories::PiecewisePolynomial<
                                   double>& piecewise_polynomial,
                               // NOLINTNEXTLINE(runtime/references)
                               drake::lcmt_piecewise_polynomial& msg) {
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
    const drake::lcmt_piecewise_polynomial& msg) {
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
