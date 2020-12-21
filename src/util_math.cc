///@file: util_math.cc
#include "util_math.h"

#include "dexai_log.h"

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

bool is_continuous(std::unique_ptr<PPType>& old_plan,
                   std::unique_ptr<PPType>& new_plan, 
                   double franka_time, 
                   Eigen::VectorXd pos_tolerance, 
                   Eigen::VectorXd vel_tolerance, 
                   Eigen::VectorXd acc_tolerance)
{
    std::function is_tolerated = [&](int d, Eigen::VectorXd tolerance)-> bool {
        Eigen::VectorXd err = (old_plan->derivative(d).value(franka_time) - new_plan->derivative(d).value(franka_time)).cwiseAbs();
        return (tolerance - err).sum() >= 0.0;
    };

    return (is_tolerated(0, pos_tolerance) && is_tolerated(1, vel_tolerance) && is_tolerated(2, acc_tolerance));


}
}   //  namespace utils
