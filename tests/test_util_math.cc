// @file: test_util_math.cc -- part of a googletest suite
#include <gtest/gtest.h>

#include "util_math.h"

TEST(UtilMath, v_to_e_and_e_to_v) {
  Eigen::VectorXd e = Eigen::VectorXd::Zero(7);
  e << 10, 8, 10, 8, 8, 4, 9;
  std::vector<double> v;
  ASSERT_NO_THROW(v = utils::e_to_v(e));
  Eigen::VectorXd e2;
  ASSERT_NO_THROW(e2 = utils::v_to_e(v));
  EXPECT_TRUE(utils::VectorEpsEq(e, e2));
}
