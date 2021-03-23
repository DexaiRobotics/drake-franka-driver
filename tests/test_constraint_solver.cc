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

// @file: test_constraint_solver.cc -- part of a googletest suite
#include <gtest/gtest.h>

#include "driver/constraint_solver.h"

static std::string empty_string;
using namespace franka_driver;
using franka_driver::RobotParameters;

namespace util_test {

///// RobotParameters Loading: test use only
/////////////////////////////////////////
const std::string g_test_params_yaml_file_name = "franka_test.yaml";

const std::string get_test_params_yaml_file_name() {
  return g_test_params_yaml_file_name;
}
/// Load can throw std::runtime_error
inline bool load_params(std::string& yaml_full_path, RobotParameters& params,
                        int verbose = 1) {
  if (yaml_full_path.empty()) {
    yaml_full_path = get_test_params_yaml_file_name();
  }

  params = franka_driver::loadYamlParameters(yaml_full_path, verbose);
  return true;
}

/// The current way of making a mock constraint_solver is to make a real one,
/// albeit using local (CLI-specified) or test (default) config and URDF files.
ConstraintSolver* make_constraint_solver(
    RobotParameters& params,      // output
    std::string& yaml_full_path,  // input & output
    const int verbose = 0         // input
) {
  ConstraintSolver* constraint_solver = nullptr;
  if (load_params(yaml_full_path, params, verbose)) {
    params.urdf_dir = utils::get_cwd();

    params.UpdateUrdf(params.urdf);
    params.urdf_filepath = params.urdf;
    constraint_solver = new ConstraintSolver(&params);
  }
  return constraint_solver;
}

}  // end of namespace util_test

TEST(ConstraintSolver, TestConstructor) {
  int verbose = 1;
  RobotParameters p;
  ConstraintSolver* constraint_solver = nullptr;
  try {
    constraint_solver =
        util_test::make_constraint_solver(p, empty_string, verbose);
    SUCCEED();
  } catch (std::exception const& err) {
    FAIL() << "CAUGHT AN EXCEPTION: " << err.what() << std::endl;
  }
  EXPECT_FALSE(constraint_solver->GetUrdfPath().empty());
}
