// @file: test_constraint_solver.cc -- part of a googletest suite
#include <gtest/gtest.h>

#include "constraint_solver.h"

static std::string empty_string;
using namespace franka_driver;
using franka_driver::RobotParameters;

namespace util_test {

    ///// RobotParameters Loading: test use only //////////////////////////////////////
    const std::string g_test_params_yaml_file_name    = "franka_test.yaml";

    const std::string get_test_params_yaml_file_name() {
        return g_test_params_yaml_file_name;
    }
    /// Load can throw std::runtime_error
    inline bool load_params(std::string& yaml_full_path, RobotParameters &params, int verbose = 1)
    {
        if (yaml_full_path.empty()) {
            yaml_full_path = get_test_params_yaml_file_name();
        }

        params = franka_driver::loadYamlParameters(yaml_full_path, verbose);
        return true;
    }

    /// The current way of making a mock constraint_solver is to make a real one,
    /// albeit using local (CLI-specified) or test (default) config and URDF files.
    inline ConstraintSolver *make_constraint_solver( RobotParameters& params        // output
                                , std::string& yaml_full_path           // input & output
                                , const int verbose = 0                 // input
                                , const int threads = 6                 // max threads for OMPL
    ) {
        ConstraintSolver *constraint_solver = nullptr;
        if (load_params(yaml_full_path, params, verbose)) {
            params.urdf_dir = utils::get_cwd();

            params.UpdateUrdf(params.urdf);
            params.urdf_filepath = params.urdf;
            constraint_solver = new ConstraintSolver(&params);
        }
        return constraint_solver;
    }

} // end of namespace util_test

TEST (ConstraintSolver, TestConstructor)
{
    int verbose = 1;
    RobotParameters p;
    ConstraintSolver *constraint_solver = nullptr;
    try {
        constraint_solver = util_test::make_constraint_solver(p, empty_string, verbose);
        SUCCEED();
    }
    catch (std::exception const & err) {
        FAIL() << "CAUGHT AN EXCEPTION: " << err.what() << std::endl;
    }
    EXPECT_EQ(constraint_solver->GetNumActuatableJoints(), constraint_solver->GetRigidBodyTreeRef().get_num_positions());
    EXPECT_FALSE(constraint_solver->GetUrdfPath().empty());
}
