/// @file: test_yaml.cc -- part of a googletest suite
#include <gtest/gtest.h>
// #include "util_test.h"
#include "driver/constraint_solver.h"
#include "utils/robot_parameters.h"

static std::string empty_string("");

using dexai::log;
using franka_driver::RobotParameters;
using namespace franka_driver;

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
}  // end of namespace util_test

/// Tests test function: load_params
TEST(LoadParams, load_params) {
  std::string empty_yaml_path;
  RobotParameters params;
  int verbose = 2;
  bool loaded = false;
  try {
    loaded = util_test::load_params(empty_yaml_path, params, verbose);
    SUCCEED();
  } catch (std::exception const& err) {
    FAIL() << "CAUGHT AN EXCEPTION: " << err.what() << std::endl;
  }
  // EXPECT_NO_THROW( loaded = util_test::load_params(empty_yaml_path, params,
  // verbose));
  EXPECT_TRUE(loaded);

  ASSERT_FALSE(params.yaml_source_full_path.empty());
  ASSERT_FALSE(params.urdf_filepath.empty());
  // ASSERT_FALSE( params.urdf_robot_only_filepath.empty() );
  EXPECT_FALSE(params.world_frame.empty());
  const RobotParameters* const_copy = new RobotParameters(params);
  // EXPECT_EQ( const_copy->gravity_vector, params.gravity_vector );
  EXPECT_EQ(const_copy->world_frame, params.world_frame);
  EXPECT_EQ(const_copy->urdf_filepath, params.urdf_filepath);
  EXPECT_EQ(const_copy->yaml_source_full_path, params.yaml_source_full_path);
}

TEST(SaveParams, ParametersLog) {
  dexai::create_log("test_yaml", "log_dexai");
  std::string empty_yaml_path;
  RobotParameters params;
  int verbose = 1;
  bool loaded = false;
  EXPECT_NO_THROW(loaded =
                      util_test::load_params(empty_yaml_path, params, verbose));
  EXPECT_TRUE(loaded);
  const std::string& yaml_test_file_full_path =
      util_test::get_test_params_yaml_file_name();
  EXPECT_EQ(params.yaml_source_full_path, yaml_test_file_full_path);
}

/// Tests that loadYamlParameters throws an exception on failure
TEST(LoadParams, MissingAllParamsException) {
  std::string bogus_path("/dev/null");
  RobotParameters params;
  int verbose = 2;
  bool loaded = true;
  EXPECT_ANY_THROW(loaded =
                       util_test::load_params(bogus_path, params, verbose));
  EXPECT_TRUE(loaded);  // still true because not re-set

  // once again, direct call:
  EXPECT_THROW(params = franka_driver::loadYamlParameters(bogus_path, verbose),
               std::runtime_error);
  EXPECT_TRUE(params.urdf == "");
}

TEST(LoadParams, MissingFoundConverted) {
  // set-up
  const std::string& yaml_path = util_test::get_test_params_yaml_file_name();
  YAML::Node config, params;
  EXPECT_NO_THROW(config = YAML::LoadFile(yaml_path));
  EXPECT_NO_THROW(params = config["robot_parameters"]);
  EXPECT_TRUE(params);
  auto params_size = params.size();
  EXPECT_GT(params_size, 0);
  double dbl_val = std::numeric_limits<double>::max();
  std::string str_val;
  std::string non_key = "Evan_Rachel_Wood_plays_Dolores";

  // raw yaml-cpp
  auto any_val = config[non_key];
  EXPECT_FALSE(any_val);
  log()->info("YAML value for {} as assigned to auto: ({})", non_key, any_val);
  EXPECT_ANY_THROW(dbl_val = config[non_key].as<double>());
  EXPECT_ANY_THROW(str_val =
                       config["Thandie_Newton_plays_Maeve"].as<std::string>());

  // yaml-cpp wrapped in getters
  int verbose = 2;
  bool got_val =
      franka_driver::get_yaml_val_verbose(params, non_key, dbl_val, verbose);
  EXPECT_FALSE(got_val);
  EXPECT_EQ(dbl_val, std::numeric_limits<double>::max());

  std::string dbl_key = "yaml_test_dbl";
  got_val =
      franka_driver::get_yaml_val_verbose(params, dbl_key, dbl_val, verbose);
  EXPECT_TRUE(got_val);
  EXPECT_TRUE(utils::EpsEq(dbl_val, 9.87654321));

  std::string key = "gravity_vector";
  std::vector<double> vec_val {M_SQRT2, M_E, M_PI};
  log()->info(
      "YAML value for {} as std::vector<double>, before get: ({} {} {})", key,
      vec_val[0], vec_val[1], vec_val[2]);
  got_val = franka_driver::get_yaml_val(params, "gravity_vector", vec_val);
  log()->info(
      "YAML value for {} as std::vector<double>,  after get: ({} {} {})", key,
      vec_val[0], vec_val[1], vec_val[2]);
  EXPECT_TRUE(got_val);
  EXPECT_EQ(vec_val[0], 0.0);

  Eigen::Vector3d gravity_vector {M_PI, M_PI_2, M_PI_4};
  log()->info("YAML value for {} as Eigen::Vector3d, before get: ({})", key,
              gravity_vector.transpose());
  got_val =
      franka_driver::get_yaml_val(params, "gravity_vector", gravity_vector);
  log()->info("YAML value for {} as Eigen::Vector3d,  after get: ({})", key,
              gravity_vector.transpose());
  EXPECT_TRUE(got_val);
  EXPECT_EQ(gravity_vector[0], 0.0);

  log()->error(
      "This is a test of dexai::Logger::log_id_size:\n{:>{}}"
      "This line shall line up with the line up one line.",
      "", dexai::log_id_size());
}

TEST(LoadParams, BadURDF) {
  ConstraintSolver* constraint_solver = 0;
  RobotParameters params;

  std::string fake_yaml = "this_is_not_a_params.yaml";
  EXPECT_ANY_THROW(util_test::load_params(fake_yaml, params, true));

  EXPECT_TRUE(util_test::load_params(empty_string, params, true));

  params.urdf_dir = utils::get_cwd();
  params.UpdateUrdf(params.urdf);

  try {
    constraint_solver = new ConstraintSolver(&params);
    SUCCEED();
  } catch (std::exception const& err) {
    FAIL() << "CAUGHT AN EXCEPTION: " << err.what() << std::endl;
  }
  // ASSERT_NO_THROW(constraint_solver = new ConstraintSolver(&params));

  params.UpdateUrdf("this_is_not_a_urdf.urdf");

  delete constraint_solver;

  EXPECT_ANY_THROW(constraint_solver = new ConstraintSolver(&params));
}

TEST(LoadParams, UseEmpty) {
  ConstraintSolver* constraint_solver = 0;
  RobotParameters params;

  EXPECT_TRUE(util_test::load_params(empty_string, params, true));

  EXPECT_NO_THROW(constraint_solver = new ConstraintSolver(&params));
  delete constraint_solver;
}

TEST(UpdateParams, UpdateRobotName) {
  std::string empty_yaml_path;
  RobotParameters params;
  int verbose = 2;
  bool loaded = false;
  EXPECT_NO_THROW(loaded =
                      util_test::load_params(empty_yaml_path, params, verbose));
  EXPECT_TRUE(loaded);

  std::string old_robot_name = params.robot_name;
  std::string new_robot_name = "NEW_ROBOT_NAME";

  EXPECT_NE(params.robot_name, new_robot_name);
  EXPECT_EQ(params.lcm_plan_channel, old_robot_name + "_PLAN");

  params.UpdateRobotName(new_robot_name);

  EXPECT_EQ(params.robot_name, new_robot_name);
  EXPECT_EQ(params.lcm_plan_channel, new_robot_name + "_PLAN");
}
