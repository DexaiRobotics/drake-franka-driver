/// @file franka_driver.cc
///
/// franka_driver runs an instance of franka_plan_runner

#include "franka_plan_runner.h"
#include "drac_util_io.h"            // for lock_pid_file

namespace dru = dracula_utils;

namespace franka_driver {

int do_main(std::string param_yaml = "franka_test.yaml") {
  create_momap_log("franka_driver");
  int verbose = 0;
  momap::log()->info("Loading parameters: {}", param_yaml);
  parameters::Parameters params =
      parameters::loadYamlParameters(param_yaml, verbose);
  FrankaPlanRunner frankaPlanRunner(params);
  return frankaPlanRunner.Run();
}

}  // namespace franka_driver

int main(int argc, char** argv) {

  // Ensure app is singleton (added by 5yler):
  std::string pid_file = "/var/run/cobot_driver.pid";
  if (! dru::lock_pid_file(pid_file)) {
    std::cerr << "Failed to set up singleton cobot driver app." << std::endl;
    return 1;
  }

  if (argc != 1 && argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <params_filepath>" << std::endl;
    return -1;
  }

  if (argc == 1) {
    momap::log()->info(
        "Loading default parameters with sim robot: franka_test.yaml");
    return franka_driver::do_main();
  } else {
    std::string param_yaml = argv[1];
    return franka_driver::do_main(param_yaml);
  }
}
