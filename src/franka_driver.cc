/// @file franka_driver.cc
///
/// franka_driver runs an instance of franka_plan_runner

#include "util_io.h"  // for lock_pid_file
#include "franka_plan_runner.h"

namespace dru = utils;

namespace franka_driver {

int do_main(std::string param_yaml = "franka_test.yaml") {
  dexai::create_log("franka_driver");
  int verbose = 0;
  dexai::log()->info("Loading parameters: {}", param_yaml);
  RobotParameters params =
      loadYamlParameters(param_yaml, verbose);
  FrankaPlanRunner frankaPlanRunner(params);
  return frankaPlanRunner.Run();
}

}  // namespace franka_driver

int main(int argc, char** argv) {
  // Ensure app is singleton (added by 5yler):
  std::string pid_file = "/var/run/cobot_driver.pid";
  bool kill_existing_process = true;
  bool prompt_before_kill = false;
  if (!utils::lock_pid_file(pid_file, kill_existing_process, prompt_before_kill)) {
    std::cerr << "Failed to set up singleton cobot driver app." << std::endl;
    return 1;
  }

  if (argc != 1 && argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <params_filepath>" << std::endl;
    return -1;
  }

  if (argc == 1) {
    dexai::log()->info(
        "Loading default parameters with sim robot: franka_test.yaml");
    return franka_driver::do_main();
  } else {
    std::string param_yaml = argv[1];
    return franka_driver::do_main(param_yaml);
  }
}
