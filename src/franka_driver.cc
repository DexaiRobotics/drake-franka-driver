/// @file franka_driver.cc
///
/// franka_driver runs an instance of franka_plan_runner

#include "franka_plan_runner.h"
#include "util_io.h"  // for lock_pid_file

namespace dru = utils;

namespace franka_driver {

int do_main(std::string param_yaml, uint verbosity) {
  dexai::create_log("franka_driver");

  switch (verbosity) {
    case 0:
      dexai::log()->set_level(spdlog::level::critical);
      break;
    case 1:
      dexai::log()->set_level(spdlog::level::err);
      break;
    case 2:
      dexai::log()->set_level(spdlog::level::warn);
      break;
    case 3:
      dexai::log()->set_level(spdlog::level::info);
      break;
    case 4:
      dexai::log()->set_level(spdlog::level::debug);
      break;
    case 5:
      dexai::log()->set_level(spdlog::level::trace);
      break;
    default:
      dexai::log()->set_level(spdlog::level::info);
      break;
  }

  dexai::log()->info("Loading parameters: {}", param_yaml);
  RobotParameters params = loadYamlParameters(param_yaml, verbosity);
  FrankaPlanRunner frankaPlanRunner(params);
  return frankaPlanRunner.Run();
}

}  // namespace franka_driver

int main(int argc, char** argv) {
  // Ensure app is singleton (added by 5yler):
  std::string pid_file = "/var/run/cobot_driver.pid";
  bool kill_existing_process = true;
  bool prompt_before_kill = false;
  uint verbosity {3};
  if (!utils::lock_pid_file(pid_file, kill_existing_process,
                            prompt_before_kill)) {
    std::cerr << "Failed to set up singleton cobot driver app." << std::endl;
    return 1;
  }

  if (argc != 2 && argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <params_filepath>" << std::endl;
    return -1;
  }

  if (argc == 3) {
    verbosity = atoi(argv[2]);
  }
  std::cout << "verbosity: " << verbosity << std::endl;

  std::string param_yaml = argv[1];
  return franka_driver::do_main(param_yaml, verbosity);
}
