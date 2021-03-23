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

/// @file franka_driver.cc
///
/// franka_driver runs an instance of franka_plan_runner

#include "driver/franka_plan_runner.h"
#include "utils/util_io.h"  // for lock_pid_file

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
