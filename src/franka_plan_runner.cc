/// @file franka_plan_runner.cc
///
/// franka_plan_runner is designed to wait for LCM messages containing
/// a robot_spline_t message, and then execute the plan on a franka arm
/// (also reporting via LCM lcmt_franka_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

#include "franka_plan_runner.h"

namespace drake {
namespace franka_driver {

int do_main(std::string robot_ip_addr) {
    create_momap_log("franka_plan_runner");
    FrankaPlanRunner frankaPlanRunner(robot_ip_addr);
    return frankaPlanRunner.Run(); 
}

}  // namespace robot_plan_runner
}  // namespace drake


int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    return drake::franka_driver::do_main(argv[1]);
}
