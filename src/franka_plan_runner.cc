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

int do_main(std::string robot_ip_addr, std::string param_yaml="dracula_test.yaml") {
    create_momap_log("franka_plan_runner");
    FrankaPlanRunner frankaPlanRunner(robot_ip_addr, param_yaml);
    return frankaPlanRunner.Run(); 
}

}  // namespace robot_plan_runner
}  // namespace drake


int main(int argc, char** argv) {
    if (argc != 2 && argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << " <param.yaml> (optional)" << std::endl;
        return -1;
    }
    std::string ip_addr = argv[1];
    std::remove( ip_addr.begin(), ip_addr.end(), ' ' );
    if (argc == 2){
        if (ip_addr == drake::franka_driver::home_addr ){
            std::cerr << "ip addr " << ip_addr << " == " << drake::franka_driver::home_addr << ". No param.yaml provided. Exiting." << std::endl;
            return -1; 
        }
        return drake::franka_driver::do_main(argv[1]);
    } else {
        if (ip_addr != drake::franka_driver::home_addr ){
            std::cerr << "ip addr " << ip_addr << " != " << drake::franka_driver::home_addr << ". Ignoring param.yaml spec." << std::endl;
            return -1; 
        }
        return drake::franka_driver::do_main(ip_addr, argv[2]);
    }
}
