/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages contraining
/// a robot_plan_t message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

#include <iostream>
#include <memory>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/robot_spline_t.hpp"
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
// #include "iiwa_common.h" //eventually get from drake binary distro, when it is included
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>

#include "examples_common.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace franka_driver {

const char* const kLcmStatusChannel = "FRANKA_STATUS";
// const char* const kLcmCommandChannel = "FRANKA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kLcmStopChannel = "STOP";
const int kNumJoints = 7;

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef PPType::PolynomialType PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

struct robot_data {
    std::mutex mutex;
    bool has_data;
    std::array<double, 7> tau_d_last;
    franka::RobotState robot_state;
    std::array<double, 7> gravity;
};

class RobotPlanRunner {
private:
    const std::string robot_ip_addr_;
    ::lcm::LCM lcm_;
    int plan_number_{};
    std::unique_ptr<PiecewisePolynomial<double>> plan_;
    // lcmt_iiwa_status iiwa_status_;
    PPType piecewise_polynomial;
    robot_data print_data; 
    std::atomic_bool running;
public:
    RobotPlanRunner(const std::string robot_ip_addr) : robot_ip_addr_(robot_ip_addr), plan_number_(0)
    {
        lcm_.subscribe(kLcmPlanChannel, &RobotPlanRunner::HandlePlan, this);
        lcm_.subscribe(kLcmStopChannel, &RobotPlanRunner::HandleStop, this);
        running = false;
        print_data.has_data = false;
    }

    int Run() {
        int cur_plan_number = plan_number_;
        int64_t cur_time_us = -1;
        int64_t start_time_us = -1;

        try {
            franka::Robot robot(robot_ip_addr_);
            setDefaultBehavior(robot);
            running = true;

            size_t count = 0;
            robot.read([&count](const franka::RobotState& robot_state) {
            // Printing to std::cout adds a delay. This is acceptable for a read loop such as this, but
            // should not be done in a control loop.
            std::cout << robot_state << std::endl;
            return count++ < 100;
            });

            std::cout << "Done." << std::endl;
    
            // while (true) {
               
            // } // whie
        } catch (const franka::Exception& e) {
            std::cout << e.what() << std::endl;
            return -1;
        }
    }

private:
    void HandlePlan(const lcm::ReceiveBuffer*, const std::string&, const lcmtypes::robot_spline_t* rst)
    {
        // momap::log()->info("New plan received.");
        // if (iiwa_status_.utime == -1) {
        //     momap::log()->info("Discarding plan, no status message received yet");
        //     return;
        // }

        // piecewise_polynomial = TrajectorySolver::RobotSplineTToPPType(*rst);
        // if (piecewise_polynomial.get_number_of_segments()<1)
        // {
        //     momap::log()->info("Discarding plan, invalid piecewise polynomial.");
        //     return;
        // }

        // momap::log()->info("utime: {}", rst->utime);
        // momap::log()->info("start time: {}", piecewise_polynomial.start_time());
        // momap::log()->info("end time: {}", piecewise_polynomial.end_time());

        // //Naive implementation of velocity check
        // const int velocity_check_samples = 100;
        // PPType piecewise_polynomial_derivative = piecewise_polynomial.derivative();
        // double step_size = (piecewise_polynomial.end_time()-piecewise_polynomial.start_time())/velocity_check_samples; //FIXME: numer
        // double test_time = piecewise_polynomial.start_time();
        // while(test_time<piecewise_polynomial.end_time())
        // {
        //     VectorXd sampled_velocity = piecewise_polynomial_derivative.value(test_time);
        //     //Check dimension
        //     if (sampled_velocity.size()!= rst->dof)
        //     {
        //         momap::log()->info("Discarding plan, invalid piecewise polynomial derivative.");
        //         return;
        //     }
        //     for (int joint = 0; joint < rst->dof; joint++)
        //     {
        //         if (sampled_velocity(joint)>rst->robot_joints[joint].velocity_upper_limit||
        //         sampled_velocity(joint)<rst->robot_joints[joint].velocity_lower_limit)
        //         {
        //             momap::log()->info("Discarding plan, joint velocity out of bounds.");
        //             momap::log()->info("sampled joint {} velocity: {}", joint, sampled_velocity(joint));
        //             momap::log()->info("lower limit: {} upper limit: {}", rst->robot_joints[joint].velocity_lower_limit, rst->robot_joints[joint].velocity_upper_limit);
        //             return;
        //         }
        //     }
        //     test_time+=step_size;
        // }

        // //Start position == goal position check
        // VectorXd commanded_start = piecewise_polynomial.value(piecewise_polynomial.start_time());
        // for (int joint = 0; joint < rst->dof; joint++)
        // {
        //     if (!du::EpsEq(commanded_start(joint),iiwa_status_.joint_position_measured[joint], 0.05))//FIXME: non-arbitrary tolerance
        //     {
        //         momap::logatomic object while()->info("Discarding plan, mismatched start position.");
        //         return;
        //     }
        // }


        // //TODO: add end position==goal position check (upstream)
        // plan_.release();
        // plan_.reset(&piecewise_polynomial);
        // // PiecewisePolynomial<double>::Cubic(input_time, knots, knot_dot, knot_dot)));
        // ++plan_number_;
    }

    void HandleStop(const lcm::ReceiveBuffer*, const std::string&, const lcmtypes::robot_spline_t*) {
        // momap::log()->info("Received stop command. Discarding plan.");
        plan_.reset();
    }
};

int do_main(std::string robot_ip_addr) {
    RobotPlanRunner runner(robot_ip_addr);
    runner.Run();
    return 0;
}

}  // namespace franka_driver
}  // namespace drake


int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    return drake::franka_driver::do_main(argv[1]);
}
