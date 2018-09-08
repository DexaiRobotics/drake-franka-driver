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

#include <array>
#include <atomic>

#include <cassert>
#include <cmath>
#include <cstring>

#include <iostream>
#include <memory>

#include <functional>
#include <iostream>
#include <iterator>

#include <limits>
#include <mutex>
#include <poll.h>

#include <stdexcept>
#include <sys/time.h>
#include <thread>
#include <vector>

#include <gflags/gflags.h>

#include "lcm/lcm-cpp.hpp"
#include "lcmtypes/robot_spline_t.hpp"
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

// #include "iiwa_common.h" //eventually get from drake binary distro, when it is included

#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "drake/lcmt_iiwa_status.hpp"

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>

#include "examples_common.h"

#include <iostream>
#include <memory>

#include <momap/momap_robot_plan_v1.h>
#include <lcmtypes/robot_spline_t.hpp>

#include "trajectory_solver.h"
#include "log_momap.h"
#include <dracula_utils.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using drake::lcmt_iiwa_status;

namespace drake {
namespace franka_driver {

const char* const kLcmStatusChannel = "FRANKA_STATUS";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kLcmStopChannel = "STOP";
const int kNumJoints = 7;

std::atomic_bool running{false};
std::atomic_bool isRobotAlive{false};

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef PPType::PolynomialType PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

struct RobotData {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
};
template <typename T, std::size_t SIZE>
std::vector<T> ConvertToVector(std::array<T, SIZE> &a){
    std::vector<T> v(a.begin(), a.end());
    return v;
}

lcmt_iiwa_status ConvertToLcmStatus(franka::RobotState &robot_state){
    lcmt_iiwa_status robot_status{}; 
    int num_joints_ = robot_state.q.size();
    struct timeval  tv;
    gettimeofday(&tv, NULL);

    robot_status.utime = int64_t(tv.tv_sec * 1e6 + tv.tv_usec); //int64_t(1000.0 * robot_state.time.toMSec());
    robot_status.num_joints = num_joints_;
    // q
    robot_status.joint_position_measured = ConvertToVector(robot_state.q);
    robot_status.joint_position_commanded = ConvertToVector(robot_state.q_d);
    robot_status.joint_position_ipo.resize(num_joints_, 0);
    robot_status.joint_velocity_estimated = ConvertToVector(robot_state.dq);
    robot_status.joint_torque_measured = ConvertToVector(robot_state.tau_J);
    robot_status.joint_torque_commanded = ConvertToVector(robot_state.tau_J_d);
    robot_status.joint_torque_external.resize(num_joints_, 0);

    return robot_status; 
}

int do_main(std::string robot_ip_addr) {
    create_momap_log("kuka_plan_runner");
    running = true; 
    isRobotAlive = false; 
    RobotData robot_data{};
    robot_data.has_data = false; 
    
    ::lcm::LCM lcm_;

    lcm_.subscribe(kLcmPlanChannel, &HandlePlan, this);

    int plan_number_{};
    std::unique_ptr<PiecewisePolynomial<double>> plan_;
    lcmt_iiwa_status iiwa_status_;
    PPType piecewise_polynomial;

    // Set and initialize trajectory parameters.
    const double radius = 0.05;
    const double vel_max = 0.25;
    const double acceleration_time = 2.0;
    const double run_time = 20.0;
    // Set print rate for comparing commanded vs. measured torques.
    const double print_rate = 10.0;

    double vel_current = 0.0;
    double angle = 0.0;
    double time = 0.0; 

    // Set print rate for comparing commanded vs. measured torques.
    const double lcm_publish_rate = 200.0; //Hz
    // Start print thread.
    std::thread lcm_publish_status_thread([lcm_publish_rate, &robot_data, &running, &lcm_]() {
        while (running) {
            // Sleep to achieve the desired print rate.
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>( 1000.0/lcm_publish_rate )));
            // Try to lock data to avoid read write collisions.
            if (robot_data.mutex.try_lock()) {
                if (robot_data.has_data) {
                    lcmt_iiwa_status franka_status = ConvertToLcmStatus(robot_data.robot_state); 

                    // publish data over lcm
                    lcm_.publish(kLcmStatusChannel, &franka_status);

                    robot_data.has_data = false;
                }
                robot_data.mutex.unlock();
            }
        }
    });

    try {
        // Connect to robot.
        franka::Robot robot(robot_ip_addr);
        setDefaultBehavior(robot);
        isRobotAlive = true; 

        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot! "
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;

        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set collision behavior.
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        // Load the kinematics and dynamics model.
        franka::Model model = robot.loadModel();

        std::array<double, 7> goal_joint_conifg;

        // Define callback for the joint torque control loop.
        robot.control([&goal_joint_conifg, &time, &robot_data](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
            time += period.toSec();

            if (time == 0.0) {
                goal_joint_conifg = robot_state.q_d;
            }

            franka::JointPositions output = {{ goal_joint_conifg[0], goal_joint_conifg[1],
                                               goal_joint_conifg[2], goal_joint_conifg[3],
                                               goal_joint_conifg[4], goal_joint_conifg[5],
                                               goal_joint_conifg[6] }};
            // Update data to publish.
            if (robot_data.mutex.try_lock()) {
                robot_data.has_data = true;
                robot_data.robot_state = robot_state;
                robot_data.mutex.unlock();
            }

            // if (time >= 60.0) {
            if (0) {
                std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(output);
            }
            return output;
        });

    } catch (const franka::Exception& ex) {
        running = false;
        std::cerr << ex.what() << std::endl;
    }

    if (lcm_publish_status_thread.joinable()) {
        lcm_publish_status_thread.join();
    }
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
