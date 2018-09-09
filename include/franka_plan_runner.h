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

#ifndef ROBOT_PLAN_RUNNER_H
#define ROBOT_PLAN_RUNNER_H

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

namespace du = dracula_utils;

namespace drake {
namespace franka_driver {

const char* const kLcmStatusChannel = "FRANKA_STATUS";
const char* const kLcmPlanChannel = "FRANKA_PLAN";
const char* const kLcmInterfaceChannel = "FRANKA_SIMPLE_INTERFACE";
const char* const kLcmStopChannel = "STOP";
const int kNumJoints = 7;

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

class FrankaPlanRunner {
private:
    std::string ip_addr_;
    std::atomic_bool running_{false};
    std::atomic_bool robot_alive_{false};
    ::lcm::LCM lcm_;
    int plan_number_{};
    std::unique_ptr<PiecewisePolynomial<double>> plan_;
    RobotData robot_data_{}; 
    PPType piecewise_polynomial;
    std::thread lcm_publish_status_thread_ptr;

    // Set print rate for comparing commanded vs. measured torques.
    const double lcm_publish_rate = 200.0; //Hz
    double franka_time;

public:
    FrankaPlanRunner(const std::string ip_addr) : ip_addr_(ip_addr), plan_number_(0)
    {
        lcm_.subscribe(kLcmPlanChannel, &FrankaPlanRunner::HandlePlan, this);
        // lcm_.subscribe(kLcmPlanChannel, &RobotPlanRunner::HandleSimpleCommand, this);
        lcm_.subscribe(kLcmStopChannel, &FrankaPlanRunner::HandleStop, this);
        running_ = true;
        franka_time = 0.0; 
        robot_data_.has_data = false; 
        
    };

    ~FrankaPlanRunner(){
        // if (lcm_publish_status_thread_ptr.joinable()) {
        //     lcm_publish_status_thread_ptr.join();
        // }
    };

    // bool ConnectToRobot();
    int Run(){
        lcm_publish_status_thread_ptr = std::thread(&FrankaPlanRunner::PublishLcmStatus, this);
        try {
            // Connect to robot.
            franka::Robot robot(ip_addr_);
            setDefaultBehavior(robot);
            robot_alive_ = true; 

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

            // Define callback for the joint position control loop.
            std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
                joint_position_callback =
                    [&goal_joint_conifg, &franka_time = franka_time, &robot_data_ = robot_data_](
                        const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
                franka_time += period.toSec();

                if (franka_time == 0.0) {
                    goal_joint_conifg = robot_state.q_d;
                }

                franka::JointPositions output = {{ goal_joint_conifg[0], goal_joint_conifg[1],
                                                goal_joint_conifg[2], goal_joint_conifg[3],
                                                goal_joint_conifg[4], goal_joint_conifg[5],
                                                goal_joint_conifg[6] }};
                // Update data to publish.
                if (robot_data_.mutex.try_lock()) {
                    robot_data_.has_data = true;
                    robot_data_.robot_state = robot_state;
                    robot_data_.mutex.unlock();
                }

                // if (time >= 60.0) {
                if (0) {
                    std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                    return franka::MotionFinished(output);
                }
                return output;
            };

            robot.control(joint_position_callback);

        } catch (const franka::Exception& ex) {
            running_ = false;
            std::cerr << ex.what() << std::endl;
        }
        if (lcm_publish_status_thread_ptr.joinable()) {
            lcm_publish_status_thread_ptr.join();
        }
        return 0;
    };

private:
    void PublishLcmStatus(){ //::lcm::LCM &lcm, RobotData &robot_data, std::atomic_bool &running
        while (running_) {
            // Sleep to achieve the desired print rate.
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>( 1000.0/lcm_publish_rate )));
            // Try to lock data to avoid read write collisions.
            if (robot_data_.mutex.try_lock()) {
                if (robot_data_.has_data) {
                    lcmt_iiwa_status franka_status = ConvertToLcmStatus(robot_data_.robot_state); 
                    // publish data over lcm
                    lcm_.publish(kLcmStatusChannel, &franka_status);
                    robot_data_.has_data = false;
                }
                robot_data_.mutex.unlock();
            }
        }
    };
    
    void HandlePlan(const lcm::ReceiveBuffer*, const std::string&, const robot_spline_t* rst)
    {
        momap::log()->info("New plan received.");
        if (! robot_alive_) {
            momap::log()->info("Discarding plan, no status message received yet from the robot");
            return;
        }

        piecewise_polynomial = TrajectorySolver::RobotSplineTToPPType(*rst);
        if (piecewise_polynomial.get_number_of_segments()<1)
        {
            momap::log()->info("Discarding plan, invalid piecewise polynomial.");
            return;
        }

        momap::log()->info("utime: {}", rst->utime);
        momap::log()->info("start time: {}", piecewise_polynomial.start_time());
        momap::log()->info("end time: {}", piecewise_polynomial.end_time());

        //Naive implementation of velocity check
        const int velocity_check_samples = 100;
        PPType piecewise_polynomial_derivative = piecewise_polynomial.derivative();
        double step_size = (piecewise_polynomial.end_time()-piecewise_polynomial.start_time())/velocity_check_samples; //FIXME: numer
        double test_time = piecewise_polynomial.start_time();
        while(test_time<piecewise_polynomial.end_time())
        {
            VectorXd sampled_velocity = piecewise_polynomial_derivative.value(test_time);
            //Check dimension
            if (sampled_velocity.size()!= rst->dof)
            {
                momap::log()->info("Discarding plan, invalid piecewise polynomial derivative.");
                return;
            }
            for (int joint = 0; joint < rst->dof; joint++)
            {
                if (sampled_velocity(joint)>rst->robot_joints[joint].velocity_upper_limit||
                sampled_velocity(joint)<rst->robot_joints[joint].velocity_lower_limit)
                {
                    momap::log()->info("Discarding plan, joint velocity out of bounds.");
                    momap::log()->info("sampled joint {} velocity: {}", joint, sampled_velocity(joint));
                    momap::log()->info("lower limit: {} upper limit: {}", rst->robot_joints[joint].velocity_lower_limit, rst->robot_joints[joint].velocity_upper_limit);
                    return;
                }
            }
            test_time+=step_size;
        }

        //Start position == goal position check
        VectorXd commanded_start = piecewise_polynomial.value(piecewise_polynomial.start_time());
        for (int joint = 0; joint < rst->dof; joint++)
        {
            if (!du::EpsEq(commanded_start(joint),robot_data_.robot_state.q[joint], 0.05))//FIXME: non-arbitrary tolerance
            {
                momap::log()->info("Discarding plan, mismatched start position.");
                return;
            }
        }


        //TODO: add end position==goal position check (upstream)
        plan_.release();
        plan_.reset(&piecewise_polynomial);
        // PiecewisePolynomial<double>::Cubic(input_time, knots, knot_dot, knot_dot)));
        ++plan_number_;
    };

    void HandleStop(const lcm::ReceiveBuffer*, const std::string&,
        const robot_plan_t*) {
        momap::log()->info("Received stop command. Discarding plan.");
        plan_.reset();
    };

        
};
} // robot_plan_runner
} // drake

#endif 