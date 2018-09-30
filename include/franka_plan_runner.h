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
#include <chrono>

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

// #include <momap/momap_robot_plan_v1.h>
#include <lcmtypes/robot_spline_t.hpp>

#include "trajectory_solver.h"
#include "log_momap.h"
#include "dracula_utils.h"
#include "mock_dracula.h"

using namespace std::chrono;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace du = dracula_utils;

namespace drake {
namespace franka_driver {

// const char* const kLcmStatusChannel = "FRANKA_STATUS";
// const char* const kLcmPlanChannel = "FRANKA_PLAN";
// const char* const kLcmPlanReceivedChannel = "FRANKA_PLAN_RECEIVED";
// const char* const kLcmInterfaceChannel = "FRANKA_SIMPLE_INTERFACE";
// const char* const kLcmStopChannel = "STOP";
// const char* const kLCMURL = "udpm://239.255.76.67:7667?ttl=2";
const int kNumJoints = 7;
const std::string home_addr = "192.168.1.1"; 


using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef PPType::PolynomialType PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

struct RobotData {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
};

struct RobotPiecewisePolynomial {
    std::mutex mutex;
    bool has_data;
    std::unique_ptr<PiecewisePolynomial<double>> plan;
};

template <typename T, std::size_t SIZE>
std::vector<T> ConvertToVector(std::array<T, SIZE> &a){
    std::vector<T> v(a.begin(), a.end());
    return v;
}
template <typename T, std::size_t SIZE>
void ConvertToArray(std::vector<T> &v, std::array<T, SIZE> &a){
    for(int i=0; i<SIZE; i++){
        a[i] = v[i];
    }
}


void AssignToLcmStatus(franka::RobotState &robot_state, lcmt_iiwa_status &robot_status){
    int num_joints_ = kNumJoints;
    struct timeval  tv;
    gettimeofday(&tv, NULL);

    robot_status.utime = int64_t(tv.tv_sec * 1e6 + tv.tv_usec); //int64_t(1000.0 * robot_state.time.toMSec());
    robot_status.num_joints = num_joints_;
    // q
    robot_status.joint_position_measured.assign(std::begin(robot_state.q), std::end(robot_state.q)) ;
    robot_status.joint_position_commanded.assign(std::begin(robot_state.q_d), std::end(robot_state.q_d)) ; // = ConvertToVector(robot_state.q_d);
    robot_status.joint_position_ipo.resize(num_joints_, 0);
    robot_status.joint_velocity_estimated.assign(std::begin(robot_state.dq), std::end(robot_state.dq)) ;// = ConvertToVector(robot_state.dq);
    robot_status.joint_torque_measured.assign(std::begin(robot_state.tau_J), std::end(robot_state.tau_J)) ; // = ConvertToVector(robot_state.tau_J);
    robot_status.joint_torque_commanded.assign(std::begin(robot_state.tau_J_d), std::end(robot_state.tau_J_d)) ; // = ConvertToVector(robot_state.tau_J_d);
    robot_status.joint_torque_external.resize(num_joints_, 0);

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

void ResizeStatusMessage(lcmt_iiwa_status &lcm_status_){
  lcm_status_.utime = -1;
  lcm_status_.num_joints = kNumJoints;
  lcm_status_.joint_position_measured.resize(kNumJoints, 0);
  lcm_status_.joint_position_commanded.resize(kNumJoints, 0);
  lcm_status_.joint_position_ipo.resize(kNumJoints, 0);
  lcm_status_.joint_velocity_estimated.resize(kNumJoints, 0);
  lcm_status_.joint_torque_measured.resize(kNumJoints, 0);
  lcm_status_.joint_torque_commanded.resize(kNumJoints, 0);
  lcm_status_.joint_torque_external.resize(kNumJoints, 0);
}

class FrankaPlanRunner {
private:
    std::string param_yaml_; 
    parameters::Parameters p;
    std::string ip_addr_;
    std::atomic_bool running_{false};
    std::atomic_bool robot_alive_{false};
    ::lcm::LCM lcm_;
    int plan_number_{};
    int cur_plan_number{};
    int64_t cur_time_us{};
    int64_t start_time_us{};
    RobotPiecewisePolynomial plan_;
    RobotData robot_data_{}; 
    PPType piecewise_polynomial;
    std::thread lcm_publish_status_thread;
    std::thread lcm_handle_thread;
    int sign_{};
    std::atomic_bool editing_plan{false};
    std::condition_variable not_editing;

    // Set print rate for comparing commanded vs. measured torques.
    const double lcm_publish_rate = 200.0; //Hz
    double franka_time;

public:
    FrankaPlanRunner(const std::string ip_addr, const parameters::Parameters params) : ip_addr_(ip_addr), p(params), plan_number_(0), lcm_(params.lcm_url)
    {
        lcm_.subscribe(p.lcm_plan_channel, &FrankaPlanRunner::HandlePlan, this);
        // lcm_.subscribe(kLcmPlanChannel, &RobotPlanRunner::HandleSimpleCommand, this);
        lcm_.subscribe(p.lcm_stop_channel, &FrankaPlanRunner::HandleStop, this);
        running_ = true;
        franka_time = 0.0; 

        cur_plan_number = plan_number_;
        cur_time_us = -1;
        start_time_us = -1;
        sign_ = +1; 

        momap::log()->info("Plan channel: {}", p.lcm_plan_channel);
        momap::log()->info("Stop channel: {}", p.lcm_stop_channel);
        momap::log()->info("Plan received channel: {}", p.lcm_plan_received_channel);
        momap::log()->info("Status channel: {}", p.lcm_status_channel);
    
    };

    ~FrankaPlanRunner(){
        // if (lcm_publish_status_thread.joinable()) {
        //     lcm_publish_status_thread.join();
        // }
    };
    int Run(){
        // start LCM threads; independent of sim vs. real robot
        lcm_publish_status_thread = std::thread(&FrankaPlanRunner::PublishLcmStatus, this);
        lcm_handle_thread = std::thread(&FrankaPlanRunner::HandleLcm, this);
        int return_value = -1; //
        if(ip_addr_ == home_addr){
            return_value = RunSim();
        } else {
            return_value = RunFranka();
        }
        // clean-up threads if they're still alive.
        if (lcm_publish_status_thread.joinable()) {
            lcm_publish_status_thread.join();
        }
        if (lcm_handle_thread.joinable()) {
            lcm_handle_thread.join();
        }
        return return_value;
    }
private: 
    int RunFranka(){
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

            std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
                joint_position_callback = [&, this](
                        const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
                return this->FrankaPlanRunner::JointPositionCallback(robot_state, period);
            };
            robot.control(joint_position_callback);

        } catch (const franka::Exception& ex) {
            running_ = false;
            momap::log()->error("drake::franka_driver::RunFranka Caught expection: {}", ex.what() );
            return -99; // bad things happened. 
        }
        return 0; 
        
    };
    int RunSim(){
        robot_alive_ = true; // the sim robot *always* starts as planned
        momap::log()->info("Starting sim robot.");
        // first, load some parameters
        Dracula *dracula = new Dracula(p);
        dracula->getViz()->loadRobot();
        Eigen::VectorXd next_conf = Eigen::VectorXd::Zero(kNumJoints); // output state
        next_conf[5] = 1.5; // set robot in a starting position which is not in collision
        franka::RobotState robot_state; // internal state; mapping to franka state
        franka::Duration period;
        milliseconds last_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

        while(1){
            std::vector<double> next_conf_vec = du::e_to_v(next_conf);
            ConvertToArray(next_conf_vec, robot_state.q);
            ConvertToArray(next_conf_vec, robot_state.q_d);

            franka::JointPositions cmd_pos = JointPositionCallback(robot_state, period);
            next_conf = du::v_to_e(ConvertToVector(cmd_pos.q)); 
            dracula->getViz()->displayState(next_conf);

            milliseconds current_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
            int64_t delta_ms = int64_t( (current_ms - last_ms).count() );            
            period = franka::Duration(delta_ms);
            last_ms = current_ms;
        }
        return 0; 
    };

    franka::JointPositions JointPositionCallback(const franka::RobotState& robot_state, franka::Duration period){
        franka_time += period.toSec();
        cur_time_us = int64_t(franka_time * 1.0e6); 

        Eigen::VectorXd desired_next = Eigen::VectorXd::Zero(kNumJoints);
        std::array<double, 7> current_conf = robot_state.q_d; // set to actual, not desired
        desired_next = du::v_to_e( ConvertToVector(current_conf) );

        // std::unique_lock<std::mutex> lck(plan_.mutex);
        // not_editing.wait(lck, [this](){return editing_plan == false;});

        if (plan_.mutex.try_lock() ){
            // momap::log()->info("got the lock!");
            if (plan_.plan) {
                if (plan_number_ != cur_plan_number) {
                    momap::log()->info("Starting new plan.");
                    start_time_us = cur_time_us;
                    cur_plan_number = plan_number_;
                }

                const double cur_traj_time_s = static_cast<double>(cur_time_us - start_time_us) / 1e6;
                desired_next = plan_.plan->value(cur_traj_time_s);
            } 
            plan_.mutex.unlock();
        }
        // TODO: debug lines which move robot, remove soon @dmsj
        // if (desired_next(0) > 1.5 && sign_ > 0){
        //     sign_ = -1;
        //     momap::log()->info("set sign: {}", sign_);
        // } else if (desired_next(0) < -1.5 && sign_ < 0){
        //     sign_ = +1; 
        //     momap::log()->info("set sign: {}", sign_);
        // }
        // desired_next(0) += 0.001*sign_; 

        // set desired position based on interpolated spline
        franka::JointPositions output = {{ desired_next[0], desired_next[1],
                                           desired_next[2], desired_next[3],
                                           desired_next[4], desired_next[5],
                                           desired_next[6] }};
        // Update data to publish.
        if (robot_data_.mutex.try_lock()) {
            robot_data_.has_data = true;
            robot_data_.robot_state = robot_state;
            robot_data_.mutex.unlock();
        }

        // TODO: remove with a better way to quit @dmsj
        // if (time >= 60.0) {
        if (0) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(output);
        }
        return output;
    };

    void HandleLcm(){
        while (true) {
            lcm_.handleTimeout(0);
        }
    }

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
                    lcm_.publish(p.lcm_status_channel, &franka_status);
                    robot_data_.has_data = false;
                }
                robot_data_.mutex.unlock();
            }
        }
    };
    
    void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&, const robot_spline_t* rst)
    {
        momap::log()->info("New plan received.");
        if (! robot_alive_) {
            momap::log()->info("Discarding plan, no status message received yet from the robot");
            return;
        }

        lcmt_iiwa_status plan_received_status;
        ResizeStatusMessage(plan_received_status);
        plan_received_status.utime = rst->utime;
        //$ publish confirmation that plan was received with same utime
        //$ TODO: use a different, simpler LCM type for this?
        lcm_.publish(p.lcm_plan_received_channel, &plan_received_status);
        momap::log()->info("Published confirmation of received plan");

        std::unique_lock<std::mutex> lck(plan_.mutex);
        editing_plan = true;

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

        plan_.has_data = true;
        //TODO: add end position==goal position check (upstream)
        plan_.plan.release();
        plan_.plan.reset(&piecewise_polynomial);

        editing_plan = false;
        plan_.mutex.unlock();
        not_editing.notify_one();

        // PiecewisePolynomial<double>::Cubic(input_time, knots, knot_dot, knot_dot)));
        ++plan_number_;
    };

    void HandleStop(const ::lcm::ReceiveBuffer*, const std::string&,
        const robot_plan_t*) {
        momap::log()->info("Received stop command. Discarding plan.");

        std::unique_lock<std::mutex> lck(plan_.mutex);
        editing_plan = true;

        plan_.has_data = false;
        plan_.plan.release();

        editing_plan = false;
        plan_.mutex.unlock();
        not_editing.notify_one();

    };

        
};
} // robot_plan_runner
} // drake

#endif 