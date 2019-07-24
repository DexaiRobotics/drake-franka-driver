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

#include <math.h>
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
#include <franka/rate_limiting.h>

#include "examples_common.h"

#include <iostream>
#include <memory>

// #include <momap/momap_robot_plan_v1.h>
#include <lcmtypes/robot_spline_t.hpp>
#include <robot_msgs/bool_t.hpp>

#include "trajectory_solver.h"
#include "momap/momap_log.h"
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
    std::atomic<bool> has_data;
    int64_t utime;
    std::unique_ptr<PiecewisePolynomial<double>> plan;
    int64_t end_time_us;
    bool paused;
    bool unpausing;
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


// TODO: @dmsj - make this call ConvertToLcmStatus()
static void AssignToLcmStatus(franka::RobotState &robot_state, lcmt_iiwa_status &robot_status){
    int num_joints_ = kNumJoints;
    struct timeval  tv;
    gettimeofday(&tv, NULL);
    // TODO: @dmsj store both timeofday and franka_time
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

// TODO: use this
int64_t get_current_utime() {
    struct timeval  tv;
    gettimeofday(&tv, NULL);
    int64_t current_utime = int64_t(tv.tv_sec * 1e6 + tv.tv_usec);
    return current_utime;
}

class FrankaPlanRunner {
private:
    Dracula *dracula = nullptr;
    std::string param_yaml_; 
    parameters::Parameters p;
    std::string ip_addr_;
    std::atomic_bool running_{false};
    std::atomic_bool robot_alive_{false};
    ::lcm::LCM lcm_;
    int plan_number_{};
    int cur_plan_number{};
    int64_t cur_plan_utime_{};
    int64_t cur_time_us{};
    int64_t start_time_us{};
    RobotPiecewisePolynomial plan_;
    RobotData robot_data_{}; 
    PPType piecewise_polynomial;
    std::thread lcm_publish_status_thread;
    std::thread lcm_handle_thread;
    int sign_{};
    std::atomic_bool editing_plan{false};
    std::array<double, 16> initial_pose;
    Eigen::MatrixXd joint_limits;
    long timestep;
    float target_stop_time;
    const float STOP_EPSILON = 20; //make this a yaml parameter
    float stop_epsilon;
    float stop_duration;



    // Set print rate for comparing commanded vs. measured torques.
    const double lcm_publish_rate = 200.0; //Hz
    double franka_time;
    Eigen::VectorXd max_accels;

public:
    FrankaPlanRunner(const parameters::Parameters params)
    : p(params), ip_addr_(params.robot_ip), plan_number_(0),
    lcm_(params.lcm_url) {
        lcm_.subscribe(p.lcm_plan_channel, &FrankaPlanRunner::HandlePlan, this);
        lcm_.subscribe(p.lcm_stop_channel, &FrankaPlanRunner::HandleStop, this);
        running_ = true;
        franka_time = 0.0; 
        max_accels = params.robot_max_accelerations;

        dracula = new Dracula(p);
        joint_limits = dracula->GetCS()->GetJointLimits();
        momap::log()->info("Joint limits: {}", joint_limits.transpose());

        cur_plan_number = plan_number_;
        cur_time_us = -1;
        start_time_us = -1;
        sign_ = +1; 

        plan_.has_data = false; 
        plan_.utime = -1;
        plan_.end_time_us = 0;
        plan_.paused = false;
        plan_.unpausing = false;
        momap::log()->info("Plan channel: {}", p.lcm_plan_channel);
        momap::log()->info("Stop channel: {}", p.lcm_stop_channel);
        momap::log()->info("Plan received channel: {}", p.lcm_plan_received_channel);
        momap::log()->info("Plan complete channel: {}", p.lcm_plan_complete_channel);
        momap::log()->info("Status channel: {}", p.lcm_status_channel);
    
    };

    ~FrankaPlanRunner() {
        // if (lcm_publish_status_thread.joinable()) {
        //     lcm_publish_status_thread.join();
        // }
    };
    int Run() {
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
    int RunFranka() {
        // const double print_rate = 10.0;
        // struct {
        //     std::mutex mutex;
        //     bool has_data;
        //     std::array<double, 7> tau_d_last;
        //     franka::RobotState robot_state;
        //     std::array<double, 7> gravity;
        // } print_data{};

        // Start print thread.
        // std::thread print_thread([print_rate, &print_data, this]() {
        //     while (this->running_) {
        //     // Sleep to achieve the desired print rate.
        //     std::this_thread::sleep_for(
        //         std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));

        //     // Try to lock data to avoid read write collisions.
        //     if (print_data.mutex.try_lock()) {
        //         if (print_data.has_data) {
        //         std::array<double, 7> tau_error{};
        //         double error_rms(0.0);
        //         std::array<double, 7> tau_d_actual{};
        //         for (size_t i = 0; i < 7; ++i) {
        //             tau_d_actual[i] = print_data.tau_d_last[i] + print_data.gravity[i];
        //             tau_error[i] = tau_d_actual[i] - print_data.robot_state.tau_J[i];
        //             error_rms += std::pow(tau_error[i], 2.0) / tau_error.size();
        //         }
        //         error_rms = std::sqrt(error_rms);

        //         // Print data to console
        //         std::cout << "tau_error [Nm]: " << du::v_to_e(ConvertToVector(tau_error)).transpose() << std::endl
        //                     << "tau_commanded [Nm]: " << du::v_to_e(ConvertToVector(tau_d_actual)).transpose() << std::endl
        //                     << "tau_measured [Nm]: " << du::v_to_e(ConvertToVector(print_data.robot_state.tau_J)).transpose() << std::endl
        //                     // << "root mean square of tau_error [Nm]: " << error_rms << std::endl
        //                     << "-----------------------" << std::endl;
        //         print_data.has_data = false;
        //         }
        //         print_data.mutex.unlock();
        //     }
        //     }
        // });

        try {
            // Connect to robot.
            franka::Robot robot(ip_addr_);
            setDefaultBehavior(robot);
            robot_alive_ = true; 
            // First move the robot to a suitabcurrent_desiredle joint configuration
            // std::array<double, 7> q_goal = {current_desired{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            // MotionGenerator motion_generator(0.5, q_goal);
            // std::cout << "WARNING: This example will move the robot! "
            //         << "Please make sure to have the user stop button at hand!" << std::endl
            //         << "Press Enter to continue..." << std::endl;
            // std::cin.ignore();
            // robot.control(motion_generator);
            // std::cout << "Finished moving to initial joint configuration." << std::endl;
            std::cout << "Ready." << std::endl;

            // Set additional parameters always before the control loop, NEVER in the control loop!
            // Set collision behavior.

	    bool we_care_about_safety = false;
	    if (we_care_about_safety) {
            robot.setCollisionBehavior(
                {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}}, {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
                {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}}, {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
                {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}}, {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}},
                {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}}, {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}});
	    } else {
            robot.setCollisionBehavior(
                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        }

            std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
                joint_position_callback = [&, this](
                        const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
                return this->FrankaPlanRunner::JointPositionCallback(robot_state, period);
            };

            
            bool stop_signal = false; 
            while(!stop_signal){
                // std::cout << "top of loop: Executing motion." << std::endl;
                try {
                    if (plan_.has_data) {
                        robot.control(joint_position_callback); //impedance_control_callback
                    } else {
                        // publish robot_status
                        // TODO: add a timer to be closer to 200 Hz. 
                        // std::cout << "only should be here when sitting.\n";
                        robot.read([this](const franka::RobotState& robot_state) {
                            if (this->robot_data_.mutex.try_lock()) {
                                this->robot_data_.has_data = true;
                                this->robot_data_.robot_state = robot_state;
                                this->robot_data_.mutex.unlock();
                            }
                            std::this_thread::sleep_for(
                                    std::chrono::milliseconds(static_cast<int>( 1.0 / lcm_publish_rate )));
                            return false;
                        });
                    }
                    
                } catch (const franka::ControlException& e) {
                    std::cout << e.what() << std::endl;
                    std::cout << "Running error recovery..." << std::endl;
                    robot.automaticErrorRecovery();
                }
            }

        } catch (const franka::Exception& ex) {
            running_ = false;
            momap::log()->error("drake::franka_driver::RunFranka Caught expection: {}", ex.what() );
            // return -99; // bad things happened. 
        }
        // if (print_thread.joinable()) {
        //     print_thread.join();
        // }
        return 0; 
        
    };

    int RunSim() {
        robot_alive_ = true; // the sim robot *always* starts as planned
        momap::log()->info("Starting sim robot.");
        // first, load some parameters
        dracula->MutableViz()->loadRobot();
        Eigen::VectorXd next_conf = Eigen::VectorXd::Zero(kNumJoints); // output state
        next_conf << -0.9577375507190063, -0.7350638062912122, 0.880988748620542, -2.5114236381136448, 0.6720116891296624, 1.9928838396072361, -1.2954019628351783; // set robot in a starting position which is not in collision
        Eigen::VectorXd prev_conf;
        std::vector<double> vel(7,1);
        franka::RobotState robot_state; // internal state; mapping to franka state
        franka::Duration period;
        milliseconds last_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

        while(1){
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>( 1000.0/lcm_publish_rate )));

            std::vector<double> next_conf_vec = du::e_to_v(next_conf);
            ConvertToArray(next_conf_vec, robot_state.q);
            ConvertToArray(next_conf_vec, robot_state.q_d);
            ConvertToArray(vel, robot_state.dq);

 
            franka::JointPositions cmd_pos = JointPositionCallback(robot_state, period);

            prev_conf = next_conf.replicate(1,1);

            next_conf = du::v_to_e(ConvertToVector(cmd_pos.q)); 
            dracula->GetViz()->displayState(next_conf);

            next_conf_vec = du::e_to_v(next_conf);
            std::vector<double> prev_conf_vec =  du::e_to_v(prev_conf);

            for(int i=0; i<7; i++){
                vel[i] = (next_conf_vec[i] - prev_conf_vec[i]) / (double) period.toSec();                
            }

            milliseconds current_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
            int64_t delta_ms = int64_t( (current_ms - last_ms).count() );            
            period = franka::Duration(delta_ms);
            last_ms = current_ms;

        }
        return 0; 
    };

    double StopPeriod(double period){
        /*Logistic growth function: t' = f - 4 / [a(e^{at}+1] where
        f = target_stop time, t' = franka time, t = real time
        Returns delta t', the period that should be incremented to franka time*/
        double a = 2 / target_stop_time;
        double current_time = (this->target_stop_time-4/(a*(exp(a*period*this->timestep)+1)));
        double prev_time = (this->target_stop_time-4/(a*(exp(a*period*(this->timestep-1))+1)));
        return (current_time - prev_time);
    }

    franka::JointPositions JointPositionCallback( const franka::RobotState& robot_state
                                                , franka::Duration period
    ) {
        franka::JointPositions output = robot_state.q; // should this be robot_state.q_d?
        if ( plan_.mutex.try_lock() ) {
            // we got the lock, so try and do stuff.
            // momap::log()->info("got the lock!");

            // TODO: remove the need for this check. who cares if it is a new plan?
            // TODO: make sure we've called motion finished and reset the timer?
            if (plan_number_ != cur_plan_number) {
                momap::log()->info("Starting new plan.");
                start_time_us = cur_time_us; // implies that we should have call motion finished
                cur_plan_number = plan_number_;
            }

            if (plan_.paused) {
                if (target_stop_time == 0) { //if target_stop_time not set, set target_stop_time
                    std::array<double,7> vel = robot_state.dq;
                    float temp_target_stop_time = 0;
                    for (int i = 0; i < 7; i++) {
                        float stop_time = vel[i] / this->max_accels[i];
                        if(stop_time > temp_target_stop_time){
                            temp_target_stop_time = stop_time;
                        }
                    }
                    this->target_stop_time = temp_target_stop_time;
                    this->stop_epsilon = period.toSec() / STOP_EPSILON;
                }

                double new_stop = StopPeriod(period.toSec());
                franka_time += new_stop;
                // cout.precision(17);
                // cout << "S - OG PERIOD: " << period.toSec() << "  PERIOD: " << fixed << new_stop << endl;
                timestep++;
                if(new_stop > this->stop_epsilon){
                    this->stop_duration++;
                }
            } else if (!plan_.paused && plan_.unpausing) { //robot is unpausing
                if (timestep >= 0) { //if robot has reached full speed again
                    std::unique_lock<std::mutex> lck(plan_.mutex);
                    plan_.unpausing = false;
                }
                double new_stop = StopPeriod(period.toSec());
                franka_time += new_stop;
                // cout.precision(17);
                // cout << "C - OG PERIOD: " << period.toSec() << "  PERIOD: " << fixed << new_stop << " " << this->target_stop_time << " " << this->timestep << endl;
                timestep++;
                
            }
            else {
                franka_time += period.toSec();
            }

            cur_time_us = int64_t(franka_time * 1.0e6); 

            // Update data to publish.
            // TODO: move to publish loop
            if (robot_data_.mutex.try_lock()) {
                robot_data_.has_data = true;
                robot_data_.robot_state = robot_state;
                robot_data_.mutex.unlock();
            }

            Eigen::VectorXd desired_next = Eigen::VectorXd::Zero(kNumJoints);
            std::array<double, 7> current_cmd = robot_state.q_d; // set to actual, not desired
            std::array<double, 7> current_conf = robot_state.q_d; // set to actual, not desired
            desired_next = du::v_to_e( ConvertToVector(current_cmd) );
            
            double error = DBL_MAX; 

            const double cur_traj_time_s = static_cast<double>(cur_time_us - start_time_us) / 1e6;
            if (plan_.plan) {
                desired_next = plan_.plan->value(cur_traj_time_s);
                // TODO: remove - not DRY
                for (int j = 0; j < desired_next.size(); j++) {
                    if (desired_next(j) > joint_limits(j, 1) ){
                        desired_next(j) = joint_limits(j,1);
                    } else if ( desired_next(j) < joint_limits(j, 0)) {
                        desired_next(j) = joint_limits(j,0);
                    }
                }

                error = ( du::v_to_e( ConvertToVector(current_conf) ) - plan_.plan->value(plan_.plan->end_time()) ).norm();
                // set desired position based on interpolated spline
                output = {{ desired_next[0], desired_next[1],
                            desired_next[2], desired_next[3],
                            desired_next[4], desired_next[5],
                            desired_next[6] }};

                // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
                // output = q_goal; 
        
                if (franka_time > plan_.plan->end_time()) {
                    if (error < 0.007) { // TODO: replace with non arbitrary number
                        franka::JointPositions ret_val = current_conf;
                        std::cout << std::endl << "Finished motion, exiting controller" << std::endl;
                        plan_.plan.release();
                        plan_.has_data = false; 
                        // plan_.utime = -1;
                        plan_.mutex.unlock();
                        
                        PublishUtimeToChannel(plan_.utime, p.lcm_plan_complete_channel);
                        return output;
                        // plan_.mutex.unlock();
                        // return franka::MotionFinished(output);
                    }
                    else {
                        momap::log()->info("Plan running overtime and not converged, error: {}", error);
                        momap::log()->info("q:   {}", du::v_to_e( ConvertToVector(current_conf)).transpose());
                        momap::log()->info("q_d: {}", desired_next.transpose());
                    }
                }
            }
        
            plan_.mutex.unlock();
            return output;

        }
            
        // we couldn't get the lock, so probably need to return motion::finished()
        // plan_.mutex.unlock();
        return franka::MotionFinished(output);
    };

    void HandleLcm() {
        while (true) {
            lcm_.handleTimeout(0);
            usleep(1e3 * 1); //$ sleep 1ms
        }
    }

    void PublishLcmStatus() { //::lcm::LCM &lcm, RobotData &robot_data, std::atomic_bool &running
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

    //$ TODO: use a different, simpler LCM type for this?
    void PublishUtimeToChannel(int64_t utime, std::string lcm_channel) {
        lcmt_iiwa_status dummy_status;
        ResizeStatusMessage(dummy_status);
        dummy_status.utime = utime;
        lcm_.publish(lcm_channel, &dummy_status);
    }
    
    void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&, const robot_spline_t* rst)
    {
        momap::log()->info("New plan received.");
        if (! robot_alive_) {
            momap::log()->info("Discarding plan, no status message received yet from the robot");
            return;
        }
        // plan_.mutex.lock();
        while( ! plan_.mutex.try_lock()) {
            momap::log()->warn("trying to get a lock on the plan_.mutex. Sleeping 1 ms and trying again.");
            std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>( 1.0 )));
        }
        

        editing_plan = true;
    
        momap::log()->info("utime: {}", rst->utime);
        plan_.utime = rst->utime;
        //$ publish confirmation that plan was received with same utime
        PublishUtimeToChannel(plan_.utime, p.lcm_plan_received_channel);
        momap::log()->info("Published confirmation of received plan");

        franka_time = 0.0;
        piecewise_polynomial = TrajectorySolver::RobotSplineTToPPType(*rst);

        if (piecewise_polynomial.get_number_of_segments()<1)
        {
            momap::log()->info("Discarding plan, invalid piecewise polynomial.");
            plan_.mutex.unlock();
            return;
        }

        momap::log()->info("start time: {}", piecewise_polynomial.start_time());
        momap::log()->info("end time: {}", piecewise_polynomial.end_time());

        // Start position == goal position check
        // TODO: add end position==goal position check (upstream)
        // TODO: change to append initial position and respline here
        VectorXd commanded_start = piecewise_polynomial.value(piecewise_polynomial.start_time());
        for (int joint = 0; joint < rst->dof; joint++)
        {
            if (!du::EpsEq(commanded_start(joint),robot_data_.robot_state.q[joint], 0.05))//FIXME: non-arbitrary tolerance
            {
                momap::log()->info("Discarding plan, mismatched start position.");
                plan_.has_data = false;
                plan_.plan.release();
                plan_.mutex.unlock();
                return;
            }
        }

        plan_.plan.release();
        plan_.plan.reset(&piecewise_polynomial);
        plan_.has_data = true;

        ++plan_number_;
        momap::log()->warn("Finished Handle Plan!"); 
        editing_plan = false;
        plan_.mutex.unlock();
        
    };

    void HandleStop(const ::lcm::ReceiveBuffer*, const std::string&,
        const robot_msgs::bool_t* msg) {
        if(plan_.has_data && msg->data && !plan_.paused){
            momap::log()->info("Received pause command. Pausing plan.");
            std::unique_lock<std::mutex> lck(plan_.mutex);
            plan_.paused = true;
            plan_.mutex.unlock();

            this->target_stop_time = 0;
            this->timestep = 1;
            this->stop_duration = 0;

        }
        else if(plan_.has_data && !msg->data && plan_.paused){
            momap::log()->info("Received continue command. Continuing plan.");
            this->timestep = -1 * this->stop_duration; //how long unpausing should take
            // cout << "STOP DURATION: " << stop_duration << endl;
            std::unique_lock<std::mutex> lck(plan_.mutex);
            plan_.paused = false;
            plan_.unpausing = true;
            plan_.mutex.unlock();
        }
        

    };

        
};
} // robot_plan_runner
} // drake

#endif 
