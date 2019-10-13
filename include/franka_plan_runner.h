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
#include <set>

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
#include <robot_msgs/pause_cmd.hpp>
#include <robot_msgs/trigger_t.hpp>

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
};

enum class QueuedCommand {
    NONE,
    PAUSE,
    CONTINUE
};

template <typename T, std::size_t SIZE>
std::vector<T> ConvertToVector(std::array<T, SIZE> &a) {
    std::vector<T> v(a.begin(), a.end());
    return v;
}
template <typename T, std::size_t SIZE>
void ConvertToArray(std::vector<T> &v, std::array<T, SIZE> &a) {
    for(int i = 0; i < SIZE; i++) {
        a[i] = v[i];
    }
}


// TODO: @dmsj - make this call ConvertToLcmStatus()
static void AssignToLcmStatus(franka::RobotState &robot_state, lcmt_iiwa_status &robot_status) {
    int num_joints_ = kNumJoints;
    struct timeval  tv;
    gettimeofday(&tv, NULL);
    // TODO: @dmsj store both timeofday and franka_time_
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

lcmt_iiwa_status ConvertToLcmStatus(franka::RobotState &robot_state) {
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

void ResizeStatusMessage(lcmt_iiwa_status &lcm_status_) {
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

std::string RobotModeToString(franka::RobotMode mode) {
    std::string mode_string;
    switch(mode) {
        case franka::RobotMode::kOther                  : mode_string = "Other"; break;
        case franka::RobotMode::kIdle                   : mode_string = "Idle"; break;
        case franka::RobotMode::kMove                   : mode_string = "Move"; break;
        case franka::RobotMode::kGuiding                : mode_string = "Guiding"; break;
        case franka::RobotMode::kReflex                 : mode_string = "Reflex"; break;
        case franka::RobotMode::kUserStopped            : mode_string = "User Stopped"; break;
        case franka::RobotMode::kAutomaticErrorRecovery : mode_string = "Automatic Error Recovery"; break;
    }
    return mode_string;
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
    int64_t cur_time_us_{};
    int64_t start_time_us_{};
    RobotPiecewisePolynomial plan_;
    RobotData robot_data_{}; 
    PPType piecewise_polynomial_;

    int sign_{};
    std::atomic_bool editing_plan_{false};
    std::array<double, 16> initial_pose_;
    Eigen::MatrixXd joint_limits_;
    long timestep_;
    float target_stop_time_;
    float STOP_SCALE = 0.8; //this should be yaml param
    float stop_duration_;
    std::atomic_bool pausing_;
    std::atomic_bool paused_;
    std::atomic_bool unpausing_;
    float stop_margin_counter_ = 0;
    QueuedCommand queued_cmd_ = QueuedCommand::NONE;
    std::set <std::string> stop_set_;  


    Eigen::VectorXd starting_conf_;
    std::array<double, 7> starting_franka_q_; 

    const double lcm_publish_rate_ = 200.0; //Hz
    double franka_time_;
    Eigen::VectorXd max_accels_;

    std::thread lcm_publish_status_thread;
    std::thread lcm_handle_thread;

    // not used @deprecated
    // Eigen::VectorXd integral_error; //make sure this doesn't get destroyed after each call of callback - ask sprax

    // for inv dynamics :
    bool run_inverse_dynamics_;
    std::unique_ptr<drake::systems::Context<double>> mb_plant_context_; // for multibody plants
    drake::multibody::MultibodyPlant<double> mb_plant_ ;
    Eigen::VectorXd integral_error;
    // PiecewisePolynomial<double> ref_vd_;// = plan_.plan->derivative(2); // might move this into setup

    std::string lcm_driver_status_channel_;

    Eigen::Matrix<double, 7, 1> pos_start;
    bool runonce = true;

public:
    FrankaPlanRunner(const parameters::Parameters params)
    : p(params), ip_addr_(params.robot_ip), plan_number_(0),
    lcm_(params.lcm_url) {
        lcm_.subscribe(p.lcm_plan_channel, &FrankaPlanRunner::HandlePlan, this);
        lcm_.subscribe(p.lcm_stop_channel, &FrankaPlanRunner::HandleStop, this);
        running_ = true;
        franka_time_ = 0.0; 
        max_accels_ = params.robot_max_accelerations;

        dracula = new Dracula(p);
        lcm_driver_status_channel_ = p.robot_name + "_DRIVER_STATUS";
        joint_limits_ = dracula->GetCS()->GetJointLimits();
        momap::log()->info("Joint limits: {}", joint_limits_.transpose());

        cur_plan_number = plan_number_;
        cur_time_us_ = -1;
        start_time_us_ = -1;
        sign_ = +1; 

        pausing_ = false;
        paused_ = false;
        unpausing_ = false;

        starting_franka_q_ = {{0,0,0,0,0,0,0}}; 
        starting_conf_ = Eigen::VectorXd::Zero(kNumJoints);

        plan_.has_data = false; 
        plan_.plan.release();
        plan_.utime = -1;
        plan_.end_time_us = 0;
        momap::log()->info("Plan channel: {}", p.lcm_plan_channel);
        momap::log()->info("Stop channel: {}", p.lcm_stop_channel);
        momap::log()->info("Plan received channel: {}", p.lcm_plan_received_channel);
        momap::log()->info("Plan complete channel: {}", p.lcm_plan_complete_channel);
        momap::log()->info("Status channel: {}", p.lcm_status_channel);

    };

    ~FrankaPlanRunner() {};

    int Run() {
        // start LCM threads; independent of sim vs. real robot
        lcm_publish_status_thread = std::thread(&FrankaPlanRunner::PublishLcmStatus, this);
        lcm_handle_thread = std::thread(&FrankaPlanRunner::HandleLcm, this);
        int return_value = -1; //
        if (ip_addr_ == home_addr) {
            return_value = RunSim();
        } else {
            return_value = RunFranka();
        }
        momap::log()->debug("Before LCM thread join");

        running_ = false;

        // clean-up threads if they're still alive.
        while ( ! lcm_handle_thread.joinable() ||
                ! lcm_publish_status_thread.joinable()) {
            usleep(1e5 * 1);
            momap::log()->info("Waiting for LCM threads to be joinable...");
        }

        lcm_publish_status_thread.join();
        lcm_handle_thread.join();

        momap::log()->debug("After LCM thread join");
        return return_value;
    }
private:
    int RunFranka() {

        //$ attempt connection to robot and read current mode
        //$ return if connection fails, or robot is in a mode that cannot receive commands
        try {
            franka::Robot robot(ip_addr_);

            size_t count = 0;
            franka::RobotMode current_mode;
            robot.read([&count, &current_mode](const franka::RobotState& robot_state) {
                current_mode = robot_state.robot_mode;
                return count++ < 100;
            });
            momap::log()->info("Current mode: {}", RobotModeToString(current_mode));
            
            if (current_mode != franka::RobotMode::kIdle) {
                momap::log()->info("Robot cannot receive commands in mode: {}", RobotModeToString(current_mode));
                PublishTriggerToChannel(get_current_utime(), lcm_driver_status_channel_, false, RobotModeToString(current_mode));
                return 1;
            }

        } catch (franka::Exception const& e) {
            std::cout << e.what() << std::endl; 
            PublishTriggerToChannel(get_current_utime(), lcm_driver_status_channel_, false, e.what());
            return -1;
        }

        try {
            // Connect to robot.
            franka::Robot robot(ip_addr_);
            setDefaultBehavior(robot);
            robot_alive_ = true; 

            std::cout << "Ready." << std::endl;
            PublishTriggerToChannel(get_current_utime(), lcm_driver_status_channel_, true);

            // Set additional parameters always before the control loop, NEVER in the control loop!
            // Set collision behavior.

            bool we_care_about_safety = true;
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

            //callback for inverseDynamics - i think all this code should be refactored at some point...
            std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                inverse_dynamics_control_callback = [&, this](
                        const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
                return this->FrankaPlanRunner::InverseDynamicsControlCallback(robot_state, period);
            };

            //for inverse dynamics
            run_inverse_dynamics_ = true; // TODO : move this somewhere else in the class?
            if(run_inverse_dynamics_){
                MultibodySetUp(mb_plant_, mb_plant_context_, "/src/drake-franka-driver/tests/data/franka_test_with_mass_no_joint_limits.urdf" );
                // std::cout << "done multibody setup" << '\n';
                 // ref_vd_ = plan_.plan->derivative(2);
            }
            
            //$ main control loop
            while(true) {
                // std::cout << "top of loop: Executing motion." << std::endl;
                try {
                    if (plan_.has_data && !paused_) { //$ prevent the plan from being started if robot is paused_
                        if(run_inverse_dynamics_) {
                            robot.control(inverse_dynamics_control_callback);
                        }
                        else{
                            robot.control(joint_position_callback);
                        }
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
                                    std::chrono::milliseconds(static_cast<int>( 1000.0 / lcm_publish_rate_ )));
                            return false;
                        });
                    }
                    
                } catch (const franka::ControlException& e) {
                    std::cout << e.what() << std::endl;
                    std::cout << "Running error recovery..." << std::endl;
                    momap::log()->error("FRANKA ERROR. returning -99.");
                    PublishTriggerToChannel(get_current_utime(), lcm_driver_status_channel_, false, e.what());
                    return -99; 

                    //$ UNCOMMENT BELOW TO PERFORM AUTOMATIC ERROR RECOVERY
                    // if (plan_.mutex.try_lock() ) {
                    //     robot.automaticErrorRecovery();    
                    //     plan_.mutex.unlock(); 
                    // } else {
                    //     momap::log()->error("failed to get a mutex after an error. returning -99.");
                    //     PublishTriggerToChannel(get_current_utime(), lcm_driver_status_channel_, false, e.what());
                    //     return -99; 
                    // }
                }
            }

        } catch (const franka::Exception& ex) {
            running_ = false;
            momap::log()->error("drake::franka_driver::RunFranka Caught expection: {}", ex.what() );
            PublishTriggerToChannel(get_current_utime(), lcm_driver_status_channel_, false, ex.what());
            return -99; // bad things happened. 
        }
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

        while(1) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>( 1000.0/lcm_publish_rate_ )));

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

            for(int i = 0; i < 7; i++) {
                vel[i] = (next_conf_vec[i] - prev_conf_vec[i]) / (double) period.toSec();                
            }

            milliseconds current_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
            int64_t delta_ms = int64_t( (current_ms - last_ms).count() );
            period = franka::Duration(delta_ms);
            last_ms = current_ms;

        }
        return 0;
    };

    double StopPeriod(double period) {
        /*Logistic growth function: t' = f - 4 / [a(e^{at}+1] where
        f = target_stop time, t' = franka time, t = real time
        Returns delta t', the period that should be incremented to franka time*/
        double a = 2 / target_stop_time_;
        double current_time = (this->target_stop_time_-4/(a*(exp(a*period*this->timestep_)+1)));
        double prev_time = (this->target_stop_time_-4/(a*(exp(a*period*(this->timestep_-1))+1)));
        return (current_time - prev_time);
    }

    void QueuedCmd() {
        robot_msgs::pause_cmd msg;
        msg.utime = get_current_utime();
        switch(queued_cmd_) {
            case QueuedCommand::NONE     : return;
            case QueuedCommand::PAUSE    : msg.data = true; break;
            case QueuedCommand::CONTINUE : msg.data = false; break;
        }
        msg.source = "queued";
        lcm_.publish(p.lcm_stop_channel, &msg);
        queued_cmd_ = QueuedCommand::NONE;
    }

    franka::JointPositions JointPositionCallback( const franka::RobotState& robot_state
                                                , franka::Duration period
    ) {
        franka::JointPositions output = robot_state.q_d; // should this be robot_state.q_d?

        if ( plan_.mutex.try_lock() ) {
            // we got the lock, so try and do stuff.
            // momap::log()->info("got the lock!");

            if (pausing_) {
                if (target_stop_time_ == 0) { //if target_stop_time_ not set, set target_stop_time_
                    std::array<double,7> vel = robot_state.dq;
                    float temp_target_stop_time_ = 0;
                    for (int i = 0; i < 7; i++) {
                        float stop_time = fabs(vel[i] / (this->max_accels_[i])); //sets target stop_time in plan as max(vel_i/max_accel_i), where i is each joint. real world stop time ~ 2x stop_time in plan
                        if (stop_time > temp_target_stop_time_) {
                            temp_target_stop_time_ = stop_time;
                        }
                    }
                    target_stop_time_ = temp_target_stop_time_ / STOP_SCALE;
                    momap::log()->debug("TARGET: {}", target_stop_time_);
                }

                double new_stop = StopPeriod(period.toSec());
                franka_time_ += new_stop;
                momap::log()->debug("STOP PERIOD: {}", new_stop);
                timestep_++;

                if (new_stop >= period.toSec() * p.stop_epsilon) { // robot counts as "stopped" when new_stop is less than a fraction of period
                    this->stop_duration_++;
                }
                else if (stop_margin_counter_ <= p.stop_margin) { // margin period after pause before robot is allowed to continue
                    stop_margin_counter_ += period.toSec();
                }
                else{
                    paused_ = true;
                    QueuedCmd();
                }
                
                
            } else if (unpausing_) { //robot is unpausing_
                if (timestep_ >= 0) { //if robot has reached full speed again
                    unpausing_ = false;
                    QueuedCmd();
                }
                double new_stop = StopPeriod(period.toSec());
                franka_time_ += new_stop;
                momap::log()->debug("CONTINUE PERIOD: {}", new_stop);
                timestep_++;
                
            }
            else {
                franka_time_ += period.toSec();
            }

            cur_time_us_ = int64_t(franka_time_ * 1.0e6); 

            // TODO: remove the need for this check. who cares if it is a new plan?
            // TODO: make sure we've called motion finished and reset the timer?

            if (plan_.plan && plan_number_ != cur_plan_number) {
                momap::log()->info("Starting new plan at {} s.", franka_time_);
                start_time_us_ = cur_time_us_; // implies that we should have call motion finished
                cur_plan_number = plan_number_;
                starting_conf_ = plan_.plan->value(0.0);
                starting_franka_q_ = robot_state.q_d; 
                momap::log()->warn("starting franka q = {}", du::v_to_e( ConvertToVector(starting_franka_q_) ).transpose()); 
                momap::log()->warn("difference between where we are and where we think = {}", 
                                    ( du::v_to_e( ConvertToVector(starting_franka_q_) ) - starting_conf_ ).norm() );

            }

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

            // const double cur_traj_time_s = static_cast<double>(cur_time_us_ - start_time_us_) / 1e6;
            if (plan_.plan) {
                desired_next = plan_.plan->value(franka_time_); //cur_traj_time_s
                // TODO: remove - not DRY
                for (int j = 0; j < desired_next.size(); j++) {
                    if (desired_next(j) > joint_limits_(j, 1) ) {
                        desired_next(j) = joint_limits_(j,1);
                    } else if ( desired_next(j) < joint_limits_(j, 0)) {
                        desired_next(j) = joint_limits_(j,0);
                    }
                }
                Eigen::VectorXd delta = desired_next - starting_conf_; 
                Eigen::VectorXd output_eigen = du::v_to_e( ConvertToVector(starting_franka_q_) ) + delta; 
                Eigen::VectorXd delta_end = plan_.plan->value(plan_.plan->end_time()) - starting_conf_; 
                Eigen::VectorXd starting_q_eigen = du::v_to_e( ConvertToVector(starting_franka_q_) );
                Eigen::VectorXd output_end = starting_q_eigen + delta_end; 
                Eigen::VectorXd current_conf_eigen = du::v_to_e( ConvertToVector(current_conf) );
                // error = ( du::v_to_e( ConvertToVector(current_conf) ) -  plan_.plan->value(plan_.plan->end_time()) ).norm();
                error = ( current_conf_eigen -  output_end ).norm();

                // momap::log()->warn("starting franka q = {}", du::v_to_e( ConvertToVector(starting_franka_q_) ).transpose()); 
                // momap::log()->warn("starting_q_eigen = {}", starting_q_eigen.transpose()); 
                // momap::log()->info("error: {}", error);
                // momap::log()->info("current_conf_eigen: {}", current_conf_eigen.transpose());
                // momap::log()->info("output_end: {}", output_end.transpose());

                // set desired position based on interpolated spline
                // output = {{ desired_next[0], desired_next[1],
                //             desired_next[2], desired_next[3],
                //             desired_next[4], desired_next[5],
                //             desired_next[6] }};

                output = {{ output_eigen[0], output_eigen[1],
                            output_eigen[2], output_eigen[3],
                            output_eigen[4], output_eigen[5],
                            output_eigen[6] }};

                // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
                // output = q_goal; 
        
                if (franka_time_ > plan_.plan->end_time()) {
                    if (error < 0.007) { // TODO: replace with non arbitrary number
                        franka::JointPositions ret_val = current_conf;
                        std::cout << std::endl << "Finished motion, exiting controller" << std::endl;
                        plan_.plan.release();
                        plan_.has_data = false;
                        // plan_.utime = -1;
                        plan_.mutex.unlock();
                        
                        PublishTriggerToChannel(plan_.utime, p.lcm_plan_complete_channel);
                        // return output;
                        // plan_.mutex.unlock();
                        return franka::MotionFinished(output);
                    }
                    else {
                        momap::log()->info("Plan running overtime and not converged, error: {}", error);
                        // momap::log()->info("q:   {}", du::v_to_e( ConvertToVector(current_conf)).transpose());
                        // momap::log()->info("q_d: {}", desired_next.transpose());
                    }
                }
            } else {
                // momap::log()->error("Inside JPC but plan_.plan != True");
            }
        
            plan_.mutex.unlock();
            return output;

        }
            
        // we couldn't get the lock, so probably need to return motion::finished()
        // plan_.mutex.unlock();
        return franka::MotionFinished(output);
    };

    void HandleLcm() {
        while (running_) {
            lcm_.handleTimeout(0);
            usleep(1e3 * 1); //$ sleep 1ms
        }
    }

    void PublishLcmStatus() { //::lcm::LCM &lcm, RobotData &robot_data, std::atomic_bool &running
        while (running_) {
            // Sleep to achieve the desired print rate.
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>( 1000.0/lcm_publish_rate_ )));
            
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

    void PublishTriggerToChannel(int64_t utime, std::string lcm_channel, bool success=true, std::string message="") {
        robot_msgs::trigger_t msg;
        msg.utime = utime;
        msg.success = success;
        msg.message = message;
        lcm_.publish(lcm_channel, &msg);
    }
    
    //$ check if robot is in a mode that can receive commands, i.e. not user stopped or error recovery
    bool CanReceiveCommands() {

        std::lock_guard<std::mutex> lock(robot_data_.mutex); //$ unlocks when lock_guard goes out of scope

        franka::RobotMode current_mode = robot_data_.robot_state.robot_mode;

        momap::log()->info("Current mode: {}", RobotModeToString(current_mode));
        
        //$ TODO: in the future, we may want to send commands to overwrite the current command
        //$ currently, can only accept a plan if not already running one
        if (current_mode == franka::RobotMode::kIdle) {
            return true;
        }

        momap::log()->error("CanReceiveCommands: Wrong mode!");
        return false;
    }

    void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&, const robot_spline_t* rst)
    {
        momap::log()->info("New plan received.");
        if ( ! robot_alive_) {
            momap::log()->info("Discarding plan, no status message received yet from the robot");
            return;
        }

        //$ check if in proper mode to receive commands
        if ( ! CanReceiveCommands()) {
            momap::log()->error("Discarding plan, in wrong mode!");
            return;
        }
        
        // plan_.mutex.lock();
        while( ! plan_.mutex.try_lock()) {
            momap::log()->warn("trying to get a lock on the plan_.mutex. Sleeping 1 ms and trying again.");
            std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>( 1.0 )));
        }

        editing_plan_ = true;
    
        momap::log()->info("utime: {}", rst->utime);
        plan_.utime = rst->utime;
        //$ publish confirmation that plan was received with same utime
        PublishTriggerToChannel(plan_.utime, p.lcm_plan_received_channel);
        momap::log()->info("Published confirmation of received plan");

        franka_time_ = 0.0;
        piecewise_polynomial_ = TrajectorySolver::RobotSplineTToPPType(*rst);

        if (piecewise_polynomial_.get_number_of_segments()<1)
        {
            momap::log()->info("Discarding plan, invalid piecewise polynomial.");
            plan_.mutex.unlock();
            return;
        }

        momap::log()->info("start time: {}", piecewise_polynomial_.start_time());
        momap::log()->info("end time: {}", piecewise_polynomial_.end_time());

        // Start position == goal position check
        // TODO: add end position==goal position check (upstream)
        // TODO: change to append initial position and respline here
        VectorXd commanded_start = piecewise_polynomial_.value(piecewise_polynomial_.start_time());
        for (int joint = 0; joint < rst->dof; joint++)
        {
            if ( ! du::EpsEq(commanded_start(joint), robot_data_.robot_state.q[joint], p.kMediumJointDistance))
            {
                momap::log()->info("Discarding plan, mismatched start position {} vs robot_q {}.",
                                    commanded_start(joint), robot_data_.robot_state.q[joint] );
                plan_.has_data = false;
                plan_.plan.release();
                plan_.mutex.unlock();
                return;
            }
        }

        plan_.plan.release();
        plan_.plan.reset(&piecewise_polynomial_);
        plan_.has_data = true; 


        ++plan_number_;
        momap::log()->warn("Finished Handle Plan!"); 
        editing_plan_ = false;
        plan_.mutex.unlock();
        
    };

    void HandleStop(const ::lcm::ReceiveBuffer*, const std::string&,
        const robot_msgs::pause_cmd* msg) {
        if (msg->data) { //if pause command received
            if (msg->source != "queued") {
                stop_set_.insert(msg->source);
            }
            momap::log()->info("Received pause from {}", msg->source);
            if ( ! pausing_) { //if this is first stop received
                if ( ! unpausing_) { //if robot isn't currently unpausing_
                    Pause();
                }
                else { //if robot is currently unpausing_, queue pause cmd
                    queued_cmd_ = QueuedCommand::PAUSE;
                }
            }
        }
        else if ( ! msg->data) { //if unpause command received
            momap::log()->info("Received continue from {}", msg->source);
            if (stop_set_.find(msg->source) != stop_set_.end() || msg->source == "queued") { //force continue if msg->source == queued in order for queue to work
                if (stop_set_.find(msg->source) != stop_set_.end()) {
                   stop_set_.erase(msg->source); 
                }

                if (stop_set_.size() == 0) {
                    if (paused_) { //if robot is currently paused_, run continue
                        Continue();
                    }
                    else if (pausing_) { //if robot is currently pausing_, queue unpause cmd
                        queued_cmd_ = QueuedCommand::CONTINUE;
                    }
                }
            }
            else {
                momap::log()->info("Continue command rejected: No matching pause command '{}'", msg->source);
            }
        }
    };

    ///
    /// creates multibody plant and its context
    /// param : mb_plant_ : is member of franka_plan_runner class, multibodyplant for calcinvdyn
    /// param : mb_plant_context : is member of franka_plan_runner class, multibodyplant context
    /// param : urdf_path : urdf to create multibodyplant
    ///
    void MultibodySetUp( drake::multibody::MultibodyPlant<double> &mb_plant_
                        , std::unique_ptr<drake::systems::Context<double>> &mb_plant_context_
                        , const std::string urdf_path);

    ///
    /// callback for inverse dynamics control
    /// TODO :make kp,ki,kd part of the yaml
    /// TO NOTE: when testing, it seemed that joint 0 was much more reactive than all the other joints
    ///
    franka::Torques InverseDynamicsControlCallback(const franka::RobotState& robot_state, franka::Duration period);



    void Pause() {
        momap::log()->info("Pausing plan.");
        paused_ = false;
        pausing_ = true;
        unpausing_ = false;
        this->target_stop_time_ = 0;
        this->timestep_ = 1;
        this->stop_duration_ = 0;
        stop_margin_counter_ = 0;
    }

    void Continue() {
        momap::log()->info("Continuing plan.");
        this->timestep_ = -1 * this->stop_duration_; //how long unpausing_ should take
        momap::log()->debug("STOP DURATION: {}", stop_duration_);
        paused_ = false;
        pausing_ = false;
        unpausing_ = true;
    }
};
} // robot_plan_runner
} // drake

#endif
