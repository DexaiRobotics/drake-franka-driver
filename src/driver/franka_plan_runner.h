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

/// @file franka_plan_runner
///
/// franka_plan_runner is designed to get a plan from the comm interface,
/// and then execute the plan on a franka arm
///
/// When a new plan is received from the comm interface,
/// it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.
#pragma once

#include <Eigen/Dense>  // for Eigen::VectorXd
#include <cnpy.h>       // to read joint position offsets
#include <drake/common/trajectories/piecewise_polynomial.h>  // for Piecewis...

#include <chrono>
#include <deque>   // for deque<>
#include <memory>  // for unique ptr
#include <mutex>   // for mutex
#include <string>
#include <thread>  // for thread

#include <robot_msgs/plan_exec_opts_t.hpp>
#include <robot_msgs/robot_spline_t.hpp>  // for robot_spline_t

#include "driver/communication_interface.h"  // for CommunicationInterface
#include "driver/constraint_solver.h"        // for ConstraintSolver
#include "franka/control_types.h"            // for franka::JointPositions
#include "franka/duration.h"                 // for franka::Duration
#include "franka/model.h"                    // for Model
#include "franka/robot.h"                    // for franka::Robot
#include "franka/robot_state.h"              // for franka::RobotState
#include "utils/robot_parameters.h"          // for RobotParameters
#include "utils/util_conv.h"                 // for RobotStatus

#define FRANKA_DOF 7

namespace franka_driver {

class FrankaPlanRunner {
 public:
  explicit FrankaPlanRunner(const RobotParameters& params);

  /// This starts the franka driver
  int Run();

 protected:
  // used only during robot init
  // collision behaviour is set by SetCollisionBehaviorSafetyOn
  // and SetCollisionBehaviorSafetyOff
  void SetDefaultBehaviorForInit() {
    if (is_sim_) {
      return;
    }

    robot_->setCollisionBehavior(
        // lower_torque_thresholds_acceleration
        {20, 20, 20, 20, 10, 10, 10},
        // upper_torque_thresholds_acceleration
        {87, 87, 87, 87, 12, 12, 12},
        // lower_torque_thresholds_nominal
        {10, 10, 10, 10, 10, 10, 10},
        // upper_torque_thresholds_nominal
        {87, 87, 87, 87, 12, 12, 12},
        // lower_force_thresholds_acceleration
        {20, 20, 20, 20, 20, 20},
        // upper_force_thresholds_acceleration
        {99, 99, 99, 99, 99, 99},
        // lower_force_thresholds_nominal
        {10, 10, 10, 10, 10, 10},
        // upper_force_thresholds_nominal
        {99, 99, 99, 99, 99, 99});
    robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
  }

  /// Sets collision behavior of robot: at what force threshold
  /// is Franka's Reflex triggered.
  /// Note: Never call this method in the realtime control loop!
  /// Only call this method during initialization. If robot was
  /// already initialized, this method will throw an exception!

  // Changes the collision behavior. Set separate torque and force boundaries
  // for acceleration/deceleration and constant velocity movement phases.

  // Forces or torques between lower and upper threshold are shown as contacts
  // in the RobotState. Forces or torques above the upper threshold are
  // registered as collision and cause the robot to stop moving.

  // TODO(@syler): can we just set the lower threshold to something reasonable
  // and get away with only increasing the upper threshold?
  void SetCollisionBehaviorSafetyOn() {
    if (is_sim_) {
      return;
    }

    if (const auto mode {GetRobotMode()}; mode == franka::RobotMode::kMove) {
      throw std::runtime_error("robot is in mode: "
                               + utils::RobotModeToString(mode)
                               + " cannot change collision behavior!");
    }
    // Params in order:
    // lower_torque_thresholds_acceleration,
    // upper_torque_thresholds_acceleration,
    // lower_torque_thresholds_nominal,
    // upper_torque_thresholds_nominal,
    // lower_force_thresholds_acceleration,
    // upper_force_thresholds_acceleration,
    // lower_force_thresholds_nominal,
    // upper_force_thresholds_nominal
    robot_->setCollisionBehavior(
        lower_torque_threshold_, upper_torque_threshold_,
        lower_torque_threshold_, upper_torque_threshold_,
        lower_force_threshold_, upper_force_threshold_, lower_force_threshold_,
        upper_force_threshold_);
  }

  void SetCollisionBehaviorSafetyOff() {
    if (is_sim_) {
      return;
    }

    if (const auto mode {GetRobotMode()}; mode == franka::RobotMode::kMove) {
      throw std::runtime_error("robot is in mode: "
                               + utils::RobotModeToString(mode)
                               + " cannot change collision behavior!");
    }
    // Forces or torques above the upper threshold
    // are registered as collision and cause the robot to stop moving.
    robot_->setCollisionBehavior(kHighTorqueThreshold, kHighTorqueThreshold,
                                 kHighTorqueThreshold, kHighTorqueThreshold,
                                 kHighForceThreshold, kHighForceThreshold,
                                 kHighForceThreshold, kHighForceThreshold);
  }

  franka::RobotMode GetRobotMode() const {
    franka::RobotMode robot_mode {robot_->readOnce().robot_mode};
    dexai::log()->debug("GetRobotMode: Franka is in mode: {}",
                        utils::RobotModeToString(robot_mode));
    return robot_mode;
  }

  int RunFranka();

  bool RecoverFromControlException();

  int RunSim();

  /// Check and limit conf according to provided parameters for joint limits
  bool LimitJoints(Eigen::VectorXd& conf);

  /// Calculate the time to advance while pausing or unpausing
  /// Inputs to method have seconds as their unit.
  /// Algorithm: Uses a logistic growth function:
  /// t' = f - 4 / [a (e^{a*t} + 1] where
  /// f = target_stop_time, t' = franka_time, t = real_time
  /// Returns delta t', the period that should be incremented to franka time
  double TimeToAdvanceWhilePausing(double period, double target_stop_time,
                                   double timestep);

  /// The franka time advances according to the pause status of the robot.
  /// The franka time is used to read out a value from a piecewise polynomial.
  /// If the robot is pausing or unpausing, then the franka time advances slower
  //  If the robot is paused, then the franka time is not advancing at all.
  void IncreaseFrankaTimeBasedOnStatus(const std::array<double, 7>& vel,
                                       double period_in_seconds);

  // When a robot state is received, the callback function is used to calculate
  // the response: the desired values for that time step. After sending back the
  // response, the callback function will be called again with the most recently
  // received robot state. Since the robot is controlled with a 1 kHz frequency,
  // the callback functions have to compute their result in a short time frame
  // in order to be accepted.

  /**
   * @brief Generates joint positions for the robot based on the current robot
   * state and the active plan.
   *
   * This callback function is called at each control loop iteration to
   * determine the next set of joint positions for the Franka robot. It handles
   * simulation control exceptions, updates internal timing based on pause
   * status, and manages plan transitions. If a new plan is available and valid,
   * it updates the active plan. The function also checks for plan completion
   * and handles plan execution options, such as stopping at the expected
   * contact point.
   *
   * When no active plan is present, or upon plan completion, it returns the
   * current joint positions to finish the motion.
   *
   * @param robot_state The current state of the robot, including joint
   * positions and velocities
   * @param period Time since last callback invocation, zero on first
   * @return A `franka::JointPositions` object indicating the next set of
   * desired joint positions for the robot
   */
  franka::JointPositions JointPositionCallback(
      const franka::RobotState& robot_state, franka::Duration period);

  /**
   * @brief Helper function to determine if the start of new plan is far from
   * the current robot conf.
   *
   * @param params - parameters from which to get comparison thresholds
   * @param franka_start_conf - current robot conf from which the robot will
   * start executing plan
   * @param start_conf_plan - start conf of the new plan
   * @return true, if the maximum angular distance between current robot conf
   * and start of new plan is greater than kMediumJointDistance
   * @return false, otherwise
   */
  static bool IsStartFarFromCurrentJointPosition(
      const RobotParameters& params, const Eigen::VectorXd& franka_start_conf,
      const Eigen::VectorXd& start_conf_plan);

  /**
   * @brief Helper function to copy robot plan, utime associated with the plan,
   * execution options associated with the plan, and expected contact vector
   * associated with robot plan from the arguments. Also sets sets utime at
   * start of plan execution, plan start conf, and plan end conf based on the
   * new plan. Resets franka time if not continuing onto new plan from
   * current plan.
   *
   * @param new_plan - unique_ptr to new plan
   * @param new_plan_utime - utime corresponding to the new plan
   * @param new_plan_exec_opt - execution options associated with the new plan
   * @param new_plan_contact_expected - unit vector in the direction of expected
   * contact assiciated with the new plan. if any
   * @param new_plan_timepoints - struct containing times when plan was received
   * and accepted
   */
  void UpdateActivePlan(std::unique_ptr<PPType> new_plan,
                        int64_t new_plan_utime, int64_t new_plan_exec_opt,
                        const Eigen::Vector3d& new_plan_contact_expected,
                        const PlanTimepoints& new_plan_timepoints);

  /// Set parameters for stiffness and goal direction based on push direction.
  /// TODO(@anyone): long-term the stiffness can be a parameter of the push
  /// request
  /// @deprecated
  void SetCompliantPushParameters(
      const franka::RobotState& initial_state,
      const Eigen::Vector3d& desired_ee_translation,
      const Eigen::Vector3d& translational_stiffness,
      const Eigen::Vector3d& rotational_stiffness);

  /// @deprecated
  void SetCompliantPushParameters(
      const franka::RobotState& initial_state,
      const Eigen::Vector3d& desired_ee_translation) {
    return SetCompliantPushParameters(initial_state, desired_ee_translation,
                                      kDefaultTranslationalStiffness,
                                      kDefaultRotationalStiffness);
  }

  /// @deprecated
  franka::Torques ImpedanceControlCallback(
      const franka::RobotState& robot_state, franka::Duration);

  Eigen::Matrix<double, 7, 7> NullSpace(
      const Eigen::Matrix<double, 6, 7> jacobian,
      const Eigen::Matrix<double, 7, 7> inertia);

  inline void ResetPlan() {
    dexai::log()->debug("ResetPlan: begin");
    plan_.reset();
    plan_utime_ = -1;
    plan_start_utime_ = -1;
    franka_time_ = 0;
  }

  /**
   * @brief Checks if `plan` is continuous in position, velocity, and
   * acceleration with the current robot plan at the current franka time
   *
   * @param plan - unique_ptr pointing to a new plan which will be checked
   * against the current plan
   * @return true, if the new plan is continuous in position, velocity, and
   * acceleration with the current plan at the current franka time
   * @return false, otherwise
   */
  bool IsContinuousWithCurrentPlan(const std::unique_ptr<PPType>& plan);

 private:
  const int dof_;          // degrees of freedom of franka
  const bool safety_off_;  // torque and force limits to max
  RobotParameters params_;
  std::string ip_addr_;
  const bool is_sim_;

  std::unique_ptr<franka::Robot> robot_ {};
  std::unique_ptr<CommunicationInterface> comm_interface_;
  std::unique_ptr<PPType> plan_;
  int64_t plan_utime_ {-1}, plan_start_utime_ {-1};
  int64_t plan_exec_opt_ {robot_msgs::plan_exec_opts_t::DEFAULT};
  Eigen::Vector3d contact_expected_ {Eigen::Vector3d::Zero()};
  std::unique_ptr<ConstraintSolver> constraint_solver_;

  // keeping track of time along plan:
  double franka_time_ {};
  // pause related:
  utils::RobotStatus status_;
  int64_t timestep_ {1};
  float target_stop_time_ {};
  float stop_duration_ {};
  float stop_margin_counter_ {};

  // last run loop status update
  std::chrono::time_point<std::chrono::steady_clock> t_last_main_loop_log_ {};

  // We control the robot at 1 kHz using the callback function
  // which gets called by the robot at 1 kHz over direct eithernet.
  // But we publish robot status via LCM over another ethernet connection
  // at 200 Hz which is frequent enough.
  // One considertion is that typically robots have resonance ~20 Hz
  // in the mechanical structure.
  // 10x the mechanical closed loop response frequency is a rule of thumb.
  // This way we don't alias in other frequencies, and are able to synthesize
  // frequency components in the region we care about with high fidelity
  const double lcm_publish_rate_ {200.0};  // Hz

  /// upper and lower limit for each joint
  Eigen::MatrixXd joint_limits_;

  /// midpoints between upper and lower joint limit
  Eigen::Matrix<double, 7, 1> q_center_;
  /// half range between upper and lower limit
  Eigen::Matrix<double, 7, 1> q_half_range_;

  float stop_delay_factor_ = 2.0;  // this should be yaml param, previously 0.8

  // config of start of plan:
  Eigen::VectorXd start_conf_plan_;
  // next config according to plan:
  Eigen::VectorXd next_conf_plan_;
  // config of franka when plan starts:
  Eigen::VectorXd start_conf_franka_;
  // config of robot when franka starts reversing:
  Eigen::VectorXd start_reversing_conf_franka_;
  // config of franka when plan ends:
  Eigen::VectorXd end_conf_plan_;

  bool is_joint_pos_offset_available_ {false};
  Eigen::VectorXd joint_pos_offset_;

  Eigen::VectorXd max_accels_;

  // empirically proven, increased to work w/ sim robot
  static constexpr double allowable_max_angle_error_ {0.001};

  // Collision torque thresholds for each joint in [Nm].
  const std::array<double, 7> kHighTorqueThreshold {100.0, 100.0, 100.0, 100.0,
                                                    100.0, 100.0, 100.0};
  const std::array<double, 7> kMediumTorqueThreshold {40.0, 40.0, 36.0, 36.0,
                                                      32.0, 28.0, 24.0};
  const std::array<double, 7> kLowTorqueThreshold {20.0, 20.0, 18.0, 18.0,
                                                   16.0, 14.0, 12.0};
  const std::array<double, 7> kImpedanceControlTorqueThreshold {
      87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

  // Collision force thresholds for (x, y, z, R, P, Y) in [N].
  const std::array<double, 6> kHighForceThreshold {100.0, 100.0, 100.0,
                                                   100.0, 100.0, 100.0};
  const std::array<double, 6> kMediumForceThreshold {40.0, 40.0, 40.0,
                                                     50.0, 50.0, 50.0};
  const std::array<double, 6> kLowForceThreshold {20.0, 20.0, 20.0,
                                                  25.0, 25.0, 25.0};

  std::array<double, 7> lower_torque_threshold_, upper_torque_threshold_;
  std::array<double, 6> lower_force_threshold_, upper_force_threshold_;

  // The following two constants must be tuned together.
  // A higher speed threshold may result in the benign libfranka exception:
  //    Motion finished commanded, but the robot is still moving!
  //    ["joint_motion_generator_acceleration_discontinuity"]
  const double CONV_ANGLE_THRESHOLD {1e-3};         // rad, empirical
  const double CONV_SPEED_NORM_THRESHOLD {6.6e-3};  // rad/s, L2
  Eigen::VectorXd CONV_SPEED_THRESHOLD;

  // Compliance parameters
  const Eigen::Vector3d kDefaultTranslationalStiffness {100.0, 100.0, 100.0};
  const Eigen::Vector3d kDefaultRotationalStiffness {10.0, 10.0, 50.0};

  // stiffness and damping for compliant push
  Eigen::Matrix<double, 6, 6> stiffness_, damping_;

  // joint centering gain for spring damper system
  const double k_centering_ {1.0};
  // filter gain to control joint centering gain growth over time
  const double filter_gain_ {0.001};
  // actual gain for joint centering, controls how much torque is applied in an
  // attempt to center joints - starts at zero and ramps up over time
  double k_jc_ramp_ {0.0};

  // store the length of time it took to complete the last callbacks to debug
  std::deque<size_t> time_elapsed_us_;

  std::unique_ptr<franka::Model> model_ {};

  Eigen::Vector3d desired_position_;
  Eigen::Quaterniond desired_orientation_;

  // number of attempts to automatically recover from being in reflex mode on
  // init
  size_t reflex_init_recovery_attempts_ {};
};  // FrankaPlanRunner

}  // namespace franka_driver
