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

#include "communication_interface.h"  // for CommunicationInterface
#include "dracula.h"                  // for Dracula
#include "drake/common/trajectories/piecewise_polynomial.h"  // for Piecewis...
#include "franka/control_types.h"  // for franka::JointPositions
#include "franka/duration.h"       // for franka::Duration
#include "franka/robot.h"          // for franka::Robot
#include "franka/robot_state.h"    // for franka::RobotState
#include "franka_driver_utils.h"   //  for dof_ and for RobotState
#include "parameters.h"            // for Parameters

#include <Eigen/Dense>                  // for Eigen::VectorXd
#include <bits/stdint-intn.h>           // for int64_t
#include <cstdint>                      // for int64_t
#include <lcmtypes/robot_spline_t.hpp>  // for robot_spline_t
#include <mutex>                        // for mutex
#include <thread>                       // for thread

namespace franka_driver {

class FrankaPlanRunner {
 public:
  FrankaPlanRunner(const parameters::Parameters params);
  ~FrankaPlanRunner(){};
  
  /// This starts the franka driver
  int Run();

 protected:
  /// Sets collision behaviour of robot: when is Franka's Reflex triggered
  /// Note: Never call this method in the realtime control loop!
  /// Only call this method during initialization.
  /// If robot was already initialized, this method will throw an exception ...
  void SetCollisionBehaviour(franka::Robot& robot, bool we_care_about_safety);
  
  int RunFranka();
  
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

  franka::JointPositions JointPositionCallback(
      const franka::RobotState& robot_state, franka::Duration period);

 private:
  std::unique_ptr<CommunicationInterface> comm_interface_;
  std::unique_ptr<PPType> plan_;
  std::unique_ptr<Dracula> dracula_;
  parameters::Parameters params_;
  std::string ip_addr_;

  // keeping track of time along plan:
  double franka_time_;
  // pause related:
  RobotStatus status_;
  long timestep_ = 1;
  float target_stop_time_;
  float stop_duration_;
  float stop_margin_counter_ = 0;
  int cur_plan_number_ = -1;               // for ensuring the plan is new
  const double lcm_publish_rate_ = 200.0;  // Hz

  Eigen::MatrixXd joint_limits_;
  float stop_delay_factor_ = 2.0;  // this should be yaml param, previously 0.8

  Eigen::VectorXd start_conf_plan_;
  Eigen::VectorXd start_conf_franka_;
  Eigen::VectorXd end_conf_franka_;

  Eigen::VectorXd max_accels_;
  // TODO @rkk: replace allowable_error_ with non arbitrary number
  double allowable_error_ = 0.007;

};  // FrankaPlanRunner

}  // namespace franka_driver
