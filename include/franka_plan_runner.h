/// @file franka_plan_runner
///
/// franka_plan_runner is designed to wait for LCM messages containing
/// a robot_plan_t message, and then execute the plan on a franka arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.
#pragma once

#include <bits/stdint-intn.h>                                // for int64_t
#include <drake/common/trajectories/piecewise_polynomial.h>  // for Piecewis...
#include <franka/control_types.h>  // for franka::JointPositions
#include <franka/duration.h>       // for franka::Duration
#include <franka/robot_state.h>    // for franka::RobotState
#include <franka/robot.h>          // for franka::Robot

#include <cstdint>                      // for int64_t
#include <lcmtypes/robot_spline_t.hpp>  // for robot_spline_t
#include <mutex>                        // for mutex
#include <thread>                       // for thread
#include <Eigen/Dense>                  // for Eigen::VectorXd

#include "communication_interface.h"  // for CommunicationInterface
#include "dracula.h"                  // for Dracula
#include "parameters.h"               // for Parameters


namespace franka_driver {

enum class RobotStatus { Uninitialized, Running, Pausing, Paused, Unpausing };

class FrankaPlanRunner {
 public:
  FrankaPlanRunner(const parameters::Parameters params);
  ~FrankaPlanRunner(){};
  int Run();

 protected:
  void SetCollisionBehaviour(franka::Robot& robot, bool we_care_about_safety);
  int RunFranka();
  int RunSim();

  bool LimitJoints(Eigen::VectorXd& conf);

  double StopPeriod(double period, double target_stop_time,
                                      double timestep);

  void IncreaseFrankaTimeBasedOnStatus(const std::array<double, 7>& vel, double period_in_seconds);
  
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
  int cur_plan_number_ = -1;           // for ensuring the plan is new
  const double lcm_publish_rate_ = 200.0;  // Hz

  Eigen::MatrixXd joint_limits_;
  float STOP_SCALE = 0.5;  // this should be yaml param, previously 0.8

  Eigen::VectorXd start_conf_plan_;
  Eigen::VectorXd start_conf_franka_;
  Eigen::VectorXd end_conf_franka_;

  Eigen::VectorXd max_accels_;
  // TODO @rkk: replace allowable_error_ with non arbitrary number
  double allowable_error_ = 0.007;

};  // FrankaPlanRunner

}  // namespace franka_driver
