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

#include <chrono>
#include <cstdint>  // for int64_t
#include <mutex>    // for mutex
#include <thread>   // for thread

#include <bits/stdint-intn.h>           // for int64_t
#include <cnpy.h>                       // to read joint position offsets
#include <lcmtypes/robot_spline_t.hpp>  // for robot_spline_t

#include "communication_interface.h"  // for CommunicationInterface
#include "constraint_solver.h"        // for ConstraintSolver
#include "drake/common/trajectories/piecewise_polynomial.h"  // for Piecewis...
#include "franka/control_types.h"  // for franka::JointPositions
#include "franka/duration.h"       // for franka::Duration
#include "franka/robot.h"          // for franka::Robot
#include "franka/robot_state.h"    // for franka::RobotState
#include "robot_parameters.h"      // for RobotParameters
#include "util_conv.h"             // for RobotStatus

#define FRANKA_DOF 7

namespace franka_driver {

class FrankaPlanRunner {
 public:
  FrankaPlanRunner(const RobotParameters params);
  ~FrankaPlanRunner() {};

  /// This starts the franka driver
  int Run();

 protected:
  /// Sets collision behavior of robot: at what force threshold
  /// is Franka's Reflex triggered.
  /// Note: Never call this method in the realtime control loop!
  /// Only call this method during initialization. If robot was
  /// already initialized, this method will throw an exception!
  void SetCollisionBehaviorSafetyOn(franka::Robot& robot);
  void SetCollisionBehaviorSafetyOff(franka::Robot& robot);

  franka::RobotMode GetRobotMode(franka::Robot& robot);

  int RunFranka();

  bool RecoverFromControlException(franka::Robot& robot);
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

  franka::JointPositions JointPositionCallback(
      const franka::RobotState& robot_state, franka::Duration period);

 private:
  const int dof_;          // degrees of freedom of franka
  const bool safety_off_;  // torque and force limits to max
  RobotParameters params_;
  std::string ip_addr_;
  const bool is_sim_;

  std::unique_ptr<CommunicationInterface> comm_interface_;
  std::unique_ptr<PPType> plan_;
  int64_t plan_utime_ = -1;
  std::unique_ptr<ConstraintSolver> constraint_solver_;

  std::function<franka::JointPositions(const franka::RobotState&,
                                       franka::Duration)>
      joint_position_callback_;

  // keeping track of time along plan:
  double franka_time_ {};
  // pause related:
  utils::RobotStatus status_;
  long timestep_ = 1;
  float target_stop_time_;
  float stop_duration_;
  float stop_margin_counter_ = 0;

  // last run loopo status update
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

  Eigen::MatrixXd joint_limits_;
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

  // Collision torque thresholds for each joint in [Nm].
  const std::array<double, 7> kHighTorqueThreshold {100.0, 100.0, 100.0, 100.0,
                                                    100.0, 100.0, 100.0};
  const std::array<double, 7> kMediumTorqueThreshold {40.0, 40.0, 36.0, 36.0,
                                                      32.0, 28.0, 24.0};

  // Collision force thresholds for (x, y, z, R, P, Y) in [N].
  const std::array<double, 6> kHighForceThreshold {100.0, 100.0, 100.0,
                                                   100.0, 100.0, 100.0};
  const std::array<double, 6> kMediumForceThreshold {40.0, 40.0, 40.0,
                                                     50.0, 50.0, 50.0};

  std::array<double, 7> upper_torque_threshold_;
  std::array<double, 6> upper_force_threshold_;

};  // FrankaPlanRunner

}  // namespace franka_driver
