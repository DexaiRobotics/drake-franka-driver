/// @file franka_driver_utils.cc

#include "franka_driver_utils.h"

#include "drake/lcmt_iiwa_status.hpp"  // for lcmt_iiwa_status

#include <bits/types/struct_timeval.h>  // for timeval
#include <cstddef>                      // for NULL
#include <sys/time.h>                   // for gettimeofday()

namespace utils {

drake::lcmt_iiwa_status ConvertToLcmStatus(
    franka::RobotState& robot_state) {
  drake::lcmt_iiwa_status robot_status{};
  int num_joints = robot_state.q.size();
  struct timeval tv;
  gettimeofday(&tv, NULL);

  // int64_t(1000.0 * robot_state.time.toMSec()) :
  robot_status.utime = int64_t(tv.tv_sec * 1e6 + tv.tv_usec);
  robot_status.num_joints = num_joints;
  // q
  robot_status.joint_position_measured = ArrayToVector(robot_state.q);
  robot_status.joint_position_commanded = ArrayToVector(robot_state.q_d);
  robot_status.joint_position_ipo.resize(num_joints, 0);
  robot_status.joint_velocity_estimated = ArrayToVector(robot_state.dq);
  robot_status.joint_torque_measured = ArrayToVector(robot_state.tau_J);
  robot_status.joint_torque_commanded = ArrayToVector(robot_state.tau_J_d);
  robot_status.joint_torque_external.resize(num_joints, 0);

  return robot_status;
}

std::string RobotModeToString(franka::RobotMode mode) {
  std::string mode_string;
  switch (mode) {
    case franka::RobotMode::kOther:
      mode_string = "Other";
      break;
    case franka::RobotMode::kIdle:
      mode_string = "Idle";
      break;
    case franka::RobotMode::kMove:
      mode_string = "Move";
      break;
    case franka::RobotMode::kGuiding:
      mode_string = "Guiding";
      break;
    case franka::RobotMode::kReflex:
      mode_string = "Reflex";
      break;
    case franka::RobotMode::kUserStopped:
      mode_string = "User Stopped";
      break;
    case franka::RobotMode::kAutomaticErrorRecovery:
      mode_string = "Automatic Error Recovery";
      break;
  }
  return mode_string;
}

std::string RobotStatusToString(RobotStatus status) {
  std::string status_string;
  switch (status) {
    case RobotStatus::Uninitialized:
      status_string = "Uninitialized";
      break;
    case RobotStatus::Running:
      status_string = "Running";
      break;
    case RobotStatus::Pausing:
      status_string = "Pausing";
      break;
    case RobotStatus::Paused:
      status_string = "Paused";
      break;
    case RobotStatus::Unpausing:
      status_string = "Unpausing";
      break;
    case RobotStatus::Reversing:
      status_string = "Reversing";
      break;
  }
  return status_string;
}

std::array<double, 7> EigenToArray(
    const Eigen::VectorXd& input) {
  std::array<double, 7> output = {
      {input[0], input[1], input[2], input[3], input[4], input[5], input[6]}};
  return output;
}

}  // namespace utils
