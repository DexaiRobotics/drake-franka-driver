/// @file franka_driver_utilsils.cc

#include "franka_driver_utils.h"

#include "drake/lcmt_iiwa_status.hpp"  // for lcmt_iiwa_status

#include <bits/types/struct_timeval.h>  // for timeval
#include <cstddef>                      // for NULL
#include <sys/time.h>                   // gettimeofday()

using namespace franka_driver;

// TODO: @dmsj - make this call ConvertToLcmStatus()
static void franka_driver::AssignToLcmStatus(
    franka::RobotState& robot_state, drake::lcmt_iiwa_status& robot_status) {
  int num_joints = robot_state.q.size();
  struct timeval tv;
  gettimeofday(&tv, NULL);
  // TODO: @dmsj store both timeofday and franka_time_
  // int64_t(1000.0 * robot_state.time.toMSec()):
  robot_status.utime = int64_t(tv.tv_sec * 1e6 + tv.tv_usec);
  robot_status.num_joints = num_joints;
  // q:
  robot_status.joint_position_measured.assign(std::begin(robot_state.q),
                                              std::end(robot_state.q));
  // q_d:
  robot_status.joint_position_commanded.assign(std::begin(robot_state.q_d),
                                               std::end(robot_state.q_d));
  robot_status.joint_position_ipo.resize(num_joints, 0);
  // dq:
  robot_status.joint_velocity_estimated.assign(std::begin(robot_state.dq),
                                               std::end(robot_state.dq));
  // tau_J:
  robot_status.joint_torque_measured.assign(std::begin(robot_state.tau_J),
                                            std::end(robot_state.tau_J));
  // tau_J_d:
  robot_status.joint_torque_commanded.assign(std::begin(robot_state.tau_J_d),
                                             std::end(robot_state.tau_J_d));
  robot_status.joint_torque_external.resize(num_joints, 0);
}

drake::lcmt_iiwa_status franka_driver::ConvertToLcmStatus(
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

void franka_driver::ResizeStatusMessage(drake::lcmt_iiwa_status& lcm_status) {
  lcm_status.utime = -1;
  lcm_status.num_joints = dof_;
  lcm_status.joint_position_measured.resize(dof_, 0);
  lcm_status.joint_position_commanded.resize(dof_, 0);
  lcm_status.joint_position_ipo.resize(dof_, 0);
  lcm_status.joint_velocity_estimated.resize(dof_, 0);
  lcm_status.joint_torque_measured.resize(dof_, 0);
  lcm_status.joint_torque_commanded.resize(dof_, 0);
  lcm_status.joint_torque_external.resize(dof_, 0);
}

std::string franka_driver::RobotModeToString(franka::RobotMode mode) {
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

std::string franka_driver::RobotStatusToString(RobotStatus status) {
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
  }
  return status_string;
}

std::array<double, 7> franka_driver::EigenToArray(
    const Eigen::VectorXd& input) {
  std::array<double, 7> output = {
      {input[0], input[1], input[2], input[3], input[4], input[5], input[6]}};
  return output;
}
