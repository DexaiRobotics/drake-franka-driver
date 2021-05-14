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

///@file: util_conv.cc
#include "utils/util_conv.h"

#include <sys/time.h>  // for gettimeofday()

#include "utils/dexai_log.h"
#include "utils/util_math.h"

namespace utils {

using dexai::log;

// Convenience Functions for converting between lcm, STL, and Eigen types

std::string RobotModeToString(franka::RobotMode mode) {
  std::string mode_string;
  switch (mode) {
    case franka::RobotMode::kOther:
      mode_string = "Locked";
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

drake::lcmt_iiwa_status ConvertToLcmStatus(
    const franka_driver::RobotData&
        robot_data) {  // franka::RobotState& robot_state) {
  const auto& robot_state = robot_data.robot_state;
  const auto& robot_plan_next_conf = robot_data.robot_plan_next_conf;

  drake::lcmt_iiwa_status robot_status {};
  int num_joints = robot_state.q.size();
  struct timeval tv;
  gettimeofday(&tv, NULL);

  // int64_t(1000.0 * robot_state.time.toMSec()) :
  robot_status.utime = int64_t(tv.tv_sec * 1e6 + tv.tv_usec);
  robot_status.num_joints = num_joints;
  // q
  robot_status.joint_position_measured = ArrayToVector(robot_state.q);
  robot_status.joint_position_commanded = ArrayToVector(robot_state.q_d);

  // NOTE: joint_position_ipo is supposed to be KUKA's motion interpolated joint
  // position but we use to pass via lcm the joint position the robot is
  // supposed to be at according to its current plan
  robot_status.joint_position_ipo = e_to_v(robot_plan_next_conf);

  robot_status.joint_velocity_estimated = ArrayToVector(robot_state.dq);


  robot_status.joint_torque_measured = ArrayToVector(robot_state.tau_J);
  robot_status.joint_torque_commanded = ArrayToVector(robot_state.tau_J_d);

  // NOTE: We use the currently unused external joint torques array to send cartesian
  // contact info for testing purposes. This is a temporary workaround until we update
  // the iiwa_status lcm message include these fields
  robot_status.joint_torque_external = ArrayToVector(robot_state.cartesian_contact);
  robot_status.joint_torque_external.resize(num_joints, 0);

  return robot_status;
}

drake::lcmt_iiwa_status EigenToLcmStatus(Eigen::VectorXd robot_state) {
  drake::lcmt_iiwa_status robot_status {};
  int num_joints = robot_state.size();
  struct timeval tv;
  gettimeofday(&tv, NULL);

  robot_status.utime =
      int64_t(tv.tv_sec * 1e6
              + tv.tv_usec);  // int64_t(1000.0 * robot_state.time.toMSec());
  // q
  robot_status.num_joints = num_joints;
  robot_status.joint_position_measured = e_to_v(robot_state);
  robot_status.joint_position_commanded.resize(num_joints, 0);
  robot_status.joint_position_ipo.resize(num_joints, 0);
  robot_status.joint_velocity_estimated.resize(num_joints, 0);
  robot_status.joint_torque_measured.resize(num_joints, 0);
  robot_status.joint_torque_commanded.resize(num_joints, 0);
  robot_status.joint_torque_external.resize(num_joints, 0);

  return robot_status;
}

franka::RobotState ConvertToCannonical(const franka::RobotState& robot_state,
                                       const Eigen::VectorXd& offsets) {
  if (robot_state.q.size() != static_cast<size_t>(offsets.size())) {
    std::string err_msg = fmt::format(
        "utils:ConvertToCannonical robot_state.q.size({}) != offsets.size({})",
        robot_state.q.size(), offsets.size());
    dexai::log()->error(err_msg);
    throw std::runtime_error(err_msg);
  }
  franka::RobotState cannonical_robot_state = robot_state;
  for (size_t i {}; i < static_cast<size_t>(offsets.size()); i++) {
    cannonical_robot_state.q[i] += offsets[i];    // actual
    cannonical_robot_state.q_d[i] += offsets[i];  // desired
  }
  return cannonical_robot_state;
}

}  // namespace utils
