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

drake::lcmt_iiwa_status ConvertToLcmIiwaStatus(
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

  // NOTE: We use the currently unused external joint torques array to send
  // cartesian contact info for testing purposes. This is a temporary workaround
  // until we update the iiwa_status lcm message include these fields
  // TODO(@anyone): update iiwa_status lcm message and get rid of these
  // workarounds
  robot_status.joint_torque_external =
      ArrayToVector(robot_state.cartesian_contact);
  robot_status.joint_torque_external.resize(num_joints, 0);

  return robot_status;
}

robot_msgs::robot_status_t ConvertToRobotStatusLcmMsg(
    const franka_driver::RobotData& robot_data) {
  const auto& robot_status {robot_data.robot_state};

  robot_msgs::robot_status_t status_msg {};
  auto num_joints {robot_status.q.size()};
  struct timeval tv {};
  gettimeofday(&tv, NULL);

  // int64_t(1000.0 * robot_state.time.toMSec()) :
  status_msg.utime = int64_t(tv.tv_sec * 1e6 + tv.tv_usec);
  status_msg.num_joints = static_cast<int>(num_joints);

  // vectors
  status_msg.tau_J.resize(num_joints, 0);
  status_msg.tau_J_d.resize(num_joints, 0);
  status_msg.dtau_J.resize(num_joints, 0);
  status_msg.q.resize(num_joints, 0);
  status_msg.q_d.resize(num_joints, 0);
  status_msg.dq.resize(num_joints, 0);
  status_msg.dq_d.resize(num_joints, 0);
  status_msg.ddq_d.resize(num_joints, 0);
  status_msg.joint_contact.resize(num_joints, 0);
  status_msg.joint_collision.resize(num_joints, 0);
  status_msg.tau_ext_hat_filtered.resize(num_joints, 0);
  status_msg.theta.resize(num_joints, 0);
  status_msg.dtheta.resize(num_joints, 0);

  std::copy(robot_status.O_T_EE.begin(), robot_status.O_T_EE.end(),
            std::begin(status_msg.O_T_EE));

  std::copy(robot_status.O_T_EE_d.begin(), robot_status.O_T_EE_d.end(),
            std::begin(status_msg.O_T_EE_d));
  std::copy(robot_status.F_T_EE.begin(), robot_status.F_T_EE.end(),
            std::begin(status_msg.F_T_EE));
  std::copy(robot_status.F_T_NE.begin(), robot_status.F_T_NE.end(),
            std::begin(status_msg.F_T_NE));
  std::copy(robot_status.NE_T_EE.begin(), robot_status.NE_T_EE.end(),
            std::begin(status_msg.NE_T_EE));
  std::copy(robot_status.EE_T_K.begin(), robot_status.EE_T_K.end(),
            std::begin(status_msg.EE_T_K));
  status_msg.m_ee = robot_status.m_ee;
  std::copy(robot_status.I_ee.begin(), robot_status.I_ee.end(),
            std::begin(status_msg.I_ee));
  std::copy(robot_status.F_x_Cee.begin(), robot_status.F_x_Cee.end(),
            std::begin(status_msg.F_x_Cee));
  status_msg.m_load = robot_status.m_load;
  std::copy(robot_status.I_load.begin(), robot_status.I_load.end(),
            std::begin(status_msg.I_load));
  std::copy(robot_status.F_x_Cload.begin(), robot_status.F_x_Cload.end(),
            std::begin(status_msg.F_x_Cload));
  status_msg.m_total = robot_status.m_total;
  std::copy(robot_status.I_total.begin(), robot_status.I_total.end(),
            std::begin(status_msg.I_total));
  std::copy(robot_status.F_x_Ctotal.begin(), robot_status.F_x_Ctotal.end(),
            std::begin(status_msg.F_x_Ctotal));
  std::copy(robot_status.elbow.begin(), robot_status.elbow.end(),
            std::begin(status_msg.elbow));
  std::copy(robot_status.elbow_d.begin(), robot_status.elbow_d.end(),
            std::begin(status_msg.elbow_d));
  std::copy(robot_status.elbow_c.begin(), robot_status.elbow_c.end(),
            std::begin(status_msg.elbow_c));
  std::copy(robot_status.delbow_c.begin(), robot_status.delbow_c.end(),
            std::begin(status_msg.delbow_c));
  std::copy(robot_status.ddelbow_c.begin(), robot_status.ddelbow_c.end(),
            std::begin(status_msg.ddelbow_c));
  std::copy(robot_status.tau_J.begin(), robot_status.tau_J.end(),
            std::begin(status_msg.tau_J));
  std::copy(robot_status.tau_J_d.begin(), robot_status.tau_J_d.end(),
            std::begin(status_msg.tau_J_d));
  std::copy(robot_status.dtau_J.begin(), robot_status.dtau_J.end(),
            std::begin(status_msg.dtau_J));
  std::copy(robot_status.q.begin(), robot_status.q.end(),
            std::begin(status_msg.q));
  std::copy(robot_status.q_d.begin(), robot_status.q_d.end(),
            std::begin(status_msg.q_d));
  std::copy(robot_status.dq.begin(), robot_status.dq.end(),
            std::begin(status_msg.dq));
  std::copy(robot_status.dq_d.begin(), robot_status.dq_d.end(),
            std::begin(status_msg.dq_d));
  std::copy(robot_status.ddq_d.begin(), robot_status.ddq_d.end(),
            std::begin(status_msg.ddq_d));
  std::copy(robot_status.joint_contact.begin(),
            robot_status.joint_contact.end(),
            std::begin(status_msg.joint_contact));
  std::copy(robot_status.cartesian_contact.begin(),
            robot_status.cartesian_contact.end(),
            std::begin(status_msg.cartesian_contact));
  std::copy(robot_status.joint_collision.begin(),
            robot_status.joint_collision.end(),
            std::begin(status_msg.joint_collision));
  std::copy(robot_status.cartesian_collision.begin(),
            robot_status.cartesian_collision.end(),
            std::begin(status_msg.cartesian_collision));
  std::copy(robot_status.tau_ext_hat_filtered.begin(),
            robot_status.tau_ext_hat_filtered.end(),
            std::begin(status_msg.tau_ext_hat_filtered));
  std::copy(robot_status.O_F_ext_hat_K.begin(),
            robot_status.O_F_ext_hat_K.end(),
            std::begin(status_msg.O_F_ext_hat_K));
  std::copy(robot_status.K_F_ext_hat_K.begin(),
            robot_status.K_F_ext_hat_K.end(),
            std::begin(status_msg.K_F_ext_hat_K));
  std::copy(robot_status.O_dP_EE_d.begin(), robot_status.O_dP_EE_d.end(),
            std::begin(status_msg.O_dP_EE_d));
  std::copy(robot_status.O_T_EE_c.begin(), robot_status.O_T_EE_c.end(),
            std::begin(status_msg.O_T_EE_c));
  std::copy(robot_status.O_dP_EE_c.begin(), robot_status.O_dP_EE_c.end(),
            std::begin(status_msg.O_dP_EE_c));
  std::copy(robot_status.O_ddP_EE_c.begin(), robot_status.O_ddP_EE_c.end(),
            std::begin(status_msg.O_ddP_EE_c));
  std::copy(robot_status.theta.begin(), robot_status.theta.end(),
            std::begin(status_msg.theta));
  std::copy(robot_status.dtheta.begin(), robot_status.dtheta.end(),
            std::begin(status_msg.dtheta));

  status_msg.control_command_success_rate =
      status_msg.control_command_success_rate;

  status_msg.robot_mode = static_cast<int16_t>(robot_status.robot_mode);

  status_msg.current_plan_utime = robot_data.current_plan_utime;
  status_msg.plan_start_utime = robot_data.plan_start_utime;
  status_msg.plan_exec_frac = robot_data.plan_completion_frac;

  return status_msg;
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
