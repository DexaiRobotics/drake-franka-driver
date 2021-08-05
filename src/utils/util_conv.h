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

/// @file util_conv.h
#pragma once

#include <Eigen/Geometry>

#include <string>

#include <drake/lcmt_iiwa_status.hpp>  // for lcmt_iiwa_status

#include "driver/communication_interface.h"
#include "franka/robot_state.h"  // for RobotState, RobotMode

namespace utils {

// Enums for Defining Status
enum class RobotStatus {
  Uninitialized,
  Running,
  Pausing,
  Paused,
  Unpausing,
  Reversing,
};

enum PauseCommandType { CONTINUE = 0, PAUSE = 1, CANCEL_PLAN = 2 };

// Status Conversions:

std::string RobotModeToString(franka::RobotMode mode);

std::string RobotStatusToString(RobotStatus status);

drake::lcmt_iiwa_status ConvertToLcmIiwaStatus(
    const franka_driver::RobotData& robot_data);

robot_msgs::robot_status_t ConvertToRobotStatusLcmMsg(
    const franka_driver::RobotData& robot_data);

drake::lcmt_iiwa_status EigenToLcmStatus(Eigen::VectorXd robot_state);

franka::RobotState ConvertToCannonical(const franka::RobotState& robot_state,
                                       const Eigen::VectorXd& offsets);

/**
 * @brief Utility function that takes converts a pose
 * from `robot_msgs::pose_t` to `drake::math::RigidTransformd`
 * which is more convenient for calculations
 *
 * @param pos3dt pose as `robot_msgs::pose_t`
 * @return pose as `drake::math::RigidTransformd`
 */
inline drake::math::RigidTransformd ToRigidTransform(
    const robot_msgs::pose_t& pos3dt) {
  return {Eigen::Quaterniond(pos3dt.rotation.w, pos3dt.rotation.x,
                             pos3dt.rotation.y, pos3dt.rotation.z),
          Eigen::Vector3d(pos3dt.translation.x, pos3dt.translation.y,
                          pos3dt.translation.z)};
}

/**
 * @brief Utility function that takes converts a pose
 * from `Eigen::Affine3d` to `drake::math::RigidTransformd`
 * which is more convenient for calculations and is widely
 * used in our code base
 *
 * @param xform_eigen pose as `Eigen::Affine3d`
 * @return pose as `drake::math::RigidTransformd`
 */
inline drake::math::RigidTransformd ToRigidTransform(
    const Eigen::Affine3d& xform_eigen) {
  return drake::math::RigidTransformd(xform_eigen.matrix());
}

/**
 * @brief Utility function that takes converts a pose
 * from `drake::math::RigidTransformd` to `Eigen::Affine3d`
 * which can be mapped to an std::array for efficient
 *
 * @param xform pose as `drake::math::RigidTransformd`
 * @return pose as `Eigen::Affine3d`
 */
inline Eigen::Affine3d ToAffine3d(const drake::math::RigidTransformd& xform) {
  return Eigen::Affine3d(xform.GetAsMatrix4());
}

}  //  namespace utils
