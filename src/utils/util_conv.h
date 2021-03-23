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

///@file util_conv.h
#pragma once

#include "Eigen/Geometry"
#include "drake/lcmt_iiwa_status.hpp"  // for lcmt_iiwa_status
#include "driver/communication_interface.h"
#include "franka/robot_state.h"  // for RobotState, RobotMode

/// NOTE @sprax: I doubt there is still a need to organized these functions
/// here. They could be elsewhere or be eliminated.
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

drake::lcmt_iiwa_status ConvertToLcmStatus(
    const franka_driver::RobotData& robot_data);

drake::lcmt_iiwa_status EigenToLcmStatus(Eigen::VectorXd robot_state);

franka::RobotState ConvertToCannonical(const franka::RobotState& robot_state,
                                       const Eigen::VectorXd& offsets);

}  //  namespace utils
