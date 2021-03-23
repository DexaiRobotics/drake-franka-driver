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
