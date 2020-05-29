///@file util_conv.h
#pragma once

#include "drake/lcmt_iiwa_status.hpp"  // for lcmt_iiwa_status
#include "franka/robot_state.h"        // for RobotState, RobotMode
#include "communication_interface.h"
#include "Eigen/Geometry"

/// NOTE @sprax: I doubt there is still a need to organized these functions here.
/// They could be elsewhere or be eliminated.
namespace utils {

    // Enums for Defining Status
    enum class RobotStatus {
      Uninitialized,
      Running,
      Pausing,
      Paused,
      Unpausing,
      Reversing
    };

    // Status Conversions:

    std::string RobotModeToString(franka::RobotMode mode);

    std::string RobotStatusToString(RobotStatus status);

    drake::lcmt_iiwa_status ConvertToLcmStatus(const franka_driver::RobotData& robot_data); // (franka::RobotState& robot_state);

    drake::lcmt_iiwa_status EigenToLcmStatus(Eigen::VectorXd robot_state);

}   //  namespace utils
