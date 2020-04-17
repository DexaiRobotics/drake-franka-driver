/// @file franka_driver_utils

#pragma once

#include "drake/lcmt_iiwa_status.hpp"  // for lcmt_iiwa_status
#include "franka/robot_state.h"        // for RobotState, RobotMode

#include <Eigen/Dense>         // for Eigen::VectorXd
#include <array>               // for array
#include <bits/stdint-intn.h>  // for int64_t
#include <cstddef>             // for size_t
#include <cstdint>             // for int64_t
#include <string>              // for string

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

    // Status Conversions
    drake::lcmt_iiwa_status ConvertToLcmStatus(franka::RobotState& robot_state);

    std::string RobotModeToString(franka::RobotMode mode);

    std::string RobotStatusToString(RobotStatus status);

    // Vector Conversions:
    template <typename T, std::size_t SIZE>
    std::vector<T> ArrayToVector(const std::array<T, SIZE>& a) {
      std::vector<T> v(a.begin(), a.end());
      return v;
    }

    template <typename T, std::size_t SIZE>
    void VectorToArray(const std::vector<T>& v, std::array<T, SIZE>& a) {
      for (int i = 0; i < SIZE; i++) {
        a[i] = v[i];
      }
    }

    std::array<double, 7> EigenToArray(const Eigen::VectorXd& input);

}  // namespace utils

