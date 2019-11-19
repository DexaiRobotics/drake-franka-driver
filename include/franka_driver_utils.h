/// @file franka_driver_utils

#pragma once

#include <bits/stdint-intn.h>    // for int64_t
#include <franka/robot_state.h>  // for RobotState, RobotMode

#include <array>    // for array
#include <cstddef>  // for size_t
#include <cstdint>  // for int64_t
#include <string>   // for string

#include "drake/lcmt_iiwa_status.hpp" // for lcmt_iiwa_status

namespace franka_driver {

const int kNumJoints_ = 7;
const std::string home_addr = "192.168.1.1";

template <typename T, std::size_t SIZE>
std::vector<T> ConvertToVector(std::array<T, SIZE>& a) {
  std::vector<T> v(a.begin(), a.end());
  return v;
}

template <typename T, std::size_t SIZE>
void ConvertToArray(std::vector<T>& v, std::array<T, SIZE>& a) {
  for (int i = 0; i < SIZE; i++) {
    a[i] = v[i];
  }
}

// TODO: @dmsj - make this call ConvertToLcmStatus()
static void AssignToLcmStatus(franka::RobotState& robot_state,
                              drake::lcmt_iiwa_status& robot_status);

drake::lcmt_iiwa_status ConvertToLcmStatus(franka::RobotState& robot_state);

void ResizeStatusMessage(drake::lcmt_iiwa_status& lcm_status);

std::string RobotModeToString(franka::RobotMode mode);

// TODO: use this
int64_t get_current_utime();

}  // namespace franka_driver
