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

#pragma once
/// @file communication_interface
///
/// communication_interface is designed to wait for LCM messages containing
/// a robot_spline_t message, and make it available to the franka_plan_runner
/// The interface also reports via LCM lcmt_franka_status
/// lcmt_franka_pause_status messages).
///
/// When a plan is received, it will indicate via HasPlan() that a plan is
/// available. The plan is moved from this communication interface to a franka
/// plan runner when the MovePlan() is called.
///
/// If a pause message is received, it will set the pause status to true and
/// keep track of what source paused it.

#include <drake/common/trajectories/piecewise_polynomial.h>  // for Piecewis...

#include <memory>
#include <mutex>  // for mutex
#include <set>
#include <string>
#include <thread>  // for thread
#include <tuple>

#include <lcm/lcm-cpp.hpp>  // for lcm

#include "franka/robot_state.h"         // for RobotState
#include "lcmtypes/robot_spline_t.hpp"  // for robot_spline_t
#include "robot_msgs/pause_cmd.hpp"     // for pause_cmd
#include "utils/robot_parameters.h"     // for RobotParameters

using drake::trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;

namespace franka_driver {

// TODO(@anyone): remove this franka specific state and make it generic
struct RobotData {
  std::atomic<bool> has_robot_data;
  franka::RobotState robot_state;
  Eigen::VectorXd robot_plan_next_conf;
};

struct PauseData {
  std::atomic<bool> paused;
  std::set<std::string> pause_sources;
};

struct RobotPiecewisePolynomial {
  int64_t utime;
  std::unique_ptr<PPType> plan;
};

class CommunicationInterface {
 public:
  explicit CommunicationInterface(const RobotParameters& params,
                                  double lcm_publish_rate = 200.0 /* Hz */);
  void StartInterface();
  void StopInterface();

  bool SimControlExceptionTriggered() const {
    return sim_control_exception_triggered_;
  }
  void ClearSimControlExceptionTrigger() {
    sim_control_exception_triggered_ = false;
  }

  bool CancelPlanRequested() const { return cancel_plan_requested_; }
  void ClearCancelPlanRequest() { cancel_plan_requested_ = false; }

  bool HasNewPlan() {
    std::scoped_lock<std::mutex> lock {robot_plan_mutex_};
    return !(new_plan_buffer_.plan == nullptr);
  }

  std::tuple<std::unique_ptr<PPType>, int64_t> PopNewPlan();

  // TODO(@anyone): remove franka specific RobotState type and
  // replace with std::array
  franka::RobotState GetRobotState();

  // acquire mutex lock and return robot mode
  franka::RobotMode GetRobotMode();

  // Set the robot state, blocking
  void SetRobotData(const franka::RobotState& robot_state,
                    const Eigen::VectorXd& robot_plan_next_conf);

  bool GetPauseStatus();
  void SetPauseStatus(bool paused);
  std::set<std::string> GetPauseSources() const {
    return pause_data_.pause_sources;
  }

  void PublishPlanComplete(const int64_t& plan_utime, bool success = true,
                           std::string driver_status_string = "");

  void PublishDriverStatus(bool success, std::string driver_status_string = "");
  void PublishBoolToChannel(int64_t utime, std::string_view lcm_channel,
                            bool data);

  std::string GetUserStopChannelName() { return lcm_user_stop_channel_; }

 protected:
  void ResetData();
  void HandleLcm();
  /// PublishLcmAndPauseStatus is called once by a separate thread in the Run()
  /// method It sends the robot status and the pause status. Pause status
  /// is separate because robot status message used does not have space for
  /// indicating the contents of the pause message.
  void PublishLcmAndPauseStatus();
  void PublishRobotStatus();
  void PublishPauseStatus();
  void PublishTriggerToChannel(int64_t utime, std::string_view lcm_channel,
                               bool success = true,
                               std::string_view message = "");
  /// check if robot is in a mode that can receive commands, i.e. not user
  /// stopped or error recovery
  bool CanReceiveCommands(const franka::RobotMode& current_mode);
  void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&,
                  const lcmtypes::robot_spline_t* robot_spline);
  void HandlePause(const ::lcm::ReceiveBuffer*, const std::string&,
                   const robot_msgs::pause_cmd* pause_cmd_msg);

  /// Handler for control exception and u-stop triggers for simulated driver so
  /// we can test full spectrum of driver states.
  void HandleSimDriverEventTrigger(
      const ::lcm::ReceiveBuffer*, const std::string&,
      const robot_msgs::pause_cmd* trigger_cmd_msg);

 private:
  RobotParameters params_;
  std::atomic_bool running_ {false};
  std::atomic<bool> sim_control_exception_triggered_ {false};
  std::atomic<bool> cancel_plan_requested_ {false};

  ::lcm::LCM lcm_;

  // This is a buffer storing the new plan received. Capacility is only 1.
  // Once this plan is popped (taken), this buffer is emptied and avialable
  // to store a new plan, while the current plan may be running.
  RobotPiecewisePolynomial new_plan_buffer_;
  std::mutex robot_plan_mutex_;

  RobotData robot_data_;
  std::mutex robot_data_mutex_;

  PauseData pause_data_;
  std::mutex pause_mutex_;

  std::thread lcm_publish_status_thread_;
  std::thread lcm_handle_thread_;

  std::string lcm_driver_status_channel_;
  std::string lcm_pause_status_channel_;
  std::string lcm_user_stop_channel_;
  std::string lcm_brakes_locked_channel_;
  std::string lcm_sim_driver_event_trigger_channel_;

  double lcm_publish_rate_;  // Hz
};

}  // namespace franka_driver