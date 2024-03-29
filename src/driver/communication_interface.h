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
/// When a plan is received, it will indicate via HasNewPlan() that a plan is
/// available. The plan is moved from this communication interface to the
/// franka plan runner when PopNewPlan() is called.
///
/// If a pause message is received, it will set the pause status to true and
/// keep track of what source paused it.

#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/piecewise_pose.h>

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <tuple>

#include <lcm/lcm-cpp.hpp>
#include <robot_msgs/bool_t.hpp>
#include <robot_msgs/driver_status_t.hpp>
#include <robot_msgs/pause_cmd.hpp>
#include <robot_msgs/plan_exec_opts_t.hpp>
#include <robot_msgs/robot_spline_t.hpp>
#include <robot_msgs/robot_status_t.hpp>

#include "franka/robot_state.h"
#include "utils/robot_parameters.h"

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PiecewisePose;

using hr_clock = std::chrono::high_resolution_clock;
using chrono_ms = std::chrono::milliseconds;
using std::chrono::duration_cast;

typedef PiecewisePolynomial<double> PPType;
typedef PiecewisePose<double> PosePoly;

namespace franka_driver {

// TODO(@anyone): remove this franka specific state and make it generic
struct RobotData {
  std::atomic<bool> has_robot_data;
  franka::RobotState robot_state;

  // time in seconds since start
  // of current plan
  double robot_time {};

  // utime associated with current plan.
  // defaults to -1 if no plan
  int64_t current_plan_utime {};

  // utime at start of current plan
  int64_t plan_start_utime {};

  // plan completion fraction.
  // should be in [0, 1]
  double plan_completion_frac {};

  Eigen::VectorXd robot_plan_next_conf;
};

struct PauseData {
  std::atomic<bool> paused;
  std::set<std::string> pause_sources;
};

struct PlanTimepoints {
  int64_t utime;
  std::chrono::time_point<hr_clock> t_received;
  std::chrono::time_point<hr_clock> t_accepted;
  std::optional<std::chrono::time_point<hr_clock>> t_confirmed {};
  std::optional<std::chrono::time_point<hr_clock>> t_started {};
};

struct RobotPlanBuffer {
  int64_t utime;
  int16_t exec_opt;
  Eigen::Vector3d contact_expected {};
  std::unique_ptr<PPType> plan;
  std::unique_ptr<PosePoly> cartesian_plan;
  PlanTimepoints timepoints;
};

class CommunicationInterface {
 public:
  explicit CommunicationInterface(
      const RobotParameters& params,
      const double lcm_publish_rate = 200.0 /* Hz */,
      const bool simulated = false);
  void StartInterface();
  void StopInterface();

  bool SimControlExceptionTriggered() const {
    return sim_control_exception_triggered_;
  }
  void ClearSimControlExceptionTrigger() {
    sim_control_exception_triggered_ = false;
  }

  /// @deprecated
  inline bool CompliantPushStartRequested() const {
    return compliant_push_start_requested_;
  }

  /// @deprecated
  void ClearCompliantPushStartRequest() {
    compliant_push_start_requested_ = false;
  }

  /// @deprecated
  inline bool CompliantPushStopRequested() const {
    return compliant_push_stop_requested_;
  }

  /// @deprecated
  void ClearCompliantPushStopRequest() {
    compliant_push_stop_requested_ = false;
  }

  /// @deprecated
  inline void SetCompliantPushActive(const bool active) {
    compliant_push_active_ = active;
  }

  inline void SetPlanTimepoints(const PlanTimepoints& timepoints) {
    timepoints_ = timepoints;
  }

  inline void LogPlanExecutionStartTime() {
    timepoints_.t_started = hr_clock::now();
  }

  bool CancelPlanRequested() const { return cancel_plan_requested_; }
  void ClearCancelPlanRequest() { cancel_plan_requested_ = false; }
  std::string GetCancelPlanSource() { return cancel_plan_source_; }

  bool HasNewPlan() {
    std::scoped_lock<std::mutex> lock {robot_plan_mutex_};
    return !(new_plan_buffer_.plan == nullptr);
  }

  /// @deprecated
  bool HasNewCartesianPlan() {
    std::scoped_lock<std::mutex> lock {robot_plan_mutex_};
    return !(new_plan_buffer_.cartesian_plan == nullptr);
  }

  void ClearNewPlan(std::string_view reason) {
    dexai::log()->warn("ClearNewPlan: {}", reason.data());
    SetPlanCompletion(new_plan_buffer_.utime, false, reason.data());
    new_plan_buffer_.plan.reset();
    new_plan_buffer_.cartesian_plan.reset();
    new_plan_buffer_.utime = -1;
  }

  std::tuple<std::unique_ptr<PPType>, int64_t, int16_t, Eigen::Vector3d,
             PlanTimepoints>
  PopNewPlan();

  /// @deprecated
  std::tuple<std::unique_ptr<PosePoly>, int64_t, int16_t, PlanTimepoints>
  PopNewCartesianPlan();

  // TODO(@anyone): remove franka specific RobotState type and
  // replace with std::array
  inline franka::RobotState GetRobotState() {
    std::scoped_lock<std::mutex> lock {robot_data_mutex_};
    return robot_data_.robot_state;
  }

  // acquire mutex lock and return robot mode
  franka::RobotMode GetRobotMode();

  /// Manual mode switch in sim as opposed to reading from Franka controller
  void SetModeIfSimulated(const franka::RobotMode& mode);

  // Set the robot state, blocking
  void SetRobotData(const franka::RobotState& robot_state,
                    const Eigen::VectorXd& robot_plan_next_conf,
                    const double robot_time, const int64_t current_plan_utime,
                    const int64_t plan_start_utime = -1,
                    const double plan_completion_frac = 0.0);

  inline bool GetPauseStatus() {
    return pause_data_.paused;  // this is atomic
  }
  inline void SetPauseStatus(const bool paused) {
    pause_data_.paused = paused;  // this is atomic
  }
  std::set<std::string> GetPauseSources() const {
    return pause_data_.pause_sources;
  }

  void SetPlanCompletion(const int64_t plan_utime, const bool success = true,
                         const std::string& driver_status_string = "");

  /// Set driver status fields with running status and error message.
  inline void SetDriverIsRunning(const bool success,
                                 const std::string& driver_status_string = "") {
    std::scoped_lock<std::mutex> lock {driver_status_mutex_};
    driver_status_msg_.driver_running = success;
    driver_status_msg_.err_msg = driver_status_string;
  }

  /// Update driver status and return publishable status message.
  robot_msgs::driver_status_t GetUpdatedDriverStatus(
      const int64_t utime, const RobotData& robot_data);

  void PublishBoolToChannel(const int64_t utime, std::string_view lcm_channel,
                            const bool data);
  void PublishPauseToChannel(const int64_t utime, std::string_view lcm_channel,
                             const int8_t data, std::string_view source = "");

  /// check if robot mode corresponds to one of the known control modes from the
  /// controller, return false if the mode is garbage
  static bool ModeIsValid(const franka::RobotMode& current_mode);

  /// check if robot is in a mode that can receive commands, i.e. not user
  /// stopped or error recovery
  static bool CanReceiveCommands(const franka::RobotMode& current_mode);

  /// Check if robot is user stopped. Return true if robot is in mode
  /// kUserStopped or kGuiding (manual move while user stopped)
  static bool IsUserStopped(const franka::RobotMode& current_mode);

 protected:
  void ResetData();
  void HandleLcm();
  /// PublishLcmAndPauseStatus is called once by a separate thread in the Run()
  /// method It sends the robot status and the pause status. Pause status
  /// is separate because robot status message used does not have space for
  /// indicating the contents of the pause message.
  void PublishLcmAndPauseStatus();
  void PublishRobotStatus();
  void PublishTriggerToChannel(const int64_t utime,
                               std::string_view lcm_channel,
                               const bool success = true,
                               std::string_view message = "");

  void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&,
                  const robot_msgs::robot_spline_t* robot_spline);
  void HandlePause(const ::lcm::ReceiveBuffer*, const std::string&,
                   const robot_msgs::pause_cmd* pause_cmd_msg);

  /// @deprecated
  void HandleCompliantPushReq(const ::lcm::ReceiveBuffer*, const std::string&,
                              const robot_msgs::bool_t* msg);

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
  std::atomic<bool> compliant_push_start_requested_ {false};
  std::atomic<bool> compliant_push_stop_requested_ {false};
  std::atomic<bool> compliant_push_active_ {false};

  std::atomic<bool> is_sim_ {false};

  ::lcm::LCM lcm_;

  // utime ID of last plan received and confirmed
  int64_t last_confirmed_plan_utime_;

  // This is a buffer storing the new plan received. Capacility is only 1.
  // Once this plan is popped (taken), this buffer is emptied and available
  // to store a new plan, while the current plan may be running.
  RobotPlanBuffer new_plan_buffer_;
  std::mutex robot_plan_mutex_;

  RobotData robot_data_;
  std::mutex robot_data_mutex_;

  PauseData pause_data_;
  PlanTimepoints timepoints_;

  std::mutex pause_mutex_;

  robot_msgs::driver_status_t driver_status_msg_;
  std::mutex driver_status_mutex_;

  std::thread lcm_publish_status_thread_;
  std::thread lcm_handle_thread_;

  std::string cancel_plan_source_;

  double lcm_publish_rate_;  // Hz
};

}  // namespace franka_driver
