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

/// @file communication_interface.cc
///
/// communication_interface is designed to wait for LCM messages containing
/// a robot_spline_t message, and make it available to the franka_plan_runner
/// The interface also reports via LCM lcmt_franka_status
/// lcmt_franka_pause_status messages).
///
/// When a plan is received, it will indicate via HasNewPlan() that a plan is
/// available. The plan is moved from this communication interface to the
/// franka plan runner when the PopNewPlan() is called.
///
/// If a pause message is received, it will set the pause status to true and
/// keep track of what source paused it.

#include "driver/communication_interface.h"

#include <chrono>   // for steady_clock, for duration
#include <utility>  // for move

#include "robot_msgs/bool_t.hpp"
#include "robot_msgs/pause_cmd.hpp"          // for pause_cmd
#include "robot_msgs/trigger_t.hpp"          // for trigger_t
#include "utils/polynomial_encode_decode.h"  // for decodePiecewisePolynomial
#include "utils/util_conv.h"                 // ConvertToLcmIiwaStatus
#include "utils/util_io.h"                   // for get_current_utime

// using namespace franka_driver;
using franka_driver::CommunicationInterface;
using franka_driver::PlanTimepoints;
using utils::PauseCommandType;

CommunicationInterface::CommunicationInterface(const RobotParameters& params,
                                               const double lcm_publish_rate,
                                               const bool simulated)
    : params_ {params},
      is_sim_ {simulated},
      lcm_ {params_.lcm_url},
      lcm_publish_rate_ {lcm_publish_rate} {
  lcm_.subscribe(params_.lcm_plan_channel, &CommunicationInterface::HandlePlan,
                 this);
  lcm_.subscribe(params_.lcm_stop_channel, &CommunicationInterface::HandlePause,
                 this);
  lcm_.subscribe(params_.lcm_compliant_push_req_channel,
                 &CommunicationInterface::HandleCompliantPushReq, this);
  lcm_.subscribe(params_.lcm_sim_driver_event_trigger_channel,
                 &CommunicationInterface::HandleSimDriverEventTrigger, this);

  dexai::log()->info("Plan channel:\t\t\t\t{}", params_.lcm_plan_channel);
  dexai::log()->info("Stop channel:\t\t\t\t{}", params_.lcm_stop_channel);
  dexai::log()->info("IIWA Status channel:\t\t\t{}",
                     params_.lcm_iiwa_status_channel);
  dexai::log()->info("Robot Status channel:\t\t\t{}",
                     params_.lcm_robot_status_channel);
  dexai::log()->info("Driver status channel:\t\t\t{}",
                     params_.lcm_driver_status_channel);
  dexai::log()->info("Sim driver event trigger channel:\t{}",
                     params_.lcm_sim_driver_event_trigger_channel);

  {
    std::scoped_lock<std::mutex> status_lock {driver_status_mutex_};
    driver_status_msg_.current_plan_utime = -1;
    driver_status_msg_.plan_start_utime = -1;
    driver_status_msg_.last_plan_utime = -1;
    driver_status_msg_.last_plan_successful = false;
    driver_status_msg_.torque_enabled = true;
  }
};

void CommunicationInterface::ResetData() {
  std::unique_lock<std::mutex> lock_data(robot_data_mutex_);
  robot_data_.has_robot_data = false;
  robot_data_.robot_state = franka::RobotState();
  lock_data.unlock();

  // initialize plan as empty:
  std::unique_lock<std::mutex> lock_plan(robot_plan_mutex_);
  new_plan_buffer_.plan.reset();            // unique ptr points to no plan
  new_plan_buffer_.cartesian_plan.reset();  // unique ptr points to no plan
  new_plan_buffer_.utime = -1;              // utime set to -1 at start
  new_plan_buffer_.exec_opt = robot_msgs::plan_exec_opts_t::DEFAULT;
  lock_plan.unlock();

  // initialize pause as false:
  std::unique_lock<std::mutex> lock_pause(pause_mutex_);
  pause_data_.paused = false;         // not paused at start
  pause_data_.pause_sources.clear();  // no pause sources at start
  lock_pause.unlock();
}

void CommunicationInterface::StartInterface() {
  // Initialize data as empty for exchange with robot driver
  ResetData();

  // initialize robot mode to idle when running in simulation.
  // on the real Franka, the robot mode gets read in via libfranka from the
  // actual robot arm
  SetModeIfSimulated(franka::RobotMode::kIdle);

  // start LCM threads; independent of sim vs. real robot
  dexai::log()->info("CommInterface:StartInterface: Start LCM threads");
  running_ = true;  // sets the lcm threads to active.
  lcm_publish_status_thread_ =
      std::thread(&CommunicationInterface::PublishLcmAndPauseStatus, this);
  lcm_handle_thread_ = std::thread(&CommunicationInterface::HandleLcm, this);
}

void CommunicationInterface::StopInterface() {
  dexai::log()->info("CommInterface:StopInterface: Before LCM threads join");
  // clean-up threads if they're still alive.
  running_ = false;  // sets the lcm threads to inactive.
  while (!lcm_handle_thread_.joinable()
         || !lcm_publish_status_thread_.joinable()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dexai::log()->warn(
        "CommInterface:StopInterface: Waiting for LCM threads to be "
        "joinable...");
  }
  lcm_publish_status_thread_.join();
  lcm_handle_thread_.join();
  dexai::log()->info("CommInterface:StopInterface: After LCM thread join");
  ResetData();
}

std::tuple<std::unique_ptr<PPType>, int64_t, int16_t, Eigen::Vector3d,
           PlanTimepoints>
CommunicationInterface::PopNewPlan() {
  if (!HasNewPlan()) {
    throw std::runtime_error(
        fmt::format("PopNewPlan: no buffered new plan available to pop; utime "
                    "in buffer: {}",
                    new_plan_buffer_.utime));
  }

  SetModeIfSimulated(franka::RobotMode::kMove);

  std::scoped_lock<std::mutex> lock {robot_plan_mutex_};
  // std::move nullifies the unique ptr new_plan_buffer_.plan
  return {std::move(new_plan_buffer_.plan), new_plan_buffer_.utime,
          new_plan_buffer_.exec_opt, new_plan_buffer_.contact_expected,
          new_plan_buffer_.timepoints};
}

/// @deprecated
std::tuple<std::unique_ptr<PosePoly>, int64_t, int16_t, PlanTimepoints>
CommunicationInterface::PopNewCartesianPlan() {
  if (!HasNewCartesianPlan()) {
    throw std::runtime_error(
        fmt::format("PopNewCartesianPlan: no buffered new plan available to "
                    "pop; utime in buffer: {}",
                    new_plan_buffer_.utime));
  }
  SetModeIfSimulated(franka::RobotMode::kMove);

  std::scoped_lock<std::mutex> lock {robot_plan_mutex_};
  // std::move nullifies the unique ptr new_plan_buffer_.cartesian_plan
  return {std::move(new_plan_buffer_.cartesian_plan), new_plan_buffer_.utime,
          new_plan_buffer_.exec_opt, new_plan_buffer_.timepoints};
}

void CommunicationInterface::SetRobotData(
    const franka::RobotState& robot_state,
    const Eigen::VectorXd& robot_plan_next_conf, const double robot_time,
    const int64_t current_plan_utime, const int64_t plan_start_utime,
    const double plan_completion_frac) {
  std::scoped_lock<std::mutex> lock {robot_data_mutex_};
  franka::RobotMode current_mode {robot_data_.robot_state.robot_mode};
  robot_data_.robot_state = robot_state;
  robot_data_.robot_plan_next_conf = robot_plan_next_conf;
  robot_data_.robot_time = robot_time;
  robot_data_.current_plan_utime = current_plan_utime;
  robot_data_.plan_start_utime = plan_start_utime;
  robot_data_.plan_completion_frac = plan_completion_frac;
  robot_data_.has_robot_data = true;
  // when running on the real robot, the robot_state passed into this function
  // is retrieved from libfranka and gives us accurate information about the
  // robot's operating mode, in sim we just want to propagate the existing mode
  // so we don't overwrite after changing it in simulation-specific logic (sim
  // driver event handler)
  if (is_sim_) {
    robot_data_.robot_state.robot_mode = current_mode;
  }
}

void CommunicationInterface::SetPlanCompletion(
    const int64_t plan_utime, const bool success,
    const std::string& plan_status_string) {
  std::string log_msg {
      fmt::format("CommInterface:SetPlanCompletion: plan {} {}", plan_utime,
                  success ? "successful" : "failed")};
  if (!success) {
    log_msg += fmt::format(", error status: {}", plan_status_string);
  }
  dexai::log()->info(log_msg);
  new_plan_buffer_.plan.reset();
  new_plan_buffer_.cartesian_plan.reset();

  {
    std::scoped_lock<std::mutex> status_lock {driver_status_mutex_};
    driver_status_msg_.last_plan_utime = plan_utime;
    driver_status_msg_.last_plan_successful = success;
    driver_status_msg_.last_plan_msg = plan_status_string;
  }
  SetModeIfSimulated(franka::RobotMode::kIdle);

  // sanity check and make sure we have all the timepoints, as plans can be
  // received and accepted but not make it to confirmation and execution
  if (plan_utime == timepoints_.utime && timepoints_.t_confirmed.has_value()
      && timepoints_.t_started.has_value()) {
    const auto ms_accept {duration_cast<chrono_ms>(timepoints_.t_accepted
                                                   - timepoints_.t_received)
                              .count()};
    const auto ms_confirm {
        duration_cast<chrono_ms>(timepoints_.t_confirmed.value()
                                 - timepoints_.t_received)
            .count()};
    const auto ms_start {duration_cast<chrono_ms>(timepoints_.t_started.value()
                                                  - timepoints_.t_received)
                             .count()};

    dexai::log()->info(
        "CommInterface:SetPlanCompletion: plan {} timing breakdown: input "
        "checking: {} ms, confirmation: {} ms, execution start: {} ms",
        plan_utime, ms_accept, ms_confirm, ms_start);
  }
}

void CommunicationInterface::HandleLcm() {
  while (running_) {
    lcm_.handleTimeout(0);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

void CommunicationInterface::PublishLcmAndPauseStatus() {
  auto desired_wait =
      std::chrono::milliseconds(static_cast<int>(1000.0 / lcm_publish_rate_));
  while (running_) {
    auto time_start = std::chrono::steady_clock::now();
    PublishRobotStatus();

    // Sleep dynamically to achieve the desired print rate.
    auto time_end = std::chrono::steady_clock::now();
    auto time_elapsed = time_end - time_start;
    auto remaining_wait = desired_wait - time_elapsed;

    if (remaining_wait < std::chrono::seconds(0)) {
      std::chrono::milliseconds time_elapsed_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(time_elapsed);
      dexai::log()->warn(
          "CommInterface:PublishLcmAndPauseStatus:"
          " publish took too long with {} ms > {} ms!",
          time_elapsed_ms.count(), 1000.0 / lcm_publish_rate_);
    }
    std::this_thread::sleep_for(remaining_wait);
  }
}

void CommunicationInterface::PublishRobotStatus() {
  // Try to lock data to avoid read write collisions.
  std::unique_lock<std::mutex> lock {robot_data_mutex_};

  drake::lcmt_iiwa_status franka_status {
      utils::ConvertToLcmIiwaStatus(robot_data_)};
  auto driver_status_msg {
      GetUpdatedDriverStatus(franka_status.utime, robot_data_)};
  // TODO(@syler): we don't continuosly update robot data when robot is in
  // reflex/automatic error recovery mode so those modes will never be reflected
  // in driver status
  lcm_.publish(params_.lcm_driver_status_channel, &driver_status_msg);

  if (robot_data_.has_robot_data) {
    robot_msgs::robot_status_t robot_status {
        utils::ConvertToRobotStatusLcmMsg(robot_data_)};

    franka::RobotMode current_mode {robot_data_.robot_state.robot_mode};

    robot_data_.has_robot_data = false;
    lock.unlock();
    // publish data over lcm
    lcm_.publish(params_.lcm_iiwa_status_channel, &franka_status);
    lcm_.publish(params_.lcm_robot_status_channel, &robot_status);

    // Cancel robot plans if robot is U-stopped.
    if (IsUserStopped(current_mode)) {
      PublishPauseToChannel(franka_status.utime, params_.lcm_stop_channel,
                            PauseCommandType::CANCEL_PLAN,
                            fmt::format("{}_U_STOP", params_.robot_name));
    }
  }
}

robot_msgs::driver_status_t CommunicationInterface::GetUpdatedDriverStatus(
    const int64_t utime, const RobotData& robot_data) {
  std::scoped_lock<std::mutex> status_lock {driver_status_mutex_};

  driver_status_msg_.utime = utime;

  franka::RobotMode current_mode {robot_data.robot_state.robot_mode};

  if (robot_data.current_plan_utime != -1) {
    const auto prev_current_plan_utime {driver_status_msg_.current_plan_utime};
    if (prev_current_plan_utime != robot_data.current_plan_utime) {
      timepoints_.t_confirmed = hr_clock::now();
    }
  }
  driver_status_msg_.current_plan_utime = robot_data.current_plan_utime;
  driver_status_msg_.plan_start_utime = robot_data.plan_start_utime;
  driver_status_msg_.has_plan = robot_data.current_plan_utime != -1;
  driver_status_msg_.brakes_locked = current_mode == franka::RobotMode::kOther;
  driver_status_msg_.user_stopped = IsUserStopped(current_mode);
  driver_status_msg_.robot_mode = utils::RobotModeToString(current_mode);
  driver_status_msg_.compliant_push_active = compliant_push_active_;

  {
    std::scoped_lock<std::mutex> pause_lock {pause_mutex_};
    driver_status_msg_.paused = pause_data_.paused;
    driver_status_msg_.pause_sources.clear();
    driver_status_msg_.num_pause_sources = pause_data_.pause_sources.size();
    if (pause_data_.paused) {
      for (const auto& source : pause_data_.pause_sources) {
        driver_status_msg_.pause_sources.push_back(source);
      }
    }
  }
  return driver_status_msg_;
}

void CommunicationInterface::PublishTriggerToChannel(
    const int64_t utime, std::string_view lcm_channel, const bool success,
    std::string_view message) {
  robot_msgs::trigger_t msg;
  msg.utime = utime;
  msg.success = success;
  msg.message = message.data();
  lcm_.publish(lcm_channel.data(), &msg);
}

void CommunicationInterface::PublishPauseToChannel(const int64_t utime,
                                                   std::string_view lcm_channel,
                                                   const int8_t data,
                                                   std::string_view source) {
  robot_msgs::pause_cmd msg;
  msg.utime = utime;
  msg.data = data;
  msg.source = source.data();
  lcm_.publish(lcm_channel.data(), &msg);
}

void CommunicationInterface::PublishBoolToChannel(const int64_t utime,
                                                  std::string_view lcm_channel,
                                                  const bool data) {
  robot_msgs::bool_t msg;
  msg.utime = utime;
  msg.data = data;
  lcm_.publish(lcm_channel.data(), &msg);
}

franka::RobotMode CommunicationInterface::GetRobotMode() {
  std::scoped_lock<std::mutex> lock {robot_data_mutex_};
  return robot_data_.robot_state.robot_mode;
}

void CommunicationInterface::SetModeIfSimulated(const franka::RobotMode& mode) {
  log()->trace("CommInterface:SetModeIfSimulated: setting to {}",
               utils::RobotModeToString(mode));
  if (is_sim_) {
    std::scoped_lock<std::mutex> lock {robot_data_mutex_};
    robot_data_.robot_state.robot_mode = mode;
    log()->info("CommInterface:SetModeIfSimulated: set mode to {}",
                utils::RobotModeToString(mode));
  }
}

bool CommunicationInterface::ModeIsValid(
    const franka::RobotMode& current_mode) {
  switch (current_mode) {
    case franka::RobotMode::kOther:
      return true;
    case franka::RobotMode::kIdle:
      return true;
    case franka::RobotMode::kMove:
      return true;
    case franka::RobotMode::kGuiding:
      return true;
    case franka::RobotMode::kReflex:
      return true;
    case franka::RobotMode::kUserStopped:
      return true;
    case franka::RobotMode::kAutomaticErrorRecovery:
      return true;
  }
  log()->error("ModeIsValid: got invalid mode: {}", current_mode);
  return false;
}

bool CommunicationInterface::CanReceiveCommands(
    const franka::RobotMode& current_mode) {
  switch (current_mode) {
    case franka::RobotMode::kOther:
      return false;
    case franka::RobotMode::kIdle:
      return true;
    case franka::RobotMode::kMove:
      dexai::log()->warn(
          "CanReceiveCommands: allowing to receive commands while in "
          "Move mode, but this needs more testing!",
          utils::RobotModeToString(current_mode));
      return true;
    case franka::RobotMode::kGuiding:
      return false;
    case franka::RobotMode::kReflex:
      return false;
    case franka::RobotMode::kUserStopped:
      return false;
    case franka::RobotMode::kAutomaticErrorRecovery:
      return false;
    default:
      dexai::log()->error("CanReceiveCommands: Mode unknown!");
      return false;
  }
}

bool CommunicationInterface::IsUserStopped(
    const franka::RobotMode& current_mode) {
  return (current_mode == franka::RobotMode::kGuiding)
         || (current_mode == franka::RobotMode::kUserStopped);
}

/// @deprecated
void CommunicationInterface::HandleCompliantPushReq(
    const ::lcm::ReceiveBuffer*, const std::string&,
    const robot_msgs::bool_t* msg) {
  if (msg->data) {
    compliant_push_start_requested_ = true;
  } else if (compliant_push_active_) {
    compliant_push_stop_requested_ = true;
  } else {
    log()->warn("CompliantPush not currently active but STOP requested!");
  }
}

void CommunicationInterface::HandlePlan(
    const ::lcm::ReceiveBuffer*, const std::string&,
    const robot_msgs::robot_spline_t* robot_spline) {
  auto t_start {hr_clock::now()};

  dexai::log()->info("CommInterface:HandlePlan: Received new plan {}",
                     robot_spline->utime);

  if (robot_spline->utime == last_confirmed_plan_utime_) {
    // Looks like plan received confirmation was not received. driver status
    // continuously publish active plan utime, so do nothing
    log()->warn("CommInterface:HandlePlan: Exact same plan received again");
    return;
  }

  // occasionally the controller will return garbage instead of a valid mode
  // reading. try querying the mode a few times before giving up
  auto current_mode {GetRobotMode()};
  size_t max_mode_check_attempts {5};
  for (size_t i {}; i < max_mode_check_attempts; i++) {
    if (ModeIsValid(current_mode)) {
      break;
    }
    SetDriverIsRunning(false, "Franka controller is reporting an unknown mode");
    log()->error("HandlePlan: attempt {}/{} to read control mode from Franka",
                 i + 1, max_mode_check_attempts);
    current_mode = GetRobotMode();
  }

  // check if in proper mode to receive commands
  if (!CanReceiveCommands(current_mode)) {
    const auto err_msg {fmt::format(
        "Discarding plan with utime: {}, robot is in wrong mode: {}!",
        robot_spline->utime, utils::RobotModeToString(current_mode))};
    dexai::log()->error("CommInterface:HandlePlan: {}", err_msg);
    SetDriverIsRunning(false, err_msg);
    SetPlanCompletion(robot_spline->utime, false, err_msg);
    return;
  }

  std::unique_lock<std::mutex> lock(robot_plan_mutex_, std::defer_lock);
  while (!lock.try_lock()) {
    dexai::log()->warn(
        "CommInterface:HandlePlan: trying to get a lock on the "
        "robot_plan_mutex_. Sleeping 1 ms and trying again.");
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1.0)));
  }

  new_plan_buffer_.utime = robot_spline->utime;
  new_plan_buffer_.timepoints.utime = robot_spline->utime;
  new_plan_buffer_.exec_opt = robot_spline->exec_opt;
  last_confirmed_plan_utime_ = robot_spline->utime;

  // Piecewise polynomial
  if (robot_spline->num_states > 0) {
    PPType piecewise_polynomial {
        decodePiecewisePolynomial(robot_spline->piecewise_polynomial)};

    if (piecewise_polynomial.get_number_of_segments() < 1) {
      const auto err_msg {"Invalid piecewise polynomial"};
      dexai::log()->error("CommInterface:HandlePlan: Discarding plan: {}",
                          err_msg);
      SetPlanCompletion(robot_spline->utime, false, err_msg);
      return;
    }

    dexai::log()->info(
        "CommInterface:HandlePlan: "
        "plan {}, start time: {:.2f}, end time: {:.2f}",
        robot_spline->utime, piecewise_polynomial.start_time(),
        piecewise_polynomial.end_time());

    // Start position == goal position check
    // TODO(@anyone): change to append initial position and respline here
    Eigen::VectorXd commanded_start {
        piecewise_polynomial.value(piecewise_polynomial.start_time())};

    // TODO(@anyone): move this check to franka plan runner
    Eigen::VectorXd q_eigen {
        utils::v_to_e(utils::ArrayToVector(this->GetRobotState().q))};

    double max_angular_distance {
        utils::max_angular_distance(commanded_start, q_eigen)};
    if (max_angular_distance > params_.kMediumJointDistance) {
      // discard the plan if we are too far away from current robot start
      auto joint_delta {q_eigen - commanded_start};
      dexai::log()->error(
          "CommInterface:HandlePlan: "
          "discarding plan {}, mismatched start position with delta: {}.",
          robot_spline->utime, joint_delta.transpose());
      new_plan_buffer_.plan.reset();
      lock.unlock();
      SetPlanCompletion(robot_spline->utime, false /*  = failed*/,
                        "Mismatched start position");
      return;
    }
    new_plan_buffer_.plan = std::make_unique<PPType>(piecewise_polynomial);

    if (new_plan_buffer_.exec_opt
        == robot_msgs::plan_exec_opts_t::MOVE_UNTIL_STOP) {
      new_plan_buffer_.contact_expected =
          utils::ToRigidTransform(robot_spline->cartesian_goal).translation();
    }

  } else {
    // Cartesian goal command. Not implemented yet. Ignore for now
  }
  new_plan_buffer_.timepoints.t_received = t_start;
  new_plan_buffer_.timepoints.t_accepted = hr_clock::now();

  lock.unlock();

  dexai::log()->info(
      "CommInterface:HandlePlan: populated buffer with new plan {}",
      new_plan_buffer_.utime);
  auto ms_accept {
      duration_cast<chrono_ms>(new_plan_buffer_.timepoints.t_accepted
                               - new_plan_buffer_.timepoints.t_received)
          .count()};

  dexai::log()->info(
      "CommInterface:HandlePlan: Finished input checking plan {} in {} ms",
      new_plan_buffer_.utime, ms_accept);
}

void CommunicationInterface::HandlePause(
    const ::lcm::ReceiveBuffer*, const std::string&,
    const robot_msgs::pause_cmd* pause_cmd_msg) {
  std::lock_guard<std::mutex> lock {pause_mutex_};
  // check if paused = true or paused = false was received:
  auto source {pause_cmd_msg->source};

  PauseCommandType pause_type {
      static_cast<PauseCommandType>(pause_cmd_msg->data)};

  switch (pause_type) {
    case PauseCommandType::CANCEL_PLAN: {
      const auto err_msg {
          fmt::format("CommInterface:HandlePause: Received cancel plan request "
                      "with source: {}",
                      source)};
      // warn on first cancellation received (we publish continuously until
      // resolved)
      if (!cancel_plan_requested_) {
        dexai::log()->warn(err_msg);
      } else {
        dexai::log()->debug(err_msg);
      }
      cancel_plan_requested_ = true;
      cancel_plan_source_ = source;
      break;
    }
    case PauseCommandType::PAUSE:
      dexai::log()->warn(
          "CommInterface:HandlePause: received pause command from source: {}",
          source);
      if (pause_data_.pause_sources.insert(source).second == false) {
        dexai::log()->warn(
            "CommInterface:HandlePause: Already paused by source: '{}'",
            source);
      }
      break;
    case PauseCommandType::CONTINUE:
      dexai::log()->warn(
          "CommInterface:HandlePause: received continue command from "
          "source: {}",
          source);
      if (pause_data_.pause_sources.find(source)
          != pause_data_.pause_sources.end()) {
        pause_data_.pause_sources.erase(source);
      } else {
        dexai::log()->warn(
            "CommInterface:HandlePause: ignoring continue command from "
            "source: {}, as the robot has not been paused by it",
            source);
      }
      break;
    default:
      dexai::log()->error(
          "CommInterface:HandlePause: ignoring unknown pause command type "
          "from source: {}",
          source);
      break;
  }

  // if the set of pause sources is empty, then
  // the robot is not paused anymore:
  if (pause_data_.pause_sources.size() == 0) {
    pause_data_.paused = false;
  } else {
    pause_data_.paused = true;
  }
}

// TODO(@syler): this could be a different message type, pause_cmd is probably
// not the best here
void CommunicationInterface::HandleSimDriverEventTrigger(
    const ::lcm::ReceiveBuffer*, const std::string&,
    const robot_msgs::pause_cmd* cmd_msg) {
  auto desired_event {cmd_msg->source};

  // simulate control exception
  if (desired_event == "control_exception") {
    dexai::log()->error(
        "CommInterface:HandleSimDriverEventTrigger: received command "
        "to simulate control exception!");
    SetModeIfSimulated(franka::RobotMode::kReflex);
    sim_control_exception_triggered_ = true;
    return;
  }
  if (desired_event == "brakes") {
    std::scoped_lock<std::mutex> lock {robot_data_mutex_};
    if (cmd_msg->data) {
      dexai::log()->warn(
          "CommInterface:HandleSimDriverEventTrigger: received "
          "command to simulate brakes lock");
      robot_data_.robot_state.robot_mode = franka::RobotMode::kOther;
    } else {
      dexai::log()->warn(
          "CommInterface:HandleSimDriverEventTrigger: received "
          "command to simulate brakes unlock");
      robot_data_.robot_state.robot_mode = franka::RobotMode::kIdle;
    }
    return;
  }
  if (desired_event == "u_stop") {
    std::scoped_lock<std::mutex> lock {robot_data_mutex_};
    if (cmd_msg->data) {
      dexai::log()->warn(
          "CommInterface:HandleSimDriverEventTrigger: received "
          "command to simulate User Stop button press");
      robot_data_.robot_state.robot_mode = franka::RobotMode::kUserStopped;
    } else {
      dexai::log()->warn(
          "CommInterface:HandleSimDriverEventTrigger: received "
          "command to simulate User Stop release");
      robot_data_.robot_state.robot_mode = franka::RobotMode::kIdle;
    }
  } else {
    dexai::log()->error(
        "CommInterface:HandleSimDriverEventTrigger: Unrecognized "
        "trigger {}!",
        desired_event);
  }
}
