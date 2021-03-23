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
/// When a plan is received, it will indicate via HasPlan() that a plan is
/// available. The plan is moved from this communication interface to a franka
/// plan runner when the MovePlan() is called.
///
/// If a pause message is received, it will set the opause status to true and
/// keep track of what source paused it.

#include "driver/communication_interface.h"

#include <chrono>   // for steady_clock, for duration
#include <utility>  // for move

#include "robot_msgs/bool_t.hpp"
#include "robot_msgs/pause_cmd.hpp"          // for pause_cmd
#include "robot_msgs/trigger_t.hpp"          // for trigger_t
#include "utils/polynomial_encode_decode.h"  // for decodePiecewisePolynomial
#include "utils/util_conv.h"                 // ConvertToLcmStatus
#include "utils/util_io.h"                   // for get_current_utime

// using namespace franka_driver;
using franka_driver::CommunicationInterface;
using utils::PauseCommandType;

CommunicationInterface::CommunicationInterface(const RobotParameters& params,
                                               double lcm_publish_rate)
    : params_ {params},
      lcm_ {params_.lcm_url},
      lcm_publish_rate_ {lcm_publish_rate} {
  lcm_.subscribe(params_.lcm_plan_channel, &CommunicationInterface::HandlePlan,
                 this);
  lcm_.subscribe(params_.lcm_stop_channel, &CommunicationInterface::HandlePause,
                 this);

  // TODO(@anyone): define this in parameters file
  lcm_driver_status_channel_ = params_.robot_name + "_DRIVER_STATUS";
  // TODO(@anyone): remove these channels and combine with robot status channel
  lcm_pause_status_channel_ = params_.robot_name + "_PAUSE_STATUS";
  lcm_user_stop_channel_ = params_.robot_name + "_USER_STOPPED";
  lcm_brakes_locked_channel_ = params_.robot_name + "_BRAKES_LOCKED";
  lcm_sim_driver_event_trigger_channel_ =
      params_.robot_name + "_SIM_EVENT_TRIGGER";

  lcm_.subscribe(lcm_sim_driver_event_trigger_channel_,
                 &CommunicationInterface::HandleSimDriverEventTrigger, this);

  dexai::log()->info("Plan channel:\t\t\t\t{}", params_.lcm_plan_channel);
  dexai::log()->info("Stop channel:\t\t\t\t{}", params_.lcm_stop_channel);
  dexai::log()->info("Plan received channel:\t\t\t{}",
                     params_.lcm_plan_received_channel);
  dexai::log()->info("Plan complete channel:\t\t\t{}",
                     params_.lcm_plan_complete_channel);
  dexai::log()->info("Status channel:\t\t\t{}", params_.lcm_status_channel);
  dexai::log()->info("Driver status channel:\t\t\t{}",
                     lcm_driver_status_channel_);
  dexai::log()->info("Pause status channel:\t\t\t{}",
                     lcm_pause_status_channel_);
  dexai::log()->info("Sim driver event trigger channel:\t{}",
                     lcm_sim_driver_event_trigger_channel_);
};

void CommunicationInterface::ResetData() {
  std::unique_lock<std::mutex> lock_data(robot_data_mutex_);
  robot_data_.has_robot_data = false;
  robot_data_.robot_state = franka::RobotState();
  lock_data.unlock();

  // initialize plan as empty:
  std::unique_lock<std::mutex> lock_plan(robot_plan_mutex_);
  new_plan_buffer_.plan.release();  // unique ptr points to no plan
  new_plan_buffer_.utime = -1;      // utime set to -1 at start
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

std::tuple<std::unique_ptr<PPType>, int64_t>
CommunicationInterface::PopNewPlan() {
  if (!HasNewPlan()) {
    throw std::runtime_error(
        "PopNewPlan: no buffered new plan available to pop!");
  }
  std::scoped_lock<std::mutex> lock {robot_plan_mutex_};
  // std::move nullifies the unique ptr robot_plan_.plan_
  return {std::move(new_plan_buffer_.plan), new_plan_buffer_.utime};
}

franka::RobotState CommunicationInterface::GetRobotState() {
  std::scoped_lock<std::mutex> lock {robot_data_mutex_};
  return robot_data_.robot_state;
}

void CommunicationInterface::SetRobotData(
    const franka::RobotState& robot_state,
    const Eigen::VectorXd& robot_plan_next_conf) {
  std::scoped_lock<std::mutex> lock {robot_data_mutex_};
  robot_data_.robot_state = robot_state;
  robot_data_.robot_plan_next_conf = robot_plan_next_conf;
  robot_data_.has_robot_data = true;
}

bool CommunicationInterface::GetPauseStatus() {
  return pause_data_.paused;  // this is atomic
}

void CommunicationInterface::SetPauseStatus(bool paused) {
  pause_data_.paused = paused;  // this is atomic
}

void CommunicationInterface::PublishPlanComplete(
    const int64_t& plan_utime, bool success, std::string plan_status_string) {
  std::string log_msg {
      fmt::format("CommInterface:PublishPlanComplete: plan {} {}", plan_utime,
                  success ? "successful" : "failed")};
  if (!success) {
    log_msg += fmt::format(", error status: {}", plan_status_string);
  }
  dexai::log()->info(log_msg);
  new_plan_buffer_.plan.release();
  PublishTriggerToChannel(plan_utime, params_.lcm_plan_complete_channel,
                          success, plan_status_string);
}

void CommunicationInterface::PublishDriverStatus(
    bool success, std::string driver_status_string) {
  PublishTriggerToChannel(utils::get_current_utime(),
                          lcm_driver_status_channel_, success,
                          driver_status_string);
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
    // TODO(@anyone): make pause status part of the robot status
    PublishPauseStatus();
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
  if (std::unique_lock<std::mutex> lock {robot_data_mutex_};
      robot_data_.has_robot_data) {
    drake::lcmt_iiwa_status franka_status {
        utils::ConvertToLcmStatus(robot_data_)};
    // publish data over lcm
    franka::RobotMode current_mode {robot_data_.robot_state.robot_mode};
    robot_data_.has_robot_data = false;
    lock.unlock();

    lcm_.publish(params_.lcm_status_channel, &franka_status);
    PublishBoolToChannel(franka_status.utime, lcm_user_stop_channel_,
                         current_mode == franka::RobotMode::kUserStopped);
    PublishBoolToChannel(franka_status.utime, lcm_brakes_locked_channel_,
                         current_mode == franka::RobotMode::kOther);
  }
}

void CommunicationInterface::PublishPauseStatus() {
  robot_msgs::trigger_t msg;
  msg.utime = utils::get_current_utime();
  std::unique_lock<std::mutex> lock(pause_mutex_);
  msg.success = pause_data_.paused;
  msg.message = "";
  if (pause_data_.paused) {
    for (auto elem : pause_data_.pause_sources) {
      msg.message.append(elem);
      msg.message.append(",");
    }
  }
  lock.unlock();
  lcm_.publish(lcm_pause_status_channel_, &msg);
}

void CommunicationInterface::PublishTriggerToChannel(
    int64_t utime, std::string_view lcm_channel, bool success,
    std::string_view message) {
  robot_msgs::trigger_t msg;
  msg.utime = utime;
  msg.success = success;
  msg.message = message.data();
  lcm_.publish(lcm_channel.data(), &msg);
}

void CommunicationInterface::PublishBoolToChannel(int64_t utime,
                                                  std::string_view lcm_channel,
                                                  bool data) {
  robot_msgs::bool_t msg;
  msg.utime = utime;
  msg.data = data;
  lcm_.publish(lcm_channel.data(), &msg);
}

franka::RobotMode CommunicationInterface::GetRobotMode() {
  std::scoped_lock<std::mutex> lock {robot_data_mutex_};
  return robot_data_.robot_state.robot_mode;
}

bool CommunicationInterface::CanReceiveCommands(
    const franka::RobotMode& current_mode) {
  switch (current_mode) {
    case franka::RobotMode::kOther:
      dexai::log()->error("CanReceiveCommands: Wrong mode: {}!",
                          utils::RobotModeToString(current_mode));
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
      dexai::log()->error("CanReceiveCommands: Wrong mode!");
      return false;
    case franka::RobotMode::kReflex:
      dexai::log()->warn(
          "CanReceiveCommands: allowing to receive commands while in "
          "Reflex mode, but this needs more testing!",
          utils::RobotModeToString(current_mode));
      return true;
    case franka::RobotMode::kUserStopped:
      dexai::log()->error("CanReceiveCommands: Wrong mode: {}!",
                          utils::RobotModeToString(current_mode));
      return false;
    case franka::RobotMode::kAutomaticErrorRecovery:
      dexai::log()->error("CanReceiveCommands: Wrong mode: {}!",
                          utils::RobotModeToString(current_mode));
      return false;
    default:
      dexai::log()->error("CanReceiveCommands: Mode unknown!");
      return false;
  }
}

void CommunicationInterface::HandlePlan(
    const ::lcm::ReceiveBuffer*, const std::string&,
    const lcmtypes::robot_spline_t* robot_spline) {
  dexai::log()->info("CommInterface:HandlePlan: Received new plan {}",
                     robot_spline->utime);

  const auto current_mode {GetRobotMode()};

  //$ check if in proper mode to receive commands
  if (!CanReceiveCommands(current_mode)) {
    const auto err_msg {fmt::format(
        "Discarding plan with utime: {}, robot is in wrong mode: {}!",
        robot_spline->utime, utils::RobotModeToString(current_mode))};
    dexai::log()->error("CommInterface:HandlePlan: {}", err_msg);
    PublishDriverStatus(false, err_msg);
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
  // publish confirmation that plan was received with same utime
  PublishTriggerToChannel(new_plan_buffer_.utime,
                          params_.lcm_plan_received_channel);
  dexai::log()->info(
      "CommInterface:HandlePlan: "
      "Published confirmation of received plan {}",
      robot_spline->utime);

  PPType piecewise_polynomial {
      decodePiecewisePolynomial(robot_spline->piecewise_polynomial)};

  if (piecewise_polynomial.get_number_of_segments() < 1) {
    dexai::log()->error(
        "CommInterface:HandlePlan: "
        "Discarding plan, invalid piecewise polynomial.");
    return;
  }

  dexai::log()->info(
      "CommInterface:HandlePlan: "
      "plan {}, start time: {:.2f}, end time: {:.2f}",
      robot_spline->utime, piecewise_polynomial.start_time(),
      piecewise_polynomial.end_time());

  // Start position == goal position check
  // TODO(@anyone): change to append initial position and respline here
  Eigen::VectorXd commanded_start =
      piecewise_polynomial.value(piecewise_polynomial.start_time());

  auto q = this->GetRobotState().q;
  // TODO(@anyone): move this check to franka plan runner
  Eigen::VectorXd q_eigen = utils::v_to_e(utils::ArrayToVector(q));

  auto max_angular_distance =
      utils::max_angular_distance(commanded_start, q_eigen);
  if (max_angular_distance > params_.kMediumJointDistance) {
    // discard the plan if we are too far away from current robot start
    Eigen::VectorXd joint_delta = q_eigen - commanded_start;
    dexai::log()->error(
        "CommInterface:HandlePlan: "
        "discarding plan {}, mismatched start position with delta: {}.",
        robot_spline->utime, joint_delta.transpose());
    new_plan_buffer_.plan.release();
    lock.unlock();
    PublishPlanComplete(robot_spline->utime, false /*  = failed*/,
                        "mismatched_start_position");
    return;
  }
  new_plan_buffer_.plan = std::make_unique<PPType>(piecewise_polynomial);
  lock.unlock();
  dexai::log()->info(
      "CommInterface::HandlePlan: populated buffer with new plan {}",
      new_plan_buffer_.utime);
  dexai::log()->debug("CommInterface:HandlePlan: Finished!");
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
      dexai::log()->error(
          "CommInterface:HandlePause: Received cancel plan request!");
      cancel_plan_requested_ = true;
      return;
    }
    case PauseCommandType::PAUSE:
      dexai::log()->warn(
          "CommInterface:HandlePause: Received pause command from '{}'",
          source);
      if (pause_data_.pause_sources.insert(source).second == false) {
        dexai::log()->warn(
            "CommInterface:HandlePause: Already paused by source: '{}'",
            source);
      }
      break;
    case PauseCommandType::CONTINUE:
      dexai::log()->warn(
          "CommInterface:HandlePause: Received continue command from "
          "'{}'",
          source);
      if (pause_data_.pause_sources.find(source)
          != pause_data_.pause_sources.end()) {
        pause_data_.pause_sources.erase(source);
      } else {
        dexai::log()->warn(
            "Unpausing command rejected: No matching "
            "pause command by source: '{}'",
            source);
      }
      break;
    default:
      dexai::log()->error(
          "Pause command rejected: Unknown pause command type from '{}'",
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
  // bool desired_state {cmd_msg->data};
  auto desired_event {cmd_msg->source};

  // simulate control exception
  if (desired_event == "control_exception") {
    dexai::log()->error(
        "CommInterface:HandleSimDriverEventTrigger: received command "
        "to simulate control exception!");
    sim_control_exception_triggered_ = true;
    return;
  } else {
    dexai::log()->error(
        "CommInterface:HandleSimDriverEventTrigger: Unrecognized "
        "trigger {}!",
        desired_event);
  }

  // TODO(@andrey): use this for u-stop, check desired state for on/off
}