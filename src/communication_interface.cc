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

#include "communication_interface.h"

#include "drac_util_io.h"  // for get_current_utime
#include "drake/lcmt_iiwa_status.hpp"
#include "franka_driver_utils.h"     // ConvertToLcmStatus
#include "robot_msgs/pause_cmd.hpp"  // for pause_cmd
#include "robot_msgs/trigger_t.hpp"  // for trigger_t
#include "trajectory_solver.h"       // for TrajectorySolver

#include <chrono>  // for steady_clock, for duration

using namespace franka_driver;
namespace dru = dracula_utils;

CommunicationInterface::CommunicationInterface(
    const parameters::Parameters params, double lcm_publish_rate)
    : params_(params),
      lcm_(params_.lcm_url),
      lcm_publish_rate_(lcm_publish_rate) {
  lcm_.subscribe(params_.lcm_plan_channel, &CommunicationInterface::HandlePlan,
                 this);
  lcm_.subscribe(params_.lcm_stop_channel, &CommunicationInterface::HandlePause,
                 this);

  // TODO @rkk: define this in parameters file?
  lcm_driver_status_channel_ = params_.robot_name + "_DRIVER_STATUS";
  // TODO @rkk: remove this status channel by combining it
  // with the robot status channel:
  lcm_pause_status_channel_ = params_.robot_name + "_PAUSE_STATUS";

  momap::log()->info("Plan channel: {}", params_.lcm_plan_channel);
  momap::log()->info("Stop channel: {}", params_.lcm_stop_channel);
  momap::log()->info("Plan received channel: {}",
                     params_.lcm_plan_received_channel);
  momap::log()->info("Plan complete channel: {}",
                     params_.lcm_plan_complete_channel);
  momap::log()->info("Status channel: {}", params_.lcm_status_channel);
  momap::log()->info("Driver status channel: {}", lcm_driver_status_channel_);
  momap::log()->info("Pause status channel: {}", lcm_pause_status_channel_);
};

void CommunicationInterface::ResetData() {
  std::unique_lock<std::mutex> lock_data(robot_data_mutex_);
  robot_data_.has_robot_data_ = false;
  robot_data_.robot_state = franka::RobotState();
  lock_data.unlock();

  // initialize plan as empty:
  std::unique_lock<std::mutex> lock_plan(robot_plan_mutex_);
  robot_plan_.has_plan_data_ = false;  // no new plan
  robot_plan_.plan_.release();         // unique ptr points to no plan
  robot_plan_.utime = -1;              // utime set to -1 at start
  lock_plan.unlock();

  // initialize pause as false:
  std::unique_lock<std::mutex> lock_pause(pause_mutex_);
  pause_data_.paused_ = false;             // not paused at start
  pause_data_.pause_sources_set_.clear();  // no pause sources at start
  lock_pause.unlock();
}

void CommunicationInterface::StartInterface() {
  // Initialize data as empty for exchange with robot driver
  ResetData();
  // start LCM threads; independent of sim vs. real robot
  momap::log()->info("Start LCM threads");
  running_ = true;  // sets the lcm threads to active.
  lcm_publish_status_thread_ =
      std::thread(&CommunicationInterface::PublishLcmAndPauseStatus, this);
  lcm_handle_thread_ = std::thread(&CommunicationInterface::HandleLcm, this);
};

void CommunicationInterface::StopInterface() {
  momap::log()->info(
      "CommunicationInterface::StopInterface: Before LCM threads join");
  // clean-up threads if they're still alive.
  running_ = false;  // sets the lcm threads to inactive.
  while (!lcm_handle_thread_.joinable() ||
         !lcm_publish_status_thread_.joinable()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    momap::log()->warn(
        "CommunicationInterface::StopInterface: Waiting for LCM threads to be "
        "joinable...");
  }
  lcm_publish_status_thread_.join();
  lcm_handle_thread_.join();
  momap::log()->info(
      "CommunicationInterface::StopInterface: After LCM thread join");
  ResetData();
};

bool CommunicationInterface::HasNewPlan() {
  return robot_plan_.has_plan_data_;  // is atomic
}

void CommunicationInterface::TakePlan(std::unique_ptr<PPType>& plan,
                                      int64_t& plan_utime) {
  std::lock_guard<std::mutex> lock(robot_plan_mutex_);
  if (!HasNewPlan() || !(robot_plan_.plan_)) {
    throw std::runtime_error("TakePlan: No plan to take over!");
  }
  robot_plan_.has_plan_data_ = false;
  plan = std::move(robot_plan_.plan_);
  if (!plan) {
    throw std::runtime_error("TakePlan: Failed to take plan!");
  }
  plan_utime = robot_plan_.utime;
}

franka::RobotState CommunicationInterface::GetRobotState() {
  std::lock_guard<std::mutex> lock(robot_data_mutex_);
  return robot_data_.robot_state;
}

void CommunicationInterface::SetRobotState(
    const franka::RobotState& robot_state) {
  std::lock_guard<std::mutex> lock(robot_data_mutex_);
  robot_data_.has_robot_data_ = true;
  robot_data_.robot_state = robot_state;
}

void CommunicationInterface::TryToSetRobotState(
    const franka::RobotState& robot_state) {
  std::unique_lock<std::mutex> lock(robot_data_mutex_, std::defer_lock);
  if (lock.try_lock()) {
    robot_data_.has_robot_data_ = true;
    robot_data_.robot_state = robot_state;
    lock.unlock();
  }
}

bool CommunicationInterface::GetPauseStatus() {
  return pause_data_.paused_;  // this is atomic
}

void CommunicationInterface::SetPauseStatus(bool paused) {
  pause_data_.paused_ = paused;  // this is atomic
}

void CommunicationInterface::PublishPlanComplete(
    const int64_t& plan_utime, bool success, std::string plan_status_string) {
  robot_plan_.plan_.release();
  robot_plan_.has_plan_data_ = false;
  PublishTriggerToChannel(plan_utime, params_.lcm_plan_complete_channel,
                          success, plan_status_string);
}

void CommunicationInterface::PublishDriverStatus(
    bool success, std::string driver_status_string) {
  PublishTriggerToChannel(dru::get_current_utime(), lcm_driver_status_channel_,
                          success, driver_status_string);
}

void CommunicationInterface::HandleLcm() {
  while (running_) {
    lcm_.handleTimeout(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void CommunicationInterface::PublishLcmAndPauseStatus() {
  auto desired_wait =
      std::chrono::milliseconds(static_cast<int>(1000.0 / lcm_publish_rate_));
  while (running_) {
    auto time_start = std::chrono::steady_clock::now();
    PublishRobotStatus();
    // TODO @rkk: make pause status part of the robot status
    PublishPauseStatus();
    // Sleep dynamically to achieve the desired print rate.
    auto time_end = std::chrono::steady_clock::now();
    auto time_elapsed = time_end - time_start;
    auto remaining_wait = desired_wait - time_elapsed;

    if (remaining_wait < std::chrono::seconds(0)) {
      std::chrono::milliseconds time_elapsed_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(time_elapsed);
      momap::log()->warn(
          "CommunicationInterface::PublishLcmAndPauseStatus:"
          " publish took too long with {} ms > {} ms!",
          time_elapsed_ms.count(), 1000.0 / lcm_publish_rate_);
    }
    std::this_thread::sleep_for(remaining_wait);
  }
}

void CommunicationInterface::PublishRobotStatus() {
  // Try to lock data to avoid read write collisions.
  std::unique_lock<std::mutex> lock(robot_data_mutex_);
  if (robot_data_.has_robot_data_) {
    drake::lcmt_iiwa_status franka_status =
        ConvertToLcmStatus(robot_data_.robot_state);
    // publish data over lcm
    robot_data_.has_robot_data_ = false;
    lock.unlock();
    lcm_.publish(params_.lcm_status_channel, &franka_status);
  } else {
    lock.unlock();
  }
}

void CommunicationInterface::PublishPauseStatus() {
  robot_msgs::trigger_t msg;
  msg.utime = dru::get_current_utime();
  std::unique_lock<std::mutex> lock(pause_mutex_);
  msg.success = pause_data_.paused_;
  msg.message = "";
  if (pause_data_.paused_) {
    for (auto elem : pause_data_.pause_sources_set_) {
      msg.message.append(elem);
      msg.message.append(",");
    }
  }
  lock.unlock();
  lcm_.publish(lcm_pause_status_channel_, &msg);
}

void CommunicationInterface::PublishTriggerToChannel(int64_t utime,
                                                     std::string lcm_channel,
                                                     bool success,
                                                     std::string message) {
  robot_msgs::trigger_t msg;
  msg.utime = utime;
  msg.success = success;
  msg.message = message;
  lcm_.publish(lcm_channel, &msg);
}

bool CommunicationInterface::CanReceiveCommands() {
  std::unique_lock<std::mutex> lock(robot_data_mutex_);
  franka::RobotMode current_mode = robot_data_.robot_state.robot_mode;
  lock.unlock();

  switch (current_mode) {
    case franka::RobotMode::kOther:
      momap::log()->error("CanReceiveCommands: Wrong mode: {}!",
                          RobotModeToString(current_mode));
      return false;
    case franka::RobotMode::kIdle:
      return true;
    case franka::RobotMode::kMove:
      momap::log()->warn(
          "CanReceiveCommands: "
          "Allowing to receive commands while in {}"
          ", but this needs more testing!",
          RobotModeToString(current_mode));
      return true;
    case franka::RobotMode::kGuiding:
      momap::log()->error("CanReceiveCommands: Wrong mode!");
      return false;
    case franka::RobotMode::kReflex:
      momap::log()->warn(
          "CanReceiveCommands: "
          "Allowing to receive commands while in {},"
          " but this needs more testing!",
          RobotModeToString(current_mode));
      return true;
    case franka::RobotMode::kUserStopped:
      momap::log()->error("CanReceiveCommands: Wrong mode: {}!",
                          RobotModeToString(current_mode));
      return false;
    case franka::RobotMode::kAutomaticErrorRecovery:
      momap::log()->error("CanReceiveCommands: Wrong mode: {}!",
                          RobotModeToString(current_mode));
      return false;
    default:
      momap::log()->error("CanReceiveCommands: Mode unknown!");
      return false;
  }
}

void CommunicationInterface::HandlePlan(
    const ::lcm::ReceiveBuffer*, const std::string&,
    const lcmtypes::robot_spline_t* robot_spline) {
  momap::log()->info("CommunicationInterface::HandlePlan: Received new plan {}",
                     robot_spline->utime);

  //$ check if in proper mode to receive commands
  if (!CanReceiveCommands()) {
    momap::log()->error(
        "CommunicationInterface::HandlePlan: "
        "Discarding plan with utime: {},"
        " robot is in wrong mode!",
        robot_spline->utime);
    return;
  }

  std::unique_lock<std::mutex> lock(robot_plan_mutex_, std::defer_lock);
  while (!lock.try_lock()) {
    momap::log()->warn(
        "CommunicationInterface::HandlePlan: trying to get a lock on the "
        "robot_plan_mutex_. Sleeping 1 ms and trying again.");
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1.0)));
  }

  robot_plan_.utime = robot_spline->utime;
  //$ publish confirmation that plan was received with same utime
  // TODO @rkk: move this to later in the function...
  PublishTriggerToChannel(robot_plan_.utime, params_.lcm_plan_received_channel);
  momap::log()->info(
      "CommunicationInterface::HandlePlan: "
      "Published confirmation of received plan {}",
      robot_spline->utime);

  PPType piecewise_polynomial =
      TrajectorySolver::RobotSplineTToPPType(*robot_spline);

  if (piecewise_polynomial.get_number_of_segments() < 1) {
    momap::log()->error(
        "CommunicationInterface::HandlePlan: "
        "Discarding plan, invalid piecewise polynomial.");
    lock.unlock();
    return;
  }

  momap::log()->info(
      "CommunicationInterface::HandlePlan: "
      "plan {}: start time: {}, end time: {}",
      robot_spline->utime, piecewise_polynomial.start_time(),
      piecewise_polynomial.end_time());

  // Start position == goal position check
  // TODO: add end position==goal position check (upstream)
  // TODO: change to append initial position and respline here
  auto commanded_start =
      piecewise_polynomial.value(piecewise_polynomial.start_time());

  auto q = this->GetRobotState().q;
  // TODO @rkk: move this check to franka plan runner...
  auto q_eigen = dru::v_to_e(ArrayToVector(q));

  auto max_angular_distance = dru::max_angular_distance(commanded_start, q_eigen);
  if (max_angular_distance > GetParameters()->kMediumJointDistance)
  {
      // discard the plan if we are too far away from current robot start
      Eigen::VectorXd joint_delta = q_eigen - commanded_start; 
      momap::log()->error(
          "CommunicationInterface::HandlePlan: "
          "Discarding plan {}, mismatched start position with delta: {}.",
          robot_spline->utime, joint_delta.transpose());
      robot_plan_.has_plan_data_ = false;
      robot_plan_.plan_.release();
      lock.unlock();
      return;
  }

  // plan is valid, so release old one first
  robot_plan_.has_plan_data_ = false;
  robot_plan_.plan_.release();
  // assign new plan:
  robot_plan_.plan_ = std::make_unique<PPType>(piecewise_polynomial);
  robot_plan_.has_plan_data_ = true;
  lock.unlock();

  momap::log()->debug("CommunicationInterface::HandlePlan: Finished!");
};

void CommunicationInterface::HandlePause(
    const ::lcm::ReceiveBuffer*, const std::string&,
    const robot_msgs::pause_cmd* pause_cmd_msg) {
  std::lock_guard<std::mutex> lock(pause_mutex_);
  // check if paused = true or paused = false was received:
  bool paused = pause_cmd_msg->data;
  if (paused) {
    momap::log()->warn(
        "CommunicationInterface::HandlePause: Received 'pause = true' from {}",
        pause_cmd_msg->source);
    if (pause_data_.pause_sources_set_.insert(pause_cmd_msg->source).second ==
        false) {
      momap::log()->warn(
          "CommunicationInterface::HandlePause: "
          "Already paused by source: {}",
          pause_cmd_msg->source);
    }
  } else {
    momap::log()->warn(
        "CommunicationInterface::HandlePause: Received 'pause = false' from {}",
        pause_cmd_msg->source);
    if (pause_data_.pause_sources_set_.find(pause_cmd_msg->source) !=
        pause_data_.pause_sources_set_.end()) {
      pause_data_.pause_sources_set_.erase(pause_cmd_msg->source);
    } else {
      momap::log()->warn(
          "Unpausing command rejected: No matching "
          "pause command by source: {}'",
          pause_cmd_msg->source);
    }
  }

  // if the set of pause sources is empty, then
  // the robot is not paused anymore:
  if (pause_data_.pause_sources_set_.size() == 0) {
    pause_data_.paused_ = false;
  } else {
    pause_data_.paused_ = true;
  }
}
