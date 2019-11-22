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

  pause_data_.paused_ = false;  // not paused at start

  lcm_driver_status_channel_ = params_.robot_name + "_DRIVER_STATUS";
  lcm_pause_status_channel_ = params_.robot_name + "_PAUSE_STATUS";

  robot_plan_.has_plan_data_ = false;
  robot_plan_.plan_.release();
  robot_plan_.utime = -1;

  momap::log()->info("Plan channel: {}", params_.lcm_plan_channel);
  momap::log()->info("Stop channel: {}", params_.lcm_stop_channel);
  momap::log()->info("Plan received channel: {}",
                     params_.lcm_plan_received_channel);
  momap::log()->info("Plan complete channel: {}",
                     params_.lcm_plan_complete_channel);
  momap::log()->info("Status channel: {}", params_.lcm_status_channel);
};

void CommunicationInterface::StartInterface() {
  // start LCM threads; independent of sim vs. real robot
  running_ = true;
  momap::log()->info("Start LCM threads");
  lcm_publish_status_thread_ =
      std::thread(&CommunicationInterface::PublishLcmAndPauseStatus, this);
  lcm_handle_thread_ = 
      std::thread(&CommunicationInterface::HandleLcm, this);
};

void CommunicationInterface::StopInterface() {
  momap::log()->info(
      "CommunicationInterface::StopInterface: Before LCM thread join");
  // clean-up threads if they're still alive.
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
};

bool CommunicationInterface::HasNewPlan() {
  return robot_plan_.has_plan_data_;  // is atomic
}

void CommunicationInterface::TakeOverPlan(std::unique_ptr<PPType>& plan) {
  std::lock_guard<std::mutex> lock(plan_mutex_);
  plan = std::move(robot_plan_.plan_);
  robot_plan_.has_plan_data_ = false;
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
  if (robot_data_mutex_.try_lock()) {
    robot_data_.has_robot_data_ = true;
    robot_data_.robot_state = robot_state;
    robot_data_mutex_.unlock();
  }
}

bool CommunicationInterface::GetPauseStatus() {
  return pause_data_.paused_;  // this is atomic
}

void CommunicationInterface::SetPauseStatus(bool paused) {
  std::lock_guard<std::mutex> lock(pause_mutex_);
  pause_data_.paused_ = paused;
  // QueuedCmd();
}

void CommunicationInterface::PublishPlanComplete(const int64_t& end_time_us) {
  robot_plan_.plan_.release();
  robot_plan_.has_plan_data_ = false;
  PublishTriggerToChannel(robot_plan_.utime, params_.lcm_plan_complete_channel);
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
  while (running_) {
    // Try to lock data to avoid read write collisions.
    if (robot_data_mutex_.try_lock()) {
      if (robot_data_.has_robot_data_) {
        drake::lcmt_iiwa_status franka_status =
            ConvertToLcmStatus(robot_data_.robot_state);
        robot_data_mutex_.unlock();
        // publish data over lcm
        lcm_.publish(params_.lcm_status_channel, &franka_status);
        robot_data_.has_robot_data_ = false;
      }
      else {
        robot_data_mutex_.unlock();
      }
    } else {
      momap::log()->warn(
          "CommunicationInterface::PublishLcmAndPauseStatus:"
          " Failed to get a lock on the robot_data_mutex_");
    }

    // TODO @rkk: make pause status part of the robot status
    PublishPauseStatus();
    // Sleep to achieve the desired print rate.
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(1000.0 / lcm_publish_rate_)));
  }
}

void CommunicationInterface::PublishPauseStatus() {
  robot_msgs::trigger_t msg;
  msg.utime = dru::get_current_utime();
  if (pause_mutex_.try_lock()) {
    msg.success = pause_data_.paused_;
    msg.message = "";
    if (pause_data_.paused_) {
      for (auto elem : pause_data_.stop_set_) {
        msg.message.append(elem);
        msg.message.append(",");
      }
      // momap::log()->info("PublishPauseStatus with msg.message: {}",
      // msg.message);
    }
    pause_mutex_.unlock();
    lcm_.publish(lcm_pause_status_channel_, &msg);
  } else {
    momap::log()->warn(
        "CommunicationInterface::PublishPauseStatus:"
        " Failed to get a lock on the pause_mutex_");
  }
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

  momap::log()->info("Current mode: {}", RobotModeToString(current_mode));

  if (current_mode == franka::RobotMode::kIdle) {
    return true;
  }
  if (current_mode == franka::RobotMode::kMove) {
    momap::log()->warn("Allowing to receive command while in {}"
        ", this needs testing!", RobotModeToString(current_mode));
    return true;
  }
  if (current_mode == franka::RobotMode::kReflex) {
    momap::log()->warn("Allowing to receive command while in {},"
        " this needs testing!", RobotModeToString(current_mode));
    return true;
  }

  momap::log()->error("CanReceiveCommands: Wrong mode!");
  return false;
}

void CommunicationInterface::HandlePlan(const ::lcm::ReceiveBuffer*,
                                        const std::string&,
                                        const lcmtypes::robot_spline_t* rst) {
  momap::log()->info("New plan received.");

  //$ check if in proper mode to receive commands
  if (!CanReceiveCommands()) {
    momap::log()->error("Discarding plan, in wrong mode!");
    return;
  }

  // plan_mutex_.lock();
  while (!plan_mutex_.try_lock()) {
    momap::log()->warn(
        "trying to get a lock on the plan_mutex_. Sleeping 1 ms and trying "
        "again.");
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1.0)));
  }

  momap::log()->info("utime: {}", rst->utime);
  robot_plan_.utime = rst->utime;
  //$ publish confirmation that plan was received with same utime
  PublishTriggerToChannel(robot_plan_.utime, params_.lcm_plan_received_channel);
  momap::log()->info("Published confirmation of received plan");

  PPType piecewise_polynomial = TrajectorySolver::RobotSplineTToPPType(*rst);

  if (piecewise_polynomial.get_number_of_segments() < 1) {
    momap::log()->info("Discarding plan, invalid piecewise polynomial.");
    plan_mutex_.unlock();
    return;
  }

  momap::log()->info("start time: {}", piecewise_polynomial.start_time());
  momap::log()->info("end time: {}", piecewise_polynomial.end_time());

  // Start position == goal position check
  // TODO: add end position==goal position check (upstream)
  // TODO: change to append initial position and respline here
  Eigen::VectorXd commanded_start =
      piecewise_polynomial.value(piecewise_polynomial.start_time());

  auto q = this->GetRobotState().q;
  // TODO @rkk: move this check to franka plan runner...
  for (int joint = 0; joint < rst->dof; joint++) {
    if (!dru::EpsEq(commanded_start(joint), q[joint],
                    params_.kMediumJointDistance)) {
      momap::log()->info("Discarding plan, mismatched start position.");
      robot_plan_.has_plan_data_ = false;
      robot_plan_.plan_.release();
      plan_mutex_.unlock();
      return;
    }
  }

  robot_plan_.plan_.release();
  robot_plan_.plan_.reset(&piecewise_polynomial);
  robot_plan_.has_plan_data_ = true;

  momap::log()->warn("Finished Handle Plan!");
  plan_mutex_.unlock();
};

void CommunicationInterface::HandlePause(const ::lcm::ReceiveBuffer*,
                                         const std::string&,
                                         const robot_msgs::pause_cmd* msg) {
  std::lock_guard<std::mutex> lock(pause_mutex_);
  // check if pause command received:
  if (msg->data) {
    momap::log()->info("Received pause from {}", msg->source);
    if (pause_data_.stop_set_.insert(msg->source).second == false) {
      momap::log()->warn(
          "CommunicationInterface::HandlePause: "
          "Already paused by source: {}",
          msg->source);
    }
  }
  // check if unpause command received
  else if (!msg->data) {
    momap::log()->info("Received unpause from {}", msg->source);
    if (pause_data_.stop_set_.find(msg->source) !=
        pause_data_.stop_set_.end()) {
      pause_data_.stop_set_.erase(msg->source);
    } else {
      momap::log()->info(
          "Unpausing command rejected: No matching "
          "pause command by source: {}'",
          msg->source);
    }
  }
  if (pause_data_.stop_set_.size() == 0) {
    pause_data_.paused_ = false;
  } else {
    pause_data_.paused_ = true;
  }
}
