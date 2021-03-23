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

/// @file franka_plan_runner.cc
///
/// franka_plan_runner is designed to wait for LCM messages containing
/// a robot_spline_t message, and then execute the plan on a franka arm
/// (also reporting via LCM lcmt_franka_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

#include "driver/franka_plan_runner.h"

#include <algorithm>  // for min
#include <cmath>      // for exp
#include <vector>     // for vector

#include <drake/lcmt_iiwa_status.hpp>

#include "franka/exception.h"  // for Exception, ControlException
#include "franka/robot.h"      // for Robot
#include "utils/util_math.h"

using franka_driver::FrankaPlanRunner;
using utils::RobotStatus;

FrankaPlanRunner::FrankaPlanRunner(const RobotParameters& params)
    : dof_ {FRANKA_DOF},
      safety_off_ {utils::getenv_var("FRANKA_SAFETY_OFF") == "true"},
      params_ {params},
      ip_addr_ {params.robot_ip},
      is_sim_ {ip_addr_ == "192.168.1.1"},
      status_ {RobotStatus::Uninitialized} {
  // setup communication interface
  comm_interface_ =
      std::make_unique<CommunicationInterface>(params_, lcm_publish_rate_);
  max_accels_ = params.robot_max_accelerations;

  assert(!params_.urdf_filepath.empty()
         && "FrankaPlanRunner ctor: bad params_.urdf_filepath");
  CONV_SPEED_THRESHOLD = Eigen::VectorXd(dof_);  // segfault without reallocate
  CONV_SPEED_THRESHOLD << 0.007, 0.007, 0.007, 0.007, 0.007, 0.007, 0.007;

  // Create a ConstraintSolver, which creates a geometric model from parameters
  // and URDF(s) and keeps it in a fully owned MultiBodyPlant.
  // Once the CS exists, we get robot and scene geometry from it, not from
  // Parameters, which cannot and should not be updated (keep them const).
  constraint_solver_ = std::make_unique<ConstraintSolver>(&params_);

  joint_limits_ = constraint_solver_->GetJointLimits();
  dexai::log()->info("Lower Joint limits URDF: {}",
                     joint_limits_.col(0).transpose());
  dexai::log()->info("Lower Joint limits YAML: {}",
                     params_.robot_low_joint_limits.transpose());
  dexai::log()->info("Upper Joint limits URDF: {}",
                     joint_limits_.col(1).transpose());
  dexai::log()->info("Upper Joint limits YAML: {}",
                     params_.robot_high_joint_limits.transpose());

  start_conf_franka_ = Eigen::VectorXd::Zero(dof_);
  start_conf_plan_ = Eigen::VectorXd::Zero(dof_);
  next_conf_plan_ = Eigen::VectorXd::Zero(dof_);
  joint_pos_offset_ = Eigen::VectorXd::Zero(dof_);

  try {
    cnpy::NpyArray joint_pos_offset_data =
        cnpy::npy_load("joint_pos_offset.npy");
    const std::array<double, FRANKA_DOF>& joint_pos_offset_array {
        *(joint_pos_offset_data.data<std::array<double, FRANKA_DOF>>())};
    const auto joint_pos_offset_v {
        utils::ArrayToVector(joint_pos_offset_array)};
    joint_pos_offset_ = utils::v_to_e(joint_pos_offset_v);
    is_joint_pos_offset_available_ = true;
    dexai::log()->info("Loaded joint position offsets: {}",
                       joint_pos_offset_.transpose());
  } catch (const std::runtime_error& error) {
    log()->info(
        "Could not load joint position offset from file. Setting offsets to "
        "zero...");
    is_joint_pos_offset_available_ = false;
  }

  log()->warn("Collision Safety {}", safety_off_ ? "OFF" : "ON");

  if (safety_off_) {
    upper_torque_threshold_ = kHighTorqueThreshold;
    upper_force_threshold_ = kHighForceThreshold;
  } else {
    upper_torque_threshold_ = kMediumTorqueThreshold;
    upper_force_threshold_ = kMediumForceThreshold;
  }
}

int FrankaPlanRunner::Run() {
  comm_interface_->StartInterface();

  int return_value = 1;  //
  if (is_sim_) {
    return_value = RunSim();
  } else {
    return_value = RunFranka();
  }

  comm_interface_->StopInterface();

  status_ = RobotStatus::Uninitialized;

  return return_value;
}

franka::RobotMode FrankaPlanRunner::GetRobotMode() const {
  franka::RobotMode current_mode;
  robot_->read([&current_mode](const franka::RobotState& robot_state) {
    current_mode = robot_state.robot_mode;
    return false;
  });
  dexai::log()->debug("GetRobotMode: Franka is in mode: {}",
                      utils::RobotModeToString(current_mode));
  return current_mode;
}

int FrankaPlanRunner::RunFranka() {
  {  // connection and mode checking and handling
    // attempt connection to robot and read current mode
    // and if it fails, keep trying instead of exiting the program
    bool connection_established {};
    do {  // do-while(!connection_established) loop, execute once first
      try {
        robot_ = std::make_unique<franka::Robot>(ip_addr_);
      } catch (franka::Exception const& e) {
        // probably something wrong with networking
        auto err_msg {fmt::format("Franka connection error: {}", e.what())};
        dexai::log()->error("RunFranka: {}", err_msg);
        comm_interface_->PublishDriverStatus(false, err_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        continue;
      }

      auto current_mode {GetRobotMode()};
      if (auto t_now {std::chrono::steady_clock::now()};
          current_mode == franka::RobotMode::kReflex) {
        // if in reflex mode, attempt automatic error recovery
        dexai::log()->warn(
            "RunFranka: robot is in Reflex mode at startup, trying "
            "automaticErrorRecovery...");
        try {
          robot_->automaticErrorRecovery();
          dexai::log()->info(
              "RunFranka: automaticErrorRecovery succeeded, out of Reflex mode,"
              " now in mode: {}.",
              utils::RobotModeToString(GetRobotMode()));
        } catch (const franka::ControlException& ce) {
          comm_interface_->PublishDriverStatus(false, ce.what());
          dexai::log()->warn(
              "RunFranka: control exception in initialisation during automatic "
              "error recovery for Reflex mode: {}.",
              ce.what());
        }
      } else if (current_mode != franka::RobotMode::kIdle) {
        auto err_msg {fmt::format("robot cannot receive commands in mode: {}",
                                  utils::RobotModeToString(current_mode))};
        comm_interface_->PublishDriverStatus(false, err_msg);
        if (t_now - t_last_main_loop_log_ >= std::chrono::seconds(1)) {
          dexai::log()->error("RunFranka: {}", err_msg);
          t_last_main_loop_log_ = t_now;
        }
      } else if (current_mode == franka::RobotMode::kUserStopped) {
        comm_interface_->PublishBoolToChannel(
            utils::get_current_utime(),
            comm_interface_->GetUserStopChannelName(), true);
        if (t_now - t_last_main_loop_log_ >= std::chrono::seconds(1)) {
          dexai::log()->error("RunFranka: robot is in User-Stopped mode");
          t_last_main_loop_log_ = t_now;
        }
      } else {  // if we got this far, we are talking to Franka and it is happy
        dexai::log()->info("RunFranka: connected to robot in {} mode",
                           utils::RobotModeToString(current_mode));
        connection_established = true;
      }
    } while (!connection_established);
  }

  try {  // initilization
    dexai::log()->info("RunFranka: setting default behavior...");
    SetDefaultBehaviorForInit();
    dexai::log()->info("RunFranka: ready.");
    comm_interface_->PublishDriverStatus(true);
    // Set collision behavior:
    SetCollisionBehaviorSafetyOn();
  } catch (const franka::Exception& ex) {
    dexai::log()->critical(
        "RunFranka: caught expection during initilization, msg: {}", ex.what());
    comm_interface_->PublishDriverStatus(false, ex.what());
    return 1;  // bad things happened.
  }

  status_ = RobotStatus::Running;  // init done, define robot as running
  bool status_has_changed {true};

  while (true) {  // main control loop
    auto new_status {comm_interface_->GetPauseStatus() ? RobotStatus::Paused
                                                       : RobotStatus::Running};
    if (new_status != status_) {  // check if status has changed
      status_has_changed = true;
      status_ = new_status;
    }

    // prevent the plan from being started if robot is not running...
    if (status_ == RobotStatus::Running && comm_interface_->HasNewPlan()) {
      dexai::log()->info(
          "RunFranka: found a new plan in buffer, attaching callback...");
      status_has_changed = true;
      try {  // Use either joint position or impedance control callback here
        // blocking
        robot_->control(std::bind(&FrankaPlanRunner::JointPositionCallback,
                                  this, std::placeholders::_1,
                                  std::placeholders::_2));
      } catch (const franka::ControlException& ce) {
        status_has_changed = true;
        if (plan_) {  // broadcast exception details over LCM
          dexai::log()->warn(
              "RunFranka: control exception during active plan "
              "{} at franka_t: {:.4f}, aborting and recovering...",
              plan_utime_, franka_time_);
          comm_interface_->PublishPlanComplete(plan_utime_, false, ce.what());
        } else {
          dexai::log()->error("RunFranka: control exception in main loop: {}.",
                              ce.what());
        }
        if (!RecoverFromControlException()) {  // plan_ is released/reset
          dexai::log()->critical(
              "RunFranka: RecoverFromControlException failed");
          comm_interface_->PublishDriverStatus(false, ce.what());
          return 1;
        }
      }
      continue;
    }
    // no new plan available in the buffer or robot isn't running
    if (comm_interface_->CancelPlanRequested()) {
      log()->error(
          "RunFranka: plan cancellation requested but there is no active "
          "plan");
      comm_interface_->ClearCancelPlanRequest();
    }
    // print out status if there's any change, or every 10 sec
    if (auto t_now {std::chrono::steady_clock::now()};
        status_has_changed
        || t_now - t_last_main_loop_log_ >= std::chrono::seconds(10)) {
      switch (status_) {
        case RobotStatus::Running:
          dexai::log()->info(
              "RunFranka: robot is running and waiting for a new plan...");
          break;
        case RobotStatus::Paused:
          dexai::log()->info(
              "RunFranka: robot is paused, waiting to get unpaused...");
          break;
        default:
          dexai::log()->error(
              "RunFranka: robot status: {}, this should not have happened",
              utils::RobotStatusToString(status_));
      }
      status_has_changed = false;  // reset
      t_last_main_loop_log_ = t_now;
    }
    // only publish robot_status, twice as fast as the lcm publish rate
    // TODO(@anyone): add a timer to be closer to lcm_publish_rate_ [Hz] * 2.
    robot_->read([this](const franka::RobotState& robot_state) {
      auto cannonical_robot_state {
          utils::ConvertToCannonical(robot_state, joint_pos_offset_)};
      // publishing cannonical values over lcm
      comm_interface_->SetRobotData(cannonical_robot_state, next_conf_plan_);
      std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int>(1000.0 / (lcm_publish_rate_ * 2.0))));
      return false;
    });
  }
  return 0;
}

bool FrankaPlanRunner::RecoverFromControlException() {
  status_ = RobotStatus::Reversing;
  dexai::log()->warn("RunFranka: turning Safety off...");
  SetCollisionBehaviorSafetyOff();
  if (auto mode {GetRobotMode()}; mode == franka::RobotMode::kUserStopped) {
    dexai::log()->warn(
        "RunFranka: Robot is in mode {}, "
        "can't run Franka's automaticErrorRecovery!",
        utils::RobotModeToString(mode));
  } else {
    dexai::log()->warn("RunFranka: Running Franka's automaticErrorRecovery...");
    robot_->automaticErrorRecovery();
    dexai::log()->warn("RunFranka: Finished Franka's automaticErrorRecovery");
  }
  dexai::log()->info("RunFranka: Turning Safety on again");
  SetCollisionBehaviorSafetyOn();
  status_ = RobotStatus::Running;
  if (plan_) {
    plan_.release();
    plan_utime_ = -1;  // reset plan utime to -1
  }
  return true;
}

int FrankaPlanRunner::RunSim() {
  dexai::log()->info("Starting sim robot and entering run loop...");
  comm_interface_->PublishDriverStatus(true);

  // first, set some parameters
  Eigen::VectorXd next_conf(dof_);  // output state
  // set robot in a starting position which is not in collision
  next_conf << -0.9577375507190063, -0.7350638062912122, 0.880988748620542,
      -2.5114236381136448, 0.6720116891296624, 1.9928838396072361,
      -1.2954019628351783;
  Eigen::VectorXd prev_conf(dof_);
  std::vector<double> vel(7, 1);
  franka::RobotState robot_state;  // internal state; mapping to franka state
  robot_state.robot_mode = franka::RobotMode::kIdle;

  franka::Duration period;
  std::chrono::milliseconds last_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());

  status_ = RobotStatus::Running;  // define robot as running at start

  while (true) {
    // The actual callback control loop runs at 1 kHz, here it's pegged to
    // the lcm_publish_rate_ to avoid excessive CPU usage in simulations.
    // The loop frequency here in no way reflects the real-world frequency.
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(1000.0 / lcm_publish_rate_)));

    std::vector<double> next_conf_vec = utils::e_to_v(next_conf);
    utils::VectorToArray(next_conf_vec, robot_state.q);
    utils::VectorToArray(next_conf_vec, robot_state.q_d);
    utils::VectorToArray(vel, robot_state.dq);

    prev_conf = next_conf.replicate(1, 1);
    next_conf = utils::v_to_e(
        utils::ArrayToVector(JointPositionCallback(robot_state, period).q));

    next_conf_vec = utils::e_to_v(next_conf);
    std::vector<double> prev_conf_vec = utils::e_to_v(prev_conf);

    for (int i {}; i < dof_; i++) {
      vel[i] = (next_conf_vec[i] - prev_conf_vec[i])
               / static_cast<double>(period.toSec());
    }

    std::chrono::milliseconds current_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
    int64_t delta_ms = int64_t((current_ms - last_ms).count());
    period = franka::Duration(delta_ms);
    last_ms = current_ms;
  }
  return 0;
}

/// Check and limit conf according to provided parameters for joint limits
bool FrankaPlanRunner::LimitJoints(Eigen::VectorXd& conf) {
  // TODO(@anyone): get limits from urdf (instead of parameter file)
  // TODO(@anyone): use eigen operator to do these operations
  bool within_limits {true};
  for (int j {}; j < conf.size(); j++) {
    if (conf(j) > joint_limits_(j, 1)) {
      conf(j) = joint_limits_(j, 1);
      within_limits = false;
    } else if (conf(j) < joint_limits_(j, 0)) {
      conf(j) = joint_limits_(j, 0);
      within_limits = false;
    }
  }
  return within_limits;
}

/// Calculate the time to advance while pausing or unpausing
/// Inputs to method have seconds as their unit.
/// Algorithm: Uses a logistic growth function:
/// t' = f - 4 / [a (e^{a*t} + 1] where
/// f = target_stop_time, t' = franka_time, t = real_time
/// Returns delta t', the period that should be incremented to franka time
double FrankaPlanRunner::TimeToAdvanceWhilePausing(double period,
                                                   double target_stop_time,
                                                   double timestep) {
  dexai::log()->debug(
      "FrankaPlanRunner:TimeToAdvanceWhilePausing: period: {:.3f} s target "
      "stop time: {:.1f} s timestep: {:.1f} s",
      period, target_stop_time, timestep);

  if (target_stop_time <= 0) {
    // target stop time will be zero if the robot was paused while not moving
    // so return early to prevent division by zero
    return 0;
  }
  double a = 2 / target_stop_time;
  double t_current = period * timestep;
  double current_franka_time =
      (target_stop_time - 4 / (a * (exp(a * t_current) + 1)));
  double t_prev = period * (timestep - 1);
  double prev_franka_time =
      (target_stop_time - 4 / (a * (exp(a * t_prev) + 1)));
  double time_to_advance {current_franka_time - prev_franka_time};
  dexai::log()->debug("FrankaPlanRunner:TimeToAdvanceWhilePausing: {:.2f} s",
                      time_to_advance);
  return time_to_advance;
}

// TODO(@anyone): rewrite this with steady_clock to make
// franka_t agree exactly with wall clock
void FrankaPlanRunner::IncreaseFrankaTimeBasedOnStatus(
    const std::array<double, 7>& vel, double period_in_seconds) {
  // get pause data from the communication interface
  auto paused = comm_interface_->GetPauseStatus();
  auto cancel_plan_requested = comm_interface_->CancelPlanRequested();
  // robot can be in four states: running, pausing, paused, unpausing

  // check if robot is supposed to be paused
  if ((paused || cancel_plan_requested) && status_ == RobotStatus::Running) {
    timestep_ = 1;             // set time step back to 1
    stop_duration_ = 0;        // reset stop duration
    stop_margin_counter_ = 0;  // reset margin counter

    // set a target_stop_time_ given the current state of the robot:
    float temp_target_stop_time_ = 0;
    for (int i = 0; i < dof_; i++) {
      // sets target stop_time in plan as
      // max(vel_i/max_accel_i), where i
      // is each joint. real world stop
      // time ~ 2x stop_time in plan
      float stop_time = fabs(vel[i] / (max_accels_[i])) * stop_delay_factor_;
      if (stop_time > temp_target_stop_time_) {
        temp_target_stop_time_ = stop_time;
      }
    }
    target_stop_time_ = temp_target_stop_time_;
    status_ = RobotStatus::Pausing;
    dexai::log()->warn(
        "IncreaseFrankaTimeBasedOnStatus: "
        "{} with target_stop_time_: {:.2f}",
        utils::RobotStatusToString(status_), target_stop_time_);
  }

  // check if robot should get unpaused
  if (!paused && status_ == RobotStatus::Paused) {
    // the duration it took to step is now used to unpause:
    timestep_ = -1 * stop_duration_;
    status_ = RobotStatus::Unpausing;
    dexai::log()->warn(
        "IncreaseFrankaTimeBasedOnStatus: "
        "{} with new timestep_: {}",
        utils::RobotStatusToString(status_), timestep_);
  }

  if (status_ == RobotStatus::Pausing) {
    double delta_franka_time = TimeToAdvanceWhilePausing(
        period_in_seconds, target_stop_time_, timestep_);
    franka_time_ += delta_franka_time;
    dexai::log()->debug(
        "IncreaseFrankaTimeBasedOnStatus: Pausing: "
        "delta_franka_time: {}",
        delta_franka_time);
    timestep_++;

    if (delta_franka_time >= period_in_seconds * params_.stop_epsilon) {
      // robot counts as "stopped" when delta_franka_time is
      // less than a fraction of period_in_seconds
      stop_duration_++;
    } else if (stop_margin_counter_ <= params_.stop_margin) {
      // margin period_in_seconds after pause before robot is
      // allowed to continue
      stop_margin_counter_ += period_in_seconds;
    } else {
      // transition to paused state
      comm_interface_->SetPauseStatus(true);
      status_ = RobotStatus::Paused;
      dexai::log()->warn(
          "IncreaseFrankaTimeBasedOnStatus: "
          "{} with delta_franka_time: {}, stop_duration_: {:.1f}"
          " and stop_margin_counter_: {:.2f}",
          utils::RobotStatusToString(status_), delta_franka_time,
          stop_duration_, stop_margin_counter_);
    }
  } else if (status_ == RobotStatus::Unpausing) {
    if (timestep_ >= 0) {
      // robot has reached full speed again
      // set robot pause status to false:
      comm_interface_->SetPauseStatus(false);
      status_ = RobotStatus::Running;
      dexai::log()->warn(
          "IncreaseFrankaTimeBasedOnStatus: "
          "{} with final timestep_: {}",
          utils::RobotStatusToString(status_), timestep_);
    }
    double delta_franka_time = TimeToAdvanceWhilePausing(
        period_in_seconds, target_stop_time_, timestep_);
    franka_time_ += delta_franka_time;
    dexai::log()->info(
        "IncreaseFrankaTimeBasedOnStatus: Unpausing "
        "delta_franka_time: {}",
        delta_franka_time);
    timestep_++;
  } else if (status_ == RobotStatus::Running) {
    // robot is neither pausing, paused nor unpausing, just increase franka
    // time...
    franka_time_ += period_in_seconds;
    // } else if (status_ == RobotStatus::Reversing) {
    //   // walk back in time
    //   franka_time_ -= period_in_seconds;
  }

  if (status_ == RobotStatus::Paused) {
    // do nothing
    if (cancel_plan_requested) {
      if (plan_) {
        dexai::log()->warn(
            "IncreaseFrankaTimeBasedOnStatus: Paused successfully after "
            "canceling plan.");
        comm_interface_->PublishPlanComplete(plan_utime_, false, "canceled");
        plan_.release();
        plan_utime_ = -1;  // reset plan to -1
      }
      comm_interface_->ClearCancelPlanRequest();
      // check if robot is paused by other sources, or only pausing because the
      // plan was canceled. if the plan was canceled, and the robot is not
      // paused, we want to go back to regular running mode. if the robot was
      // already paused and the plan was canceled, we want to remain paused
      if (comm_interface_->GetPauseSources().empty()) {
        dexai::log()->warn(
            "IncreaseFrankaTimeBasedOnStatus: Plan canceled successfully, "
            "transitioning to idle.");
        // transition to idle
        comm_interface_->SetPauseStatus(false);
        status_ = RobotStatus::Running;
      } else {
        dexai::log()->warn(
            "IncreaseFrankaTimeBasedOnStatus: Plan canceled successfully, "
            "but robot is still paused.");
        // reset since the previous plan was canceled, we can unpause instantly
        target_stop_time_ = 0;
        stop_duration_ = 0;
      }
    }
  }
}

franka::JointPositions FrankaPlanRunner::JointPositionCallback(
    const franka::RobotState& robot_state, franka::Duration period) {
  if (comm_interface_->SimControlExceptionTriggered()) {
    dexai::log()->warn(
        "JointPositionCallback: simulated control exception triggered");
    RecoverFromControlException();
    comm_interface_->ClearSimControlExceptionTrigger();
    // return current joint positions instead of running plan through to
    // completion
    return franka::MotionFinished(franka::JointPositions(robot_state.q));
  }

  // check pause status and update franka_time_:
  IncreaseFrankaTimeBasedOnStatus(robot_state.dq, period.toSec());

  // read out robot state
  franka::JointPositions output_to_franka {robot_state.q_d};
  // scale to cannonical robot state
  auto cannonical_robot_state =
      utils::ConvertToCannonical(robot_state, joint_pos_offset_);
  // set current_conf
  Eigen::VectorXd current_conf_franka =
      utils::v_to_e(utils::ArrayToVector(cannonical_robot_state.q_d));

  if (comm_interface_->HasNewPlan()) {  // pop the new plan and set it up
    std::tie(plan_, plan_utime_) = comm_interface_->PopNewPlan();
    dexai::log()->info(
        "JointPositionCallback: popped new plan {} from buffer, "
        "starting initial timestep...",
        plan_utime_);
    // first time step of plan, reset time and start conf
    franka_time_ = 0.0;
    start_conf_plan_ = plan_->value(franka_time_);

    if (!LimitJoints(start_conf_plan_)) {
      dexai::log()->warn(
          "JointPositionCallback: plan {} at franka_time_: {} seconds "
          "is exceeding the joint limits!",
          plan_utime_, franka_time_);
    }

    // the current (desired) position of franka is the starting position:
    start_conf_franka_ = current_conf_franka;
    end_conf_plan_ = plan_->value(plan_->end_time());

    // Maximum change in joint angle between two confs
    auto max_ang_distance =
        utils::max_angular_distance(start_conf_franka_, start_conf_plan_);
    if (max_ang_distance > params_.kMediumJointDistance) {
      dexai::log()->error(
          "JointPositionCallback: Discarding plan, mismatched start position."
          " Max distance: {} > {}",
          max_ang_distance, params_.kMediumJointDistance);
      plan_.release();
      plan_utime_ = -1;  // reset plan to -1
      comm_interface_->PublishPlanComplete(
          plan_utime_, false, "discarded due to mismatched start conf");
      return franka::MotionFinished(franka::JointPositions(robot_state.q));
    } else if (max_ang_distance > params_.kTightJointDistance) {
      dexai::log()->warn(
          "JointPositionCallback: max angular distance between franka and "
          "start of plan is larger than 'kTightJointDistance': {} > {}",
          max_ang_distance, params_.kTightJointDistance);
    }
  }

  if (!plan_) {
    if (!is_sim_) {
      // only in sim is JointPositionCallback spam-called
      // but we want to avoid spamming the terminal and logs
      dexai::log()->info(
          "JointPositionCallback: No plan exists (anymore), exiting "
          "controller...");
    }
    comm_interface_->SetRobotData(cannonical_robot_state, start_conf_franka_);
    return franka::MotionFinished(output_to_franka);
  }

  // read out plan for current franka time from plan:
  next_conf_plan_ = plan_->value(franka_time_);
  // async in another thread, nonblocking
  std::thread {[&]() {
    comm_interface_->SetRobotData(cannonical_robot_state, next_conf_plan_);
  }}.detach();
  const auto plan_end_time = plan_->end_time();
  Eigen::VectorXd next_conf_combined(7);  // derive the next conf for return
  {
    // We don't track the last callback and delta from it
    // So we always calculate delta from the start of the plan
    // For both the plan itself and the franks, since they don't start from
    // the exact same position.
    // delta between conf at start of plan to conft at current time of plan:
    Eigen::VectorXd delta_conf_plan {next_conf_plan_ - start_conf_plan_};
    Eigen::VectorXd next_conf_franka {start_conf_franka_ + delta_conf_plan};

    // Expotentially weigh the next plan and franka confs for faster convergence
    // plan_completion_frac is in [0, 1] despite possibly being overtime
    const double plan_completion_frac {
        std::min(1.0, franka_time_ / plan_end_time)};
    // weight term goes from 1 to nearly 0 as frac goes from 0 to 1
    const double weight_e {std::exp(-15 * plan_completion_frac)};
    next_conf_combined =
        weight_e * next_conf_franka + (1 - weight_e) * next_conf_plan_;
    // apply low-pass filter at 30 Hz
    for (size_t i {}; i < 7; i++) {
      next_conf_combined[i] =
          franka::lowpassFilter(period.toSec(), next_conf_combined[i],
                                cannonical_robot_state.q_d[i], 30.0);
    }
  }

  // overwrite the output_to_franka of this callback:
  Eigen::VectorXd output_to_franka_eigen =
      next_conf_combined - joint_pos_offset_;
  if (!LimitJoints(output_to_franka_eigen)) {
    dexai::log()->warn(
        "JointPositionCallback: next_conf_combined - joint_pos_offset_ at {}s "
        "is exceeding the joint limits!",
        franka_time_);
  }
  output_to_franka = utils::EigenToArray(output_to_franka_eigen);

  if (auto overtime {franka_time_ - plan_end_time};
      overtime > 0) {  // check for convergence when overtime
    // Maximum change in joint angle between two confs
    const double max_joint_err {
        utils::max_angular_distance(end_conf_plan_, current_conf_franka)};
    Eigen::VectorXd dq_abs {
        utils::v_to_e(utils::ArrayToVector(cannonical_robot_state.dq))
            .cwiseAbs()};
    dexai::log()->warn(
        "JointPositionCallback: plan {} overtime, "
        "franka_t: {:.3f}, max joint err: {:.4f}, speed norm: {:.5f}",
        plan_utime_, franka_time_, max_joint_err, dq_abs.norm());
    // check convergence, return finished if two conditions are met
    if (max_joint_err <= CONV_ANGLE_THRESHOLD
        && (dq_abs.array() <= CONV_SPEED_THRESHOLD.array()).all()
        && dq_abs.norm() <= CONV_SPEED_NORM_THRESHOLD) {
      dexai::log()->info(
          "JointPositionCallback: plan {} overtime by {:.4f} s, "
          "converged within grace period, finished; "
          "plan duration: {:.3f} s, franka_t: {:.3f} s",
          plan_utime_, overtime, plan_end_time, franka_time_);
      comm_interface_->PublishPlanComplete(plan_utime_, true /* = success */);
      plan_.release();     // reset unique ptr
      plan_utime_ = -1;    // reset plan to -1
      dexai::log()->info(  // for control exception
          "Joint speeds at convergence:\n\tdq:\t{}\n\texcess:\t{}",
          dq_abs.transpose(),
          (dq_abs.array() - CONV_SPEED_THRESHOLD.array()).transpose());
      return franka::MotionFinished(output_to_franka);
    }
    // proceed below when not converged
    {  // print joints positions that have diverged
      auto error_eigen = (end_conf_plan_ - current_conf_franka).cwiseAbs();
      for (std::decay_t<decltype(dof_)> i {}; i < dof_; i++) {
        if (error_eigen(i) > CONV_ANGLE_THRESHOLD) {
          dexai::log()->warn(
              "JointPositionCallback: plan {} overtime, diverged, joint {} "
              "error: {:.4f} - {:.4f} = {:.4f} > max allowable: {}",
              plan_utime_, i, end_conf_plan_(i), current_conf_franka(i),
              error_eigen(i), CONV_ANGLE_THRESHOLD);
        }
      }
    }
    // terminate plan if grace period has ended and still not converged
    // in both position and speed after the deadline
    if (franka_time_ > (plan_->end_time() + 0.1)) {  // 100 ms
      if (max_joint_err < 1e-4) {
        // converged in position, publish success and don't wait for speed
        dexai::log()->warn(
            "JointPositionCallback: plan {} overtime by {:.4f} s, grace period "
            "exceeded, position converged with small residual speed, aborted "
            "and successful",
            plan_utime_, overtime);
        comm_interface_->PublishPlanComplete(
            plan_utime_, true, "position converged with small residual speed");
        plan_.release();   // reset unique ptr
        plan_utime_ = -1;  // reset plan to -1
        return franka::MotionFinished(output_to_franka);
      }
      // position hasn't converged, truly diverged, unsuccessful
      dexai::log()->error(
          "JointPositionCallback: plan {} overtime by {:.4f} s, grace period "
          "exceeded, still divergent, aborted and unsuccessful",
          plan_utime_, overtime);
      comm_interface_->PublishPlanComplete(plan_utime_, false, "diverged");
      plan_.release();   // reset unique ptr
      plan_utime_ = -1;  // reset plan to -1
      return franka::MotionFinished(output_to_franka);
    }
    dexai::log()->warn(
        "JointPositionCallback: plan {} overtime, diverged or still moving, "
        "within allowed grace period",
        plan_utime_);
  }
  return output_to_franka;
}
