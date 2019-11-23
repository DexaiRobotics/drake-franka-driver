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

#include "franka_plan_runner.h"

#include "drake/lcmt_iiwa_status.hpp"
#include "examples_common.h"   // for setDefaultBehavior
#include "franka/exception.h"  // for Exception, ControlException
#include "franka/robot.h"      // for Robot

#include <bits/stdc++.h>  // INT_MAX
#include <cmath>          // for exp
#include <iostream>       // for size_t

using namespace franka_driver;
namespace dru = dracula_utils;

FrankaPlanRunner::FrankaPlanRunner(const parameters::Parameters params)
    : params_(params), ip_addr_(params.robot_ip) {
  // define robot's state as uninitialized at start:
  status_ = RobotStatus::Uninitialized;

  // setup communication interface
  comm_interface_ =
      std::make_unique<CommunicationInterface>(params_, lcm_publish_rate_);

  // for pause logic:
  franka_time_ = 0.0;
  max_accels_ = params.robot_max_accelerations;

  // setup dracula instance
  dracula_ = std::make_unique<Dracula>(params_);
  joint_limits_ = dracula_->GetCS()->GetJointLimits();
  momap::log()->info("Joint limits: {}", joint_limits_.transpose());

  start_conf_franka_ = Eigen::VectorXd::Zero(dof_);
  start_conf_plan_ = Eigen::VectorXd::Zero(dof_);
}

int FrankaPlanRunner::Run() {
  comm_interface_->StartInterface();

  int return_value = 1;  //
  if (ip_addr_ == home_addr) {
    return_value = RunSim();
  } else {
    return_value = RunFranka();
  }

  comm_interface_->StopInterface();

  status_ = RobotStatus::Uninitialized;

  return return_value;
}

void FrankaPlanRunner::SetCollisionBehaviour(franka::Robot& robot,
                                             bool safety_on) {
  if (status_ != RobotStatus::Uninitialized) {
    throw std::runtime_error(
        "robot is already initialized, cannot change collision behaviour!");
    return;
  }
  if (safety_on) {
    robot.setCollisionBehavior({{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
                               {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
                               {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
                               {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
                               {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}},
                               {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}},
                               {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}},
                               {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}});
  } else {
    robot.setCollisionBehavior(
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
  }
}

int FrankaPlanRunner::RunFranka() {
  //$ attempt connection to robot and read current mode
  //$ return if connection fails, or robot is in a mode that cannot receive
  // commands
  try {
    franka::Robot robot(ip_addr_);

    size_t count = 0;
    franka::RobotMode current_mode;
    robot.read([&count, &current_mode](const franka::RobotState& robot_state) {
      current_mode = robot_state.robot_mode;
      return count++ < 100;
    });
    momap::log()->info("RunFranka: Franka's current mode: {}",
                       RobotModeToString(current_mode));

    if (current_mode != franka::RobotMode::kIdle) {
      momap::log()->error(
          "RunFranka: Robot cannot receive commands in mode: {}",
          RobotModeToString(current_mode));
      comm_interface_->PublishDriverStatus(false,
                                           RobotModeToString(current_mode));
      return 1;
    }
  } catch (franka::Exception const& e) {
    momap::log()->error(
        "RunFranka: Received franka exception: {} - Do not have error handling "
        "for it yet...",
        e.what());
    comm_interface_->PublishDriverStatus(false, e.what());
    return 1;
  }

  try {
    // Connect to robot.
    franka::Robot robot(ip_addr_);
    setDefaultBehavior(robot);

    momap::log()->info("RunFranka: Ready.");
    comm_interface_->PublishDriverStatus(true);

    // Set additional parameters always before the control loop, NEVER in the
    // control loop! Set collision behavior.

    bool safety_on = true;
    SetCollisionBehaviour(robot, safety_on);

    std::function<franka::JointPositions(const franka::RobotState&,
                                         franka::Duration)>
        joint_position_callback =
            [&, this](const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::JointPositions {
      return this->FrankaPlanRunner::JointPositionCallback(robot_state, period);
    };

    // Initilization is done, define robot as running:
    status_ = RobotStatus::Running;

    int error_counter = 0;
    int counter = INT_MAX;

    //$ main control loop
    while (true) {
      // std::cout << "top of loop: Executing motion." << std::endl;
      try {
        if (error_counter == 2) {
          momap::log()->error(
              "RunFranka: Error happened twice in a row, running franka's "
              "automaticErrorRecovery!");
          robot.automaticErrorRecovery();
        }
        auto paused_by_lcm = comm_interface_->GetPauseStatus();
        if (paused_by_lcm) {
          status_ = RobotStatus::Paused;
        } else {
          status_ = RobotStatus::Running;
        }
        // prevent the plan from being started if robot is not running...
        if (comm_interface_->HasNewPlan() && status_ == RobotStatus::Running) {
          momap::log()->info("RunFranka: Got a new plan, attaching callback!");
          // joint_position_callback or impedance_control_callback can be used
          // here:
          robot.control(joint_position_callback);
          // reset error counter once motion was successful, i.e. no exception
          // was thrown
          error_counter = 0;
        } else {
          if (counter > static_cast<int>(lcm_publish_rate_ * 10)) {
            momap::log()->info("RunFranka: RobotStatus: {}, waiting for plan or unpause.",
                               RobotStatusToString(status_));
            counter = 0;  // reset
          }
          // only publish robot_status
          // TODO: add a timer to be closer to 200 Hz.
          robot.read([this](const franka::RobotState& robot_state) {
            comm_interface_->SetRobotState(robot_state);
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(1000.0 / lcm_publish_rate_)));
            return false;
          });
          counter++;
        }
      } catch (const franka::ControlException& ce) {
        error_counter++;
        momap::log()->warn("RunFranka: Caught control exception: {}.",
                           ce.what());
        if (error_counter > 2) {
          momap::log()->error("RunFranka: Error recovery did not work!");
          comm_interface_->PublishDriverStatus(false, ce.what());
          return 1;
        }
      }
    }

  } catch (const franka::Exception& ex) {
    momap::log()->error(
        "RunFranka: Caught expection during initilization, msg: {}", ex.what());
    comm_interface_->PublishDriverStatus(false, ex.what());
    return 1;  // bad things happened.
  }
  return 0;
};

int FrankaPlanRunner::RunSim() {
  momap::log()->info("Starting sim robot.");
  // first, load some parameters
  dracula_->MutableViz()->loadRobot();
  Eigen::VectorXd next_conf = Eigen::VectorXd::Zero(dof_);  // output state
  next_conf << -0.9577375507190063, -0.7350638062912122, 0.880988748620542,
      -2.5114236381136448, 0.6720116891296624, 1.9928838396072361,
      -1.2954019628351783;  // set robot in a starting position which is not in
                            // collision
  Eigen::VectorXd prev_conf;
  std::vector<double> vel(7, 1);
  franka::RobotState robot_state;  // internal state; mapping to franka state
  franka::Duration period;
  std::chrono::milliseconds last_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());

  status_ = RobotStatus::Running;  // define robot as running at start

  while (1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(1000.0 / lcm_publish_rate_)));

    std::vector<double> next_conf_vec = dru::e_to_v(next_conf);
    VectorToArray(next_conf_vec, robot_state.q);
    VectorToArray(next_conf_vec, robot_state.q_d);
    VectorToArray(vel, robot_state.dq);

    franka::JointPositions cmd_pos = JointPositionCallback(robot_state, period);

    prev_conf = next_conf.replicate(1, 1);

    next_conf = dru::v_to_e(ArrayToVector(cmd_pos.q));
    dracula_->GetViz()->displayState(next_conf);

    next_conf_vec = dru::e_to_v(next_conf);
    std::vector<double> prev_conf_vec = dru::e_to_v(prev_conf);

    for (int i = 0; i < 7; i++) {
      vel[i] = (next_conf_vec[i] - prev_conf_vec[i]) / (double)period.toSec();
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

bool FrankaPlanRunner::LimitJoints(Eigen::VectorXd& conf) {
  // check and limit according to joint limits:
  // TODO @rkk: get limits from urdf
  // TODO @rkk: use eigen operator to do this
  bool within_limits = true;
  for (int j = 0; j < conf.size(); j++) {
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

double FrankaPlanRunner::StopPeriod(double period, double target_stop_time,
                                    double timestep) {
  // Logistic growth function: t' = f - 4 / [a(e^{at}+1] where
  // f = target_stop time, t' = franka time, t = real time
  // Returns delta t', the period that should be incremented to franka time
  double a = 2 / target_stop_time;
  double current_time =
      (target_stop_time - 4 / (a * (exp(a * period * timestep) + 1)));
  double prev_time =
      (target_stop_time - 4 / (a * (exp(a * period * (timestep - 1)) + 1)));
  return (current_time - prev_time);
}

void FrankaPlanRunner::IncreaseFrankaTimeBasedOnStatus(
    const std::array<double, 7>& vel, double period_in_seconds) {
  // get pause data from the communication interface
  auto paused = comm_interface_->GetPauseStatus();

  // robot can be in four states: running, pausing, paused, unpausing

  // check if robot is supposed to be paused
  if (paused && status_ == RobotStatus::Running) {
    timestep_ = 1;             // set time step back to 1
    stop_duration_ = 0;        // reset stop duration
    stop_margin_counter_ = 0;  // reset margin counter

    // set a target_stop_time_ given the current state of the robot:
    float temp_target_stop_time_ = 0;
    for (int i = 0; i < 7; i++) {
      // sets target stop_time in plan as
      // max(vel_i/max_accel_i), where i
      // is each joint. real world stop
      // time ~ 2x stop_time in plan
      float stop_time = fabs(vel[i] / (max_accels_[i]));
      if (stop_time > temp_target_stop_time_) {
        temp_target_stop_time_ = stop_time;
      }
    }
    target_stop_time_ = temp_target_stop_time_ / STOP_SCALE;
    status_ = RobotStatus::Pausing;
    momap::log()->warn(
        "FrankaPlanRunner::IncreaseFrankaTimeBasedOnStatus: "
        "{} with target_stop_time_: {}",
        RobotStatusToString(status_), target_stop_time_);
  }

  // check if robot should get unpaused
  if (!paused && status_ == RobotStatus::Paused) {
    // the duration it took to step is now used to unpause:
    timestep_ = -1 * stop_duration_;
    status_ = RobotStatus::Unpausing;
    momap::log()->warn(
        "FrankaPlanRunner::IncreaseFrankaTimeBasedOnStatus: "
        "{} with new timestep_: {}",
        RobotStatusToString(status_), timestep_);
  }

  if (status_ == RobotStatus::Pausing) {
    double new_stop =
        StopPeriod(period_in_seconds, target_stop_time_, timestep_);
    franka_time_ += new_stop;
    momap::log()->trace(
        "FrankaPlanRunner::IncreaseFrankaTimeBasedOnStatus: Pausing period: {}",
        new_stop);
    timestep_++;

    if (new_stop >= period_in_seconds * params_.stop_epsilon) {
      // robot counts as "stopped" when new_stop is
      // less than a fraction of period_in_seconds
      stop_duration_++;
    } else if (stop_margin_counter_ <= params_.stop_margin) {
      // margin period_in_seconds after pause before robot is
      // allowed to continue
      stop_margin_counter_ += period_in_seconds;
    } else {
      comm_interface_->SetPauseStatus(true);
      status_ = RobotStatus::Paused;
      momap::log()->warn(
          "FrankaPlanRunner::IncreaseFrankaTimeBasedOnStatus: "
          "{} with new_stop: {}, stop_duration_: {}"
          " and stop_margin_counter_: {}",
          RobotStatusToString(status_), new_stop,
          stop_duration_, stop_margin_counter_);
    }
  } else if (status_ == RobotStatus::Unpausing) {
    if (timestep_ >= 0) {
      // robot has reached full speed again
      // set robot pause status to false:
      comm_interface_->SetPauseStatus(false);
      status_ = RobotStatus::Running;
      momap::log()->warn(
      "FrankaPlanRunner::IncreaseFrankaTimeBasedOnStatus: "
      "{} with final timestep_: {}",
      RobotStatusToString(status_), timestep_);
    }
    double new_stop =
        StopPeriod(period_in_seconds, target_stop_time_, timestep_);
    franka_time_ += new_stop;
    momap::log()->trace(
        "FrankaPlanRunner::IncreaseFrankaTimeBasedOnStatus: Unpausing period: "
        "{}",
        new_stop);
    timestep_++;
  } else if (status_ == RobotStatus::Running) {
    // robot is neither pausing, paused nor unpausing, just increase franka
    // time...
    franka_time_ += period_in_seconds;
  } else if (status_ == RobotStatus::Paused) {
    // do nothing
  }
}

franka::JointPositions FrankaPlanRunner::JointPositionCallback(
    const franka::RobotState& robot_state, franka::Duration period) {
  // check pause status and update franka_time_:
  IncreaseFrankaTimeBasedOnStatus(robot_state.dq, period.toSec());

  // read out robot state
  franka::JointPositions output_to_franka = robot_state.q_d;
  auto q_d_v = ArrayToVector(robot_state.q_d);
  Eigen::VectorXd current_conf_franka = dru::v_to_e(q_d_v);
  // Set robot state for LCM publishing:
  // TODO @rkk: do not use franka robot state but use a generic Eigen instead
  comm_interface_->TryToSetRobotState(robot_state);

  // get the current plan from the communication interface
  if (comm_interface_->HasNewPlan()) {
    comm_interface_->TakeOverPlan(plan_);
    if (!plan_) {
      momap::log()->error(
          "FrankaPlanRunner::JointPositionCallback: plan is empty!");
      return franka::MotionFinished(output_to_franka);
    }
    // first time step of plan, reset time:
    franka_time_ = 0.0;
    start_conf_plan_ = plan_->value(franka_time_);  // TODO @rkk: fails

    if (!LimitJoints(start_conf_plan_)) {
      momap::log()->warn("plan at {}s is exceeding the joint limits!",
                         franka_time_);
    }

    // the current (desired) position of franka is the starting position:
    start_conf_franka_ = current_conf_franka;

    Eigen::VectorXd delta_start_to_end_plan =
        plan_->value(plan_->end_time()) - start_conf_plan_;

    end_conf_franka_ = start_conf_franka_ + delta_start_to_end_plan;
    // TODO @rkk: move this print into another thread
    momap::log()->debug("starting franka q = {}",
                        start_conf_franka_.transpose());
    momap::log()->debug("starting plan q = {}", start_conf_plan_.transpose());
    if (!dru::VectorEpsEq(start_conf_plan_, start_conf_franka_,
                          params_.kMediumJointDistance)) {
      momap::log()->error(
          "JointPositionCallback: Discarding plan, mismatched start position.");
      return franka::MotionFinished(output_to_franka);
    }

    double error_start = (start_conf_franka_ - start_conf_plan_).norm();
    // TODO @rkk: move this print into another thread
    if (error_start > allowable_error_) {
      momap::log()->warn(
          "too large a difference between where we are and where we think = {}",
          error_start);
    }
  }

  if (!plan_) {
    momap::log()->info("No plan exists (anymore), exiting controller...");
    return franka::MotionFinished(output_to_franka);
  }

  // read out plan for current franka time from plan:
  Eigen::VectorXd next_conf_plan = plan_->value(franka_time_);
  if (!LimitJoints(next_conf_plan)) {
    momap::log()->warn("plan at {}s is exceeding the joint limits!",
                       franka_time_);
  }

  // delta between conf at start of plan to conft at current time of plan:
  Eigen::VectorXd delta_conf_plan = next_conf_plan - start_conf_plan_;

  // add delta to current robot state to achieve a continuous motion:
  Eigen::VectorXd next_conf_franka = start_conf_franka_ + delta_conf_plan;

  // overwrite the output_to_franka of this callback:
  output_to_franka = EigenToArray(next_conf_franka);

  double error_final = (current_conf_franka - end_conf_franka_).norm();

  if (franka_time_ > plan_->end_time()) {
    // TODO @rkk: replace allowable_error_ with non arbitrary number
    if (error_final < allowable_error_) {
      momap::log()->info("Finished motion, exiting controller");
      comm_interface_->PublishPlanComplete(franka_time_);
      return franka::MotionFinished(output_to_franka);
    } else {
      momap::log()->warn(
          "Plan running overtime and not converged, error distance: {}",
          error_final);
      momap::log()->info("current_conf_franka: {}",
                         current_conf_franka.transpose());
      momap::log()->info("next_conf_franka: {}", next_conf_franka.transpose());
      momap::log()->info("next_conf_plan: {}", next_conf_plan.transpose());
      return franka::MotionFinished(output_to_franka);
    }
  }

  return output_to_franka;
}
