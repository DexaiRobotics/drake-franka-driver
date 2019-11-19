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

#include <franka/exception.h>  // for Exception, ControlException
#include <franka/robot.h>      // for Robot
#include <cmath>     // for exp
#include <iostream>  // for size_t



#include "drake/lcmt_iiwa_status.hpp"
#include "examples_common.h"            // for setDefaultBehavior
#include "franka_driver_utils.h"        // for get_current_utime
#include "robot_msgs/pause_cmd.hpp"     // for pause_cmd
#include "robot_msgs/trigger_t.hpp"     // for trigger_t
#include "trajectory_solver.h"          // for TrajectorySolver

using namespace franka_driver;
// using namespace std::chrono;

FrankaPlanRunner::FrankaPlanRunner(const parameters::Parameters params)
    : p(params),
      ip_addr_(params.robot_ip),
      plan_number_(0),
      lcm_(params.lcm_url) {
  lcm_.subscribe(p.lcm_plan_channel, &FrankaPlanRunner::HandlePlan, this);
  lcm_.subscribe(p.lcm_stop_channel, &FrankaPlanRunner::HandleStop, this);
  running_ = true;
  franka_time_ = 0.0;
  max_accels_ = params.robot_max_accelerations;

  dracula = new Dracula(p);
  lcm_driver_status_channel_ = p.robot_name + "_DRIVER_STATUS";
  lcm_pause_status_channel_ = p.robot_name + "_PAUSE_STATUS";
  joint_limits_ = dracula->GetCS()->GetJointLimits();
  momap::log()->info("Joint limits: {}", joint_limits_.transpose());

  cur_plan_number = plan_number_;
  cur_time_us_ = -1;
  start_time_us_ = -1;
  sign_ = +1;

  pausing_ = false;
  paused_ = false;
  unpausing_ = false;

  starting_franka_q_ = {{0, 0, 0, 0, 0, 0, 0}};
  starting_conf_ = Eigen::VectorXd::Zero(kNumJoints_);

  plan_.has_data = false;
  plan_.plan.release();
  plan_.utime = -1;
  plan_.end_time_us = 0;
  momap::log()->info("Plan channel: {}", p.lcm_plan_channel);
  momap::log()->info("Stop channel: {}", p.lcm_stop_channel);
  momap::log()->info("Plan received channel: {}", p.lcm_plan_received_channel);
  momap::log()->info("Plan complete channel: {}", p.lcm_plan_complete_channel);
  momap::log()->info("Status channel: {}", p.lcm_status_channel);
};

int FrankaPlanRunner::Run() {
  // start LCM threads; independent of sim vs. real robot
  lcm_publish_status_thread =
      std::thread(&FrankaPlanRunner::PublishLcmAndPauseStatus, this);
  lcm_handle_thread = std::thread(&FrankaPlanRunner::HandleLcm, this);
  int return_value = -1;  //
  if (ip_addr_ == home_addr) {
    return_value = RunSim();
  } else {
    return_value = RunFranka();
  }
  momap::log()->debug("Before LCM thread join");

  running_ = false;

  // clean-up threads if they're still alive.
  while (!lcm_handle_thread.joinable() ||
         !lcm_publish_status_thread.joinable()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    momap::log()->info("Waiting for LCM threads to be joinable...");
  }

  lcm_publish_status_thread.join();
  lcm_handle_thread.join();

  momap::log()->debug("After LCM thread join");
  return return_value;
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
    momap::log()->info("Current mode: {}", RobotModeToString(current_mode));

    if (current_mode != franka::RobotMode::kIdle) {
      momap::log()->info("Robot cannot receive commands in mode: {}",
                         RobotModeToString(current_mode));
      PublishTriggerToChannel(get_current_utime(), lcm_driver_status_channel_,
                              false, RobotModeToString(current_mode));
      return 1;
    }

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    PublishTriggerToChannel(get_current_utime(), lcm_driver_status_channel_,
                            false, e.what());
    return -1;
  }

  try {
    // Connect to robot.
    franka::Robot robot(ip_addr_);
    setDefaultBehavior(robot);
    robot_alive_ = true;

    std::cout << "Ready." << std::endl;
    PublishTriggerToChannel(get_current_utime(), lcm_driver_status_channel_,
                            true);

    // Set additional parameters always before the control loop, NEVER in the
    // control loop! Set collision behavior.

    bool we_care_about_safety = false;
    if (we_care_about_safety) {
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

    std::function<franka::JointPositions(const franka::RobotState&,
                                         franka::Duration)>
        joint_position_callback =
            [&, this](const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::JointPositions {
      return this->FrankaPlanRunner::JointPositionCallback(robot_state, period);
    };

    //$ main control loop
    while (true) {
      // std::cout << "top of loop: Executing motion." << std::endl;
      try {
        if (plan_.has_data && !paused_) {  //$ prevent the plan from being
                                           // started if robot is paused_
          robot.control(joint_position_callback);  // impedance_control_callback
        } else {
          // publish robot_status
          // TODO: add a timer to be closer to 200 Hz.
          // std::cout << "only should be here when sitting.\n";
          robot.read([this](const franka::RobotState& robot_state) {
            if (this->robot_data_.mutex.try_lock()) {
              this->robot_data_.has_data = true;
              this->robot_data_.robot_state = robot_state;
              this->robot_data_.mutex.unlock();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(1000.0 / lcm_publish_rate_)));
            return false;
          });
        }

      } catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        momap::log()->error("FRANKA ERROR. returning -99.");
        PublishTriggerToChannel(get_current_utime(), lcm_driver_status_channel_,
                                false, e.what());
        return -99;

        //$ UNCOMMENT BELOW TO PERFORM AUTOMATIC ERROR RECOVERY
        // if (plan_.mutex.try_lock() ) {
        //     robot.automaticErrorRecovery();
        //     plan_.mutex.unlock();
        // } else {
        //     momap::log()->error("failed to get a mutex after an error.
        //     returning -99."); PublishTriggerToChannel(get_current_utime(),
        //     lcm_driver_status_channel_, false, e.what()); return -99;
        // }
      }
    }

  } catch (const franka::Exception& ex) {
    running_ = false;
    momap::log()->error("drake::franka_driver::RunFranka Caught expection: {}",
                        ex.what());
    PublishTriggerToChannel(get_current_utime(), lcm_driver_status_channel_,
                            false, ex.what());
    return -99;  // bad things happened.
  }
  return 0;
};

int FrankaPlanRunner::RunSim() {
  robot_alive_ = true;  // the sim robot *always* starts as planned
  momap::log()->info("Starting sim robot.");
  // first, load some parameters
  dracula->MutableViz()->loadRobot();
  Eigen::VectorXd next_conf =
      Eigen::VectorXd::Zero(kNumJoints_);  // output state
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

  while (1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(1000.0 / lcm_publish_rate_)));

    std::vector<double> next_conf_vec = dracula_utils::e_to_v(next_conf);
    ConvertToArray(next_conf_vec, robot_state.q);
    ConvertToArray(next_conf_vec, robot_state.q_d);
    ConvertToArray(vel, robot_state.dq);

    franka::JointPositions cmd_pos = JointPositionCallback(robot_state, period);

    prev_conf = next_conf.replicate(1, 1);

    next_conf = dracula_utils::v_to_e(ConvertToVector(cmd_pos.q));
    dracula->GetViz()->displayState(next_conf);

    next_conf_vec = dracula_utils::e_to_v(next_conf);
    std::vector<double> prev_conf_vec = dracula_utils::e_to_v(prev_conf);

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

double FrankaPlanRunner::StopPeriod(double period) {
  /*Logistic growth function: t' = f - 4 / [a(e^{at}+1] where
  f = target_stop time, t' = franka time, t = real time
  Returns delta t', the period that should be incremented to franka time*/
  double a = 2 / target_stop_time_;
  double current_time = (this->target_stop_time_ -
                         4 / (a * (exp(a * period * this->timestep_) + 1)));
  double prev_time = (this->target_stop_time_ -
                      4 / (a * (exp(a * period * (this->timestep_ - 1)) + 1)));
  return (current_time - prev_time);
}

void FrankaPlanRunner::QueuedCmd() {
  robot_msgs::pause_cmd msg;
  msg.utime = get_current_utime();
  switch (queued_cmd_) {
    case QueuedCommand::NONE:
      return;
    case QueuedCommand::PAUSE:
      msg.data = true;
      break;
    case QueuedCommand::CONTINUE:
      msg.data = false;
      break;
  }
  msg.source = "queued";
  lcm_.publish(p.lcm_stop_channel, &msg);
  queued_cmd_ = QueuedCommand::NONE;
}

franka::JointPositions FrankaPlanRunner::JointPositionCallback(
    const franka::RobotState& robot_state, franka::Duration period) {
  franka::JointPositions output =
      robot_state.q_d;  // should this be robot_state.q_d?

  if (plan_.mutex.try_lock()) {
    // we got the lock, so try and do stuff.
    // momap::log()->info("got the lock!");

    if (pausing_) {
      if (target_stop_time_ ==
          0) {  // if target_stop_time_ not set, set target_stop_time_
        std::array<double, 7> vel = robot_state.dq;
        float temp_target_stop_time_ = 0;
        for (int i = 0; i < 7; i++) {
          float stop_time =
              fabs(vel[i] /
                   (this->max_accels_[i]));  // sets target stop_time in plan as
                                             // max(vel_i/max_accel_i), where i
                                             // is each joint. real world stop
                                             // time ~ 2x stop_time in plan
          if (stop_time > temp_target_stop_time_) {
            temp_target_stop_time_ = stop_time;
          }
        }
        target_stop_time_ = temp_target_stop_time_ / STOP_SCALE;
        momap::log()->debug("TARGET: {}", target_stop_time_);
      }

      double new_stop = StopPeriod(period.toSec());
      franka_time_ += new_stop;
      momap::log()->debug("STOP PERIOD: {}", new_stop);
      timestep_++;

      if (new_stop >=
          period.toSec() *
              p.stop_epsilon) {  // robot counts as "stopped" when new_stop is
                                 // less than a fraction of period
        this->stop_duration_++;
      } else if (stop_margin_counter_ <=
                 p.stop_margin) {  // margin period after pause before robot is
                                   // allowed to continue
        stop_margin_counter_ += period.toSec();
      } else {
        paused_ = true;
        QueuedCmd();
      }

    } else if (unpausing_) {  // robot is unpausing_
      if (timestep_ >= 0) {   // if robot has reached full speed again
        unpausing_ = false;
        QueuedCmd();
      }
      double new_stop = StopPeriod(period.toSec());
      franka_time_ += new_stop;
      momap::log()->debug("CONTINUE PERIOD: {}", new_stop);
      timestep_++;

    } else {
      franka_time_ += period.toSec();
    }

    cur_time_us_ = int64_t(franka_time_ * 1.0e6);

    // TODO: remove the need for this check. who cares if it is a new plan?
    // TODO: make sure we've called motion finished and reset the timer?

    if (plan_.plan && plan_number_ != cur_plan_number) {
      momap::log()->info("Starting new plan at {} s.", franka_time_);
      start_time_us_ =
          cur_time_us_;  // implies that we should have call motion finished
      cur_plan_number = plan_number_;
      starting_conf_ = plan_.plan->value(0.0);
      starting_franka_q_ = robot_state.q_d;
      momap::log()->warn(
          "starting franka q = {}",
          dracula_utils::v_to_e(ConvertToVector(starting_franka_q_))
              .transpose());
      momap::log()->warn(
          "difference between where we are and where we think = {}",
          (dracula_utils::v_to_e(ConvertToVector(starting_franka_q_)) -
           starting_conf_)
              .norm());
    }

    // Update data to publish.
    // TODO: move to publish loop
    if (robot_data_.mutex.try_lock()) {
      robot_data_.has_data = true;
      robot_data_.robot_state = robot_state;
      robot_data_.mutex.unlock();
    }

    Eigen::VectorXd desired_next = Eigen::VectorXd::Zero(kNumJoints_);
    std::array<double, 7> current_cmd =
        robot_state.q_d;  // set to actual, not desired
    std::array<double, 7> current_conf =
        robot_state.q_d;  // set to actual, not desired
    desired_next = dracula_utils::v_to_e(ConvertToVector(current_cmd));

    double error = DBL_MAX;

    // const double cur_traj_time_s = static_cast<double>(cur_time_us_ -
    // start_time_us_) / 1e6;
    if (plan_.plan) {
      desired_next = plan_.plan->value(franka_time_);  // cur_traj_time_s
      // TODO: remove - not DRY
      for (int j = 0; j < desired_next.size(); j++) {
        if (desired_next(j) > joint_limits_(j, 1)) {
          desired_next(j) = joint_limits_(j, 1);
        } else if (desired_next(j) < joint_limits_(j, 0)) {
          desired_next(j) = joint_limits_(j, 0);
        }
      }
      Eigen::VectorXd delta = desired_next - starting_conf_;
      Eigen::VectorXd output_eigen =
          dracula_utils::v_to_e(ConvertToVector(starting_franka_q_)) + delta;
      Eigen::VectorXd delta_end =
          plan_.plan->value(plan_.plan->end_time()) - starting_conf_;
      Eigen::VectorXd starting_q_eigen =
          dracula_utils::v_to_e(ConvertToVector(starting_franka_q_));
      Eigen::VectorXd output_end = starting_q_eigen + delta_end;
      Eigen::VectorXd current_conf_eigen =
          dracula_utils::v_to_e(ConvertToVector(current_conf));
      // error = ( dracula_utils::v_to_e( ConvertToVector(current_conf) ) -
      // plan_.plan->value(plan_.plan->end_time()) ).norm();
      error = (current_conf_eigen - output_end).norm();

      // momap::log()->warn("starting franka q = {}", dracula_utils::v_to_e(
      // ConvertToVector(starting_franka_q_) ).transpose());
      // momap::log()->warn("starting_q_eigen = {}",
      // starting_q_eigen.transpose()); momap::log()->info("error: {}", error);
      // momap::log()->info("current_conf_eigen: {}",
      // current_conf_eigen.transpose()); momap::log()->info("output_end: {}",
      // output_end.transpose());

      // set desired position based on interpolated spline
      // output = {{ desired_next[0], desired_next[1],
      //             desired_next[2], desired_next[3],
      //             desired_next[4], desired_next[5],
      //             desired_next[6] }};

      output = {{output_eigen[0], output_eigen[1], output_eigen[2],
                 output_eigen[3], output_eigen[4], output_eigen[5],
                 output_eigen[6]}};

      // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2,
      // M_PI_4}}; output = q_goal;

      if (franka_time_ > plan_.plan->end_time()) {
        if (error < 0.007) {  // TODO: replace with non arbitrary number
          franka::JointPositions ret_val = current_conf;
          std::cout << std::endl
                    << "Finished motion, exiting controller" << std::endl;
          plan_.plan.release();
          plan_.has_data = false;
          // plan_.utime = -1;
          plan_.mutex.unlock();

          PublishTriggerToChannel(plan_.utime, p.lcm_plan_complete_channel);
          // return output;
          // plan_.mutex.unlock();
          return franka::MotionFinished(output);
        } else {
          momap::log()->info(
              "Plan running overtime and not converged, error: {}", error);
          // momap::log()->info("q:   {}", dracula_utils::v_to_e(
          // ConvertToVector(current_conf)).transpose());
          // momap::log()->info("q_d: {}", desired_next.transpose());
        }
      }
    } else {
      // momap::log()->error("Inside JPC but plan_.plan != True");
    }

    plan_.mutex.unlock();
    return output;
  }

  // we couldn't get the lock, so probably need to return motion::finished()
  // plan_.mutex.unlock();
  return franka::MotionFinished(output);
};

void FrankaPlanRunner::HandleLcm() {
  while (running_) {
    lcm_.handleTimeout(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void FrankaPlanRunner::PublishLcmAndPauseStatus() {  //::lcm::LCM &lcm,
                                                     // RobotData &robot_data,
                                                     // std::atomic_bool
                                                     // &running
  while (running_) {
    // Sleep to achieve the desired print rate.
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(1000.0 / lcm_publish_rate_)));

    // Try to lock data to avoid read write collisions.
    if (robot_data_.mutex.try_lock()) {
      if (robot_data_.has_data) {
        drake::lcmt_iiwa_status franka_status =
            ConvertToLcmStatus(robot_data_.robot_state);
        // publish data over lcm
        lcm_.publish(p.lcm_status_channel, &franka_status);
        robot_data_.has_data = false;
      }
      robot_data_.mutex.unlock();
    }

    if (pause_mutex_.try_lock()) {
      PublishPauseStatus();
      pause_mutex_.unlock();
    }
  }
};

void FrankaPlanRunner::PublishPauseStatus() {
  robot_msgs::trigger_t msg;
  msg.utime = get_current_utime();
  msg.success = paused_;
  msg.message = "";
  if (paused_ || pausing_) {  // TODO @syler: what if it's unpausing?
    for (auto elem : stop_set_) {
      msg.message.append(elem);
      msg.message.append(",");
    }
    // momap::log()->info("PublishPauseStatus with msg.message: {}",
    // msg.message);
  }
  lcm_.publish(lcm_pause_status_channel_, &msg);
}

void FrankaPlanRunner::PublishTriggerToChannel(int64_t utime,
                                               std::string lcm_channel,
                                               bool success,
                                               std::string message) {
  robot_msgs::trigger_t msg;
  msg.utime = utime;
  msg.success = success;
  msg.message = message;
  lcm_.publish(lcm_channel, &msg);
}

bool FrankaPlanRunner::CanReceiveCommands() {
  std::lock_guard<std::mutex> lock(
      robot_data_.mutex);  //$ unlocks when lock_guard goes out of scope

  franka::RobotMode current_mode = robot_data_.robot_state.robot_mode;

  momap::log()->info("Current mode: {}", RobotModeToString(current_mode));

  //$ TODO: in the future, we may want to send commands to overwrite the current
  // command $ currently, can only accept a plan if not already running one
  if (current_mode == franka::RobotMode::kIdle) {
    return true;
  }

  momap::log()->error("CanReceiveCommands: Wrong mode!");
  return false;
}

void FrankaPlanRunner::HandlePlan(const ::lcm::ReceiveBuffer*,
                                  const std::string&,
                                  const lcmtypes::robot_spline_t* rst) {
  momap::log()->info("New plan received.");
  if (!robot_alive_) {
    momap::log()->info(
        "Discarding plan, no status message received yet from the robot");
    return;
  }

  //$ check if in proper mode to receive commands
  if (!CanReceiveCommands()) {
    momap::log()->error("Discarding plan, in wrong mode!");
    return;
  }

  // plan_.mutex.lock();
  while (!plan_.mutex.try_lock()) {
    momap::log()->warn(
        "trying to get a lock on the plan_.mutex. Sleeping 1 ms and trying "
        "again.");
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1.0)));
  }

  editing_plan_ = true;

  momap::log()->info("utime: {}", rst->utime);
  plan_.utime = rst->utime;
  //$ publish confirmation that plan was received with same utime
  PublishTriggerToChannel(plan_.utime, p.lcm_plan_received_channel);
  momap::log()->info("Published confirmation of received plan");

  franka_time_ = 0.0;
  piecewise_polynomial_ = TrajectorySolver::RobotSplineTToPPType(*rst);

  if (piecewise_polynomial_.get_number_of_segments() < 1) {
    momap::log()->info("Discarding plan, invalid piecewise polynomial.");
    plan_.mutex.unlock();
    return;
  }

  momap::log()->info("start time: {}", piecewise_polynomial_.start_time());
  momap::log()->info("end time: {}", piecewise_polynomial_.end_time());

  // Start position == goal position check
  // TODO: add end position==goal position check (upstream)
  // TODO: change to append initial position and respline here
  Eigen::VectorXd commanded_start =
      piecewise_polynomial_.value(piecewise_polynomial_.start_time());
  for (int joint = 0; joint < rst->dof; joint++) {
    if (!dracula_utils::EpsEq(commanded_start(joint),
                              robot_data_.robot_state.q[joint],
                              p.kMediumJointDistance)) {
      momap::log()->info("Discarding plan, mismatched start position.");
      plan_.has_data = false;
      plan_.plan.release();
      plan_.mutex.unlock();
      return;
    }
  }

  plan_.plan.release();
  plan_.plan.reset(&piecewise_polynomial_);
  plan_.has_data = true;

  ++plan_number_;
  momap::log()->warn("Finished Handle Plan!");
  editing_plan_ = false;
  plan_.mutex.unlock();
};

void FrankaPlanRunner::HandleStop(const ::lcm::ReceiveBuffer*,
                                  const std::string&,
                                  const robot_msgs::pause_cmd* msg) {
  std::lock_guard<std::mutex> lock(
      pause_mutex_);  //$ unlocks when lock_guard goes out of scope
  if (msg->data) {    // if pause command received
    if (msg->source != "queued") {
      stop_set_.insert(msg->source);
    }
    momap::log()->info("Received pause from {}", msg->source);
    if (!pausing_) {      // if this is first stop received
      if (!unpausing_) {  // if robot isn't currently unpausing_
        Pause(msg->source);
      } else {  // if robot is currently unpausing_, queue pause cmd
        queued_cmd_ = QueuedCommand::PAUSE;
      }
    }
  } else if (!msg->data) {  // if unpause command received
    momap::log()->info("Received continue from {}", msg->source);
    if (stop_set_.find(msg->source) != stop_set_.end() ||
        msg->source == "queued") {  // force continue if msg->source == queued
                                    // in order for queue to work
      if (stop_set_.find(msg->source) != stop_set_.end()) {
        stop_set_.erase(msg->source);
      }

      if (stop_set_.size() == 0) {
        if (paused_) {  // if robot is currently paused_, run continue
          Continue();
        } else if (pausing_) {  // if robot is currently pausing_, queue unpause
                                // cmd
          queued_cmd_ = QueuedCommand::CONTINUE;
        }
      }
    } else {
      momap::log()->info(
          "Continue command rejected: No matching pause command '{}'",
          msg->source);
    }
  }
};

void FrankaPlanRunner::Pause(const std::string& source) {
  momap::log()->info("Pausing plan.");
  paused_ = false;
  pausing_ = true;
  unpausing_ = false;
  stop_cmd_source_ = source;
  this->target_stop_time_ = 0;
  this->timestep_ = 1;
  this->stop_duration_ = 0;
  stop_margin_counter_ = 0;
}

void FrankaPlanRunner::Continue() {
  momap::log()->info("Continuing plan.");
  this->timestep_ =
      -1 * this->stop_duration_;  // how long unpausing_ should take
  momap::log()->debug("STOP DURATION: {}", stop_duration_);
  paused_ = false;
  pausing_ = false;
  unpausing_ = true;
  stop_cmd_source_ = "";
}
