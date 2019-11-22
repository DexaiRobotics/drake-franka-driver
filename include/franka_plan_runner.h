/// @file franka_plan_runner
///
/// franka_plan_runner is designed to wait for LCM messages containing
/// a robot_plan_t message, and then execute the plan on a franka arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.
#pragma once

#include <bits/stdint-intn.h>      // for int64_t
#include <franka/control_types.h>  // for JointPositions
#include <franka/duration.h>       // for Duration
#include <franka/robot_state.h>    // for RobotState

#include <cstdint>                 // for int64_t
#include <lcmtypes/robot_spline_t.hpp>  // for robot_spline_t
#include <robot_msgs/pause_cmd.hpp>  // for pause_cmd
#include <drake/common/trajectories/piecewise_polynomial.h>  // for Piecewis...

#include <mutex>   // for mutex
#include <thread>  // for thread

#include "dracula.h"     // for Dracula
#include "parameters.h"  // for Parameters

namespace franka_driver {

using drake::trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef PPType::PolynomialType PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

struct RobotData {
  std::mutex mutex;
  bool has_data;
  franka::RobotState robot_state;
};

struct RobotPiecewisePolynomial {
  std::mutex mutex;
  std::atomic<bool> has_data;
  int64_t utime;
  std::unique_ptr<PiecewisePolynomial<double>> plan;
  int64_t end_time_us;
};

enum class QueuedCommand { NONE, PAUSE, CONTINUE };

class FrankaPlanRunner {
 public:
  FrankaPlanRunner(const parameters::Parameters params);
  ~FrankaPlanRunner(){};
  int Run();

 protected:
  int RunFranka();
  int RunSim();

  double StopPeriod(double period);
  void QueuedCmd();

  franka::JointPositions JointPositionCallback(
      const franka::RobotState& robot_state, franka::Duration period);
  void HandleLcm();
  /// PublishLcmAndPauseStatus is called once by a separate thread in the Run()
  /// method It sends the robot status and the pause status. Pause status
  /// isseparate because robot status message used does not have space for
  /// indicating the contents of the pause message.
  void PublishLcmAndPauseStatus();
  void PublishPauseStatus();
  void PublishTriggerToChannel(int64_t utime, std::string lcm_channel,
                               bool success = true, std::string message = "");
  //$ check if robot is in a mode that can receive commands, i.e. not user
  // stopped or error recovery
  bool CanReceiveCommands();
  void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&,
                  const lcmtypes::robot_spline_t* rst);
  void HandleStop(const ::lcm::ReceiveBuffer*, const std::string&,
                  const robot_msgs::pause_cmd* msg);
  void Pause(const std::string& source);
  void Continue();

 private:
  Dracula* dracula = nullptr;
  std::string param_yaml_;
  parameters::Parameters p;
  std::string ip_addr_;
  std::atomic_bool running_{false};
  std::atomic_bool robot_alive_{false};
  ::lcm::LCM lcm_;
  int plan_number_{};
  int cur_plan_number{};
  int64_t cur_plan_utime_{};
  int64_t cur_time_us_{};
  int64_t start_time_us_{};
  RobotPiecewisePolynomial plan_;
  RobotData robot_data_{};
  PPType piecewise_polynomial_;

  int sign_{};
  std::atomic_bool editing_plan_{false};
  std::array<double, 16> initial_pose_;
  Eigen::MatrixXd joint_limits_;
  long timestep_;
  float target_stop_time_;
  float STOP_SCALE = 0.8;  // this should be yaml param
  float stop_duration_;
  std::mutex pause_mutex_;
  std::atomic_bool pausing_;
  std::atomic_bool paused_;
  std::atomic_bool unpausing_;
  std::string stop_cmd_source_;
  float stop_margin_counter_ = 0;
  QueuedCommand queued_cmd_ = QueuedCommand::NONE;
  std::set<std::string> stop_set_;

  Eigen::VectorXd starting_conf_;
  std::array<double, 7> starting_franka_q_;

  const double lcm_publish_rate_ = 200.0;  // Hz
  double franka_time_;
  Eigen::VectorXd max_accels_;

  std::thread lcm_publish_status_thread;
  std::thread lcm_handle_thread;

  std::string lcm_driver_status_channel_;
  std::string lcm_pause_status_channel_;
};

}  // namespace franka_driver
