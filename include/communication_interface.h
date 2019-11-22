#pragma once
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

#include <bits/stdint-intn.h>  // for int64_t
// #include <franka/control_types.h>  // for JointPositions
// #include <franka/duration.h>       // for Duration
#include <drake/common/trajectories/piecewise_polynomial.h>  // for Piecewis...
#include <franka/robot_state.h>                              // for RobotState

#include <cstdint>                      // for int64_t
#include <lcmtypes/robot_spline_t.hpp>  // for robot_spline_t
#include <mutex>                        // for mutex
#include <robot_msgs/pause_cmd.hpp>     // for pause_cmd
#include <thread>                       // for thread

#include "dracula.h"     // for Dracula
#include "parameters.h"  // for Parameters

using drake::trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;

namespace franka_driver {

// TODO @rkk: remove this franka specific state and make it generic:
struct RobotData {
  std::atomic<bool> has_data_;
  franka::RobotState robot_state;
};

struct PauseData {
  std::atomic<bool> paused_;
  std::set<std::string> stop_set_;
};

struct RobotPiecewisePolynomial {
  std::atomic<bool> has_data_;
  int64_t utime;
  std::unique_ptr<PPType> plan_;
};

// enum class QueuedCommand { NONE, PAUSE, CONTINUE };

class CommunicationInterface {
 public:
  CommunicationInterface(const parameters::Parameters params,
                         double lcm_publish_rate = 200.0 /* Hz */);
  ~CommunicationInterface(){};
  void StartInterface();
  void StopInterface();

  bool HasNewPlan();
  void TakeOverPlan(std::unique_ptr<PPType>& plan);
  
  franka::RobotState GetRobotState();
  void TryToSetRobotState(const franka::RobotState& robot_state);

  bool GetPauseStatus();
  void SetPauseStatus(bool paused);

  void PublishPlanComplete(const int64_t& end_time_us);
  void PublishDriverStatus(bool success, std::string driver_status_string = "");

 protected:
  double StopPeriod(double period);
  // void QueuedCmd();

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
  void HandlePause(const ::lcm::ReceiveBuffer*, const std::string&,
                  const robot_msgs::pause_cmd* msg);

 private:
  parameters::Parameters params_;
  std::atomic_bool running_{false};
  ::lcm::LCM lcm_;

  RobotPiecewisePolynomial robot_plan_;
  std::mutex plan_mutex_;

  RobotData robot_data_;
  std::mutex robot_data_mutex_;

  // QueuedCommand queued_cmd_ = QueuedCommand::NONE;
  PauseData pause_data_;
  std::mutex pause_mutex_;

  std::thread lcm_publish_status_thread_;
  std::thread lcm_handle_thread_;

  std::string lcm_driver_status_channel_;
  std::string lcm_pause_status_channel_;
  double lcm_publish_rate_;  // Hz

};

}  // namespace franka_driver
