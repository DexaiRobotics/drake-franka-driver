#pragma once
/// @file communication_interface
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

#include "dracula.h"                                         // for Dracula
#include "drake/common/trajectories/piecewise_polynomial.h"  // for Piecewis...
#include "franka/robot_state.h"                              // for RobotState
#include "parameters.h"                                      // for Parameters

#include <bits/stdint-intn.h>           // for int64_t
#include <cstdint>                      // for int64_t
#include <lcmtypes/robot_spline_t.hpp>  // for robot_spline_t
#include <mutex>                        // for mutex
#include <robot_msgs/pause_cmd.hpp>     // for pause_cmd
#include <thread>                       // for thread

using drake::trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;

namespace franka_driver {

// TODO @rkk: remove this franka specific state and make it generic:
struct RobotData {
  std::atomic<bool> has_robot_data_;
  franka::RobotState robot_state;
};

struct PauseData {
  std::atomic<bool> paused_;
  std::set<std::string> stop_set_;
};

struct RobotPiecewisePolynomial {
  std::atomic<bool> has_plan_data_;
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

  // TODO @rkk: remove franka specific RobotState type 
  // and replace with std::array type:
  franka::RobotState GetRobotState();
  void SetRobotState(const franka::RobotState& robot_state);
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
  void PublishRobotStatus();
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
