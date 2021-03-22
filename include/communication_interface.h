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
/// If a pause message is received, it will set the pause status to true and
/// keep track of what source paused it.

#include <cstdint>  // for int64_t
#include <mutex>    // for mutex
#include <thread>   // for thread

#include <bits/stdint-intn.h>           // for int64_t
#include <lcm/lcm-cpp.hpp>              // for lcm
#include <lcmtypes/robot_spline_t.hpp>  // for robot_spline_t
#include <robot_msgs/pause_cmd.hpp>     // for pause_cmd
#include <robot_msgs/bool_t.hpp>        // for bool_t

#include "drake/common/trajectories/piecewise_polynomial.h"  // for Piecewis...
#include "franka/robot_state.h"                              // for RobotState
#include "robot_parameters.h"  // for RobotParameters

using drake::trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;

namespace franka_driver {

// TODO: remove this franka specific state and make it generic:
struct RobotData {
  std::atomic<bool> has_robot_data_;
  franka::RobotState robot_state;
  Eigen::VectorXd robot_plan_next_conf;
};

struct PauseData {
  std::atomic<bool> paused_;
  std::set<std::string> pause_sources_set_;
};

struct RobotPiecewisePolynomial {
  std::atomic<bool> has_plan_data_;
  int64_t utime;
  std::unique_ptr<PPType> plan_;
};

class CommunicationInterface {
 public:
  CommunicationInterface(const RobotParameters params,
                         double lcm_publish_rate = 200.0 /* Hz */);
  ~CommunicationInterface() {};
  void StartInterface();
  void StopInterface();

  bool SimControlExceptionTriggered() const {
    return sim_control_exception_triggered_;
  };
  void ClearSimControlExceptionTrigger() {
    sim_control_exception_triggered_ = false;
  };

  bool HasNewPlan();
  void TakePlan(std::unique_ptr<PPType>& plan, int64_t& plan_utime);

  // TODO: remove franka specific RobotState type and replace with std::array
  franka::RobotState GetRobotState();

  // acquire mutex lock and return robot mode
  franka::RobotMode GetRobotMode();

  /// Blocking call that sets the robot state
  void SetRobotData(const franka::RobotState& robot_state,
                    const Eigen::VectorXd& robot_plan_next_conf);
  /// Non-blocking call that sets the robot state if possible
  void TryToSetRobotData(const franka::RobotState& robot_state,
                         const Eigen::VectorXd& robot_plan_next_conf);

  bool GetPauseStatus();
  void SetPauseStatus(bool paused);

  void PublishPlanComplete(const int64_t& plan_utime, bool success = true,
                           std::string driver_status_string = "");

  void PublishDriverStatus(bool success, std::string driver_status_string = "");
  void PublishBoolToChannel(int64_t utime, std::string_view lcm_channel,
                            bool data);
  void PublishPauseToChannel(int64_t utime, std::string_view lcm_channel,
                             bool data,
                             std::string_view source = "");

  std::string GetUserStopChannelName() { return lcm_user_stop_channel_; };

 protected:
  void ResetData();
  void HandleLcm();
  /// PublishLcmAndPauseStatus is called once by a separate thread in the Run()
  /// method It sends the robot status and the pause status. Pause status
  /// is separate because robot status message used does not have space for
  /// indicating the contents of the pause message.
  void PublishLcmAndPauseStatus();
  void PublishRobotStatus();
  void PublishPauseStatus();
  void PublishTriggerToChannel(int64_t utime, std::string_view lcm_channel,
                               bool success = true,
                               std::string_view message = "");
  /// check if robot is in a mode that can receive commands, i.e. not user
  /// stopped or error recovery
  bool CanReceiveCommands(const franka::RobotMode& current_mode);
  void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&,
                  const lcmtypes::robot_spline_t* robot_spline);
  void HandlePause(const ::lcm::ReceiveBuffer*, const std::string&,
                   const robot_msgs::pause_cmd* pause_cmd_msg);

  /// Handler for control exception and u-stop triggers for simulated driver so
  /// we can test full spectrum of driver states.
  void HandleSimDriverEventTrigger(
      const ::lcm::ReceiveBuffer*, const std::string&,
      const robot_msgs::pause_cmd* trigger_cmd_msg);

 private:
  RobotParameters params_;
  std::atomic_bool running_ {false};
  std::atomic<bool> sim_control_exception_triggered_ {false};

  ::lcm::LCM lcm_;

  RobotPiecewisePolynomial robot_plan_;
  std::mutex robot_plan_mutex_;

  RobotData robot_data_;
  std::mutex robot_data_mutex_;

  PauseData pause_data_;
  std::mutex pause_mutex_;

  std::thread lcm_publish_status_thread_;
  std::thread lcm_handle_thread_;

  std::string lcm_driver_status_channel_;
  std::string lcm_pause_status_channel_;
  std::string lcm_brakes_locked_channel_;
  std::string lcm_sim_driver_event_trigger_channel_;
  std::string lcm_user_stop_channel_;

  double lcm_publish_rate_;  // Hz
};

}  // namespace franka_driver
