#include "drake_lcmtypes/drake/lcmt_iiwa_command.hpp"
#include "drake_lcmtypes/drake/lcmt_iiwa_status.hpp"

using drake::lcmt_iiwa_command;
using drake::lcmt_iiwa_status;

namespace {

const int kNumJoints = 7;
const int kDefaultPort = 30200;
const char* kLcmStatusChannel = "FRANKA_STATUS"; // TODO: make this defined as <robot_name>_STATUS
const char* kLcmCommandChannel = "FRANKA_COMMAND"; // TODO: make this defined as <robot_name>_COMMAND
const double kTimeStep = 0.001; // must maintain this rate or franka interface will fail
const double kJointLimitSafetyMarginDegree = 0.5; // not sure how franka handles this
const double kJointTorqueSafetyMarginNm = 60;
const double kJointTorqueSafetyMarginScale[kNumJoints] = {1, 1, 1, 0.5,
                                                          0.2, 0.2, 0.1};
}

DEFINE_double(ext_trq_limit, kJointTorqueSafetyMarginNm,
              "Maximal external torque that triggers safety freeze");
DEFINE_string(joint_ext_trq_limit, "", "Specify the maximum external torque "
              "that triggers safety freeze on a per-joint basis.  "
              "This is a comma separated list of numbers e.g. "
              "100,100,53.7,30,30,28.5,10.  Overrides ext_trq_limit.");
DEFINE_int32(fri_port, kDefaultPort, "First UDP port for FRI messages");
DEFINE_int32(num_robots, 1, "Number of robots to control");
DEFINE_string(lcm_command_channel, kLcmCommandChannel,
              "Channel to receive LCM command messages on");
DEFINE_string(lcm_status_channel, kLcmStatusChannel,
              "Channel to send LCM status messages on");
DEFINE_bool(restart_fri, false,
            "Restart robot motion after the FRI signal has degraded and "
            "been restored.");

namespace franka_driver {



class FrankaLCMClient  {
 public:
  explicit FrankaLCMClient()
      : num_joints_(kNumJoints) {

    // Use -1 as a sentinal to indicate that no status has been
    // sent (or received from the robot).
    lcm_status_.utime = -1;
    lcm_status_.num_joints = num_joints_;
    lcm_status_.joint_position_measured.resize(num_joints_, 0);
    lcm_status_.joint_position_commanded.resize(num_joints_, 0);
    lcm_status_.joint_position_ipo.resize(num_joints_, 0);
    lcm_status_.joint_velocity_estimated.resize(num_joints_, 0);
    lcm_status_.joint_torque_measured.resize(num_joints_, 0);
    lcm_status_.joint_torque_commanded.resize(num_joints_, 0);
    lcm_status_.joint_torque_external.resize(num_joints_, 0);

    // Use -1 as a sentinal to indicate that no command has been
    // received.
    lcm_command_.utime = -1;

    // Set torque limits.
    if (FLAGS_joint_ext_trq_limit.empty()) {
      for (int i = 0; i < kNumJoints; ++i) {
        external_torque_limit_.push_back(
            FLAGS_ext_trq_limit * kJointTorqueSafetyMarginScale[i]);
      }
    } else {
      std::string remain = FLAGS_joint_ext_trq_limit;
      for (int i = 0; i < kNumJoints - 1; ++i) {
        const auto next_comma = remain.find(",");
        if (next_comma == std::string::npos) {
          throw std::runtime_error(
              "--joint_ext_trq_limit must contain 7 comma delimited values");
        }
        std::string next = remain.substr(0, next_comma);
        external_torque_limit_.push_back(std::stod(next));
        remain = remain.substr(next_comma + 1);
      }
      if (remain.empty() || remain.find(",") != std::string::npos) {
        throw std::runtime_error(
            "--joint_ext_trq_limit must contain 7 comma delimited values");
      }
      external_torque_limit_.push_back(std::stod(remain));
    }

    std::cerr << "Joint torque limits: ";
    PrintVector(external_torque_limit_, 0, kNumJoints, std::cerr);

    // Initialize filters.
    const double cutoff_hz = 40;
    vel_filters_.resize(
        num_joints_, DiscreteTimeLowPassFilter<double>(cutoff_hz, kTimeStep));
    utime_last_.resize(num_robots, -1);

    lcm::Subscription* sub = lcm_.subscribe(FLAGS_lcm_command_channel,
        &KukaLCMClient::HandleCommandMessage, this);
    // Only pay attention to the latest command.
    sub->setQueueCapacity(1);
  }

  void UpdateRobotState(int robot_id, const KUKA::FRI::LBRState& state) {
    const int joint_offset = robot_id * kNumJoints;
    assert(joint_offset + kNumJoints <= num_joints_);

    // Current time stamp for this robot.
    const int64_t utime_now =
        state.getTimestampSec() * 1e6 + state.getTimestampNanoSec() / 1e3;
    // Get delta time for this robot.
    double robot_dt = 0.;
    if (utime_last_.at(robot_id) != -1) {
      robot_dt = (utime_now - utime_last_.at(robot_id)) / 1e6;
      // Check timing
      if (std::abs(robot_dt - kTimeStep) > 1e-3) {
        std::cout << "Warning: dt " << robot_dt
                  << ", kTimeStep " << kTimeStep << "\n";
      }
    }
    utime_last_.at(robot_id) = utime_now;

    // The choice of robot id 0 for the timestamp is arbitrary.
    if (robot_id == 0) {
      lcm_status_.utime = utime_now;
    }

    // Velocity filtering.
    if (robot_dt != 0.) {
      for (int i = 0; i < kNumJoints; i++) {
        const int index = joint_offset + i;
        const double q_diff = state.getMeasuredJointPosition()[i] -
                              lcm_status_.joint_position_measured[index];
        lcm_status_.joint_velocity_estimated[index] =
            vel_filters_[index].filter(q_diff / robot_dt);
      }
    }

    // Set other joint states.
    std::memcpy(lcm_status_.joint_position_measured.data() + joint_offset,
                state.getMeasuredJointPosition(), kNumJoints * sizeof(double));
    std::memcpy(lcm_status_.joint_position_commanded.data() + joint_offset,
                state.getCommandedJointPosition(), kNumJoints * sizeof(double));

    // In Sunrise 1.13, getIpoJointPosition changed from returning
    // NULL if it wasn't available to throwing an exception.  Try to
    // avoid triggering the exception, but catch and ignore it if it
    // happens.  Initalize everything to NaN first in case we don't
    // make it.
    for (int i = joint_offset; i < joint_offset + kNumJoints; i++) {
      lcm_status_.joint_position_ipo[i] =
          std::numeric_limits<double>::quiet_NaN();
    }
    const KUKA::FRI::ESessionState session_state = state.getSessionState();
    if (session_state == KUKA::FRI::COMMANDING_WAIT ||
        session_state == KUKA::FRI::COMMANDING_ACTIVE) {
      try {
        if (state.getIpoJointPosition() != NULL) {
          std::memcpy(lcm_status_.joint_position_ipo.data() + joint_offset,
                      state.getIpoJointPosition(), kNumJoints * sizeof(double));
        }
      } catch (...) {
        // I (sam.creasey) would prefer to catch the specific
        // exception here, but it's not clear how to detect which
        // version of FRI you're compiling against to determine if
        // it's possible to include FRIException.h.
      }
    }

    std::memcpy(lcm_status_.joint_torque_measured.data() + joint_offset,
                state.getMeasuredTorque(), kNumJoints * sizeof(double));
    std::memcpy(lcm_status_.joint_torque_commanded.data() + joint_offset,
                state.getCommandedTorque(), kNumJoints * sizeof(double));
    std::memcpy(lcm_status_.joint_torque_external.data() + joint_offset,
                state.getExternalTorque(), kNumJoints * sizeof(double));
  }

  /// @returns true if robot @p robot_id is in a safe state. Currently only
  /// checks the external torques field.
  ///
  /// Note: Should only be called after UpdateRobotState.
  bool CheckSafety(int robot_id) const {
    const int joint_offset = robot_id * kNumJoints;
    assert(joint_offset + kNumJoints <= num_joints_);

    // Check external torque for each joint.
    for (int i = 0; i < kNumJoints; i++) {
      const double ext_torque =
          lcm_status_.joint_torque_external[joint_offset + i];
      if (std::fabs(ext_torque) > external_torque_limit_[i]) {
        return false;
      }
    }

    return true;
  }

  void PrintRobotState(int robot_id, std::ostream& out) const {
    const int joint_offset = robot_id * kNumJoints;
    assert(joint_offset + kNumJoints <= num_joints_);

    out << "Robot: " << robot_id << ", at time: " << utime_last_.at(robot_id)
        << "\n";
    out << "Position: ";
    PrintVector(lcm_status_.joint_position_measured,
                joint_offset, kNumJoints, out);
    out << "Velocity: ";
    PrintVector(lcm_status_.joint_velocity_estimated,
                joint_offset, kNumJoints, out);
    out << "Ext Torque: ";
    PrintVector(lcm_status_.joint_torque_external,
                joint_offset, kNumJoints, out);
    out << "Torque: ";
    PrintVector(lcm_status_.joint_torque_measured,
                joint_offset, kNumJoints, out);

    out << "Commanded position: ";
    PrintVector(lcm_status_.joint_position_commanded,
                joint_offset, kNumJoints, out);

    out << "Commanded torque: ";
    PrintVector(lcm_status_.joint_torque_commanded,
                joint_offset, kNumJoints, out);
  }

  /// @return true if valid command data was present, or false if no
  /// command is available (in which case the array is not modified).
  bool GetPositionCommand(int robot_id, double* pos) const {
    const int joint_offset = robot_id * kNumJoints;
    assert(joint_offset + kNumJoints <= num_joints_);

    if (lcm_command_.utime == -1) {
      return false;
    }

    assert(lcm_command_.num_joints >= num_joints_);
    memcpy(pos, lcm_command_.joint_position.data() + joint_offset,
           kNumJoints * sizeof(double));
    return true;
  }

  /// @return true if valid command data was present, or false if no
  /// command is available (in which case the array is not modified).
  bool GetTorqueCommand(int robot_id, double* torque) const {
    const int joint_offset = robot_id * kNumJoints;
    assert(joint_offset + kNumJoints <= num_joints_);

    if (lcm_command_.utime == -1) {
      return false;
    }

    if (lcm_command_.num_torques == 0) {
      return false;
    }

    assert(lcm_command_.num_torques >= num_joints_);
    memcpy(torque, lcm_command_.joint_torque.data() + joint_offset,
           kNumJoints * sizeof(double));
    return true;
  }

  int PollForCommandMessage() {
    return lcm_.handleTimeout(0);
  }

  int GetLcmFileno() {
    return lcm_.getFileno();
  }

  void PublishStateUpdate() {
    lcm_.publish(FLAGS_lcm_status_channel, &lcm_status_);
  }

 private:
  void HandleCommandMessage(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const lcmt_iiwa_command* command) {
    lcm_command_ = *command;
  }

  const int num_joints_;
  lcm::LCM lcm_;
  lcmt_iiwa_status lcm_status_{};
  lcmt_iiwa_command lcm_command_{};

  std::vector<double> external_torque_limit_;

  // Filters
  std::vector<DiscreteTimeLowPassFilter<double>> vel_filters_;
  std::vector<int64_t> utime_last_;
};