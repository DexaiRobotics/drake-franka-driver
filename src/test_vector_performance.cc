#include "StopWatch.h" // https://github.com/KjellKod/Stopwatch

#include "franka_plan_runner.h"

using namespace std;

namespace drake {
namespace franka_driver {

struct TestRunAggregater
{
  double test1;
  double test2;
  double test3;

  void Reset()
  {
    test1 = test2 = test3 = 0;
  }
};

void ResizeStatusMessage(lcmt_iiwa_status &lcm_status_){
  lcm_status_.utime = -1;
  lcm_status_.num_joints = kNumJoints;
  lcm_status_.joint_position_measured.resize(kNumJoints, 0);
  lcm_status_.joint_position_commanded.resize(kNumJoints, 0);
  lcm_status_.joint_position_ipo.resize(kNumJoints, 0);
  lcm_status_.joint_velocity_estimated.resize(kNumJoints, 0);
  lcm_status_.joint_torque_measured.resize(kNumJoints, 0);
  lcm_status_.joint_torque_commanded.resize(kNumJoints, 0);
  lcm_status_.joint_torque_external.resize(kNumJoints, 0);
}
// Fuctions to show the benefit of vector::reserve()
void FillVector(vector<lcmt_iiwa_status>& testVector)
{
  for (int i = 0; i < 100; i++)
  {
    lcmt_iiwa_status bt;
    testVector.push_back(bt);
  }
}
void FillVectorPreAllocate(vector<lcmt_iiwa_status>& testVector)
{
  for (int i = 0; i < 100; i++)
  {
    lcmt_iiwa_status bt;
    ResizeStatusMessage(bt);
    testVector.push_back(bt);
  }
}
void Populate(vector<lcmt_iiwa_status>& testVector){
  for(auto &status : testVector){
    franka::RobotState rs; 
    status = ConvertToLcmStatus(rs);
  }
}

void AssignToLcmStatus(franka::RobotState &robot_state, lcmt_iiwa_status &robot_status){
    int num_joints_ = kNumJoints;
    struct timeval  tv;
    gettimeofday(&tv, NULL);

    robot_status.utime = int64_t(tv.tv_sec * 1e6 + tv.tv_usec); //int64_t(1000.0 * robot_state.time.toMSec());
    robot_status.num_joints = num_joints_;
    // q
    robot_status.joint_position_measured.assign(std::begin(robot_state.q), std::end(robot_state.q)) ;
    robot_status.joint_position_commanded.assign(std::begin(robot_state.q_d), std::end(robot_state.q_d)) ; // = ConvertToVector(robot_state.q_d);
    robot_status.joint_position_ipo.resize(num_joints_, 0);
    robot_status.joint_velocity_estimated.assign(std::begin(robot_state.dq), std::end(robot_state.dq)) ;// = ConvertToVector(robot_state.dq);
    robot_status.joint_torque_measured.assign(std::begin(robot_state.tau_J), std::end(robot_state.tau_J)) ; // = ConvertToVector(robot_state.tau_J);
    robot_status.joint_torque_commanded.assign(std::begin(robot_state.tau_J_d), std::end(robot_state.tau_J_d)) ; // = ConvertToVector(robot_state.tau_J_d);
    robot_status.joint_torque_external.resize(num_joints_, 0);

}

void PopulateByAssign(vector<lcmt_iiwa_status>& testVector){
  for(auto &status : testVector){
    franka::RobotState rs; 
    AssignToLcmStatus(rs, status);
  }
}

int do_main()
{
  StopWatch sw;

  TestRunAggregater tg;
  tg.test1 = 0;
  tg.test2 = 0;
  tg.test3 = 0;

  // #1: use convert array and vector constructure

  // #2: pre-allocate and copy

  // #3: pre-allocate and .assign

  // #1: Avoid unnecessary reallocate and copy cycles by reserving the size of vector ahead of time.
  vector<lcmt_iiwa_status> testVector1;
  vector<lcmt_iiwa_status> testVector2;

  for (int i = 0; i < 100; i++)
  {
    FillVector(testVector1);
    FillVectorPreAllocate(testVector2);
    sw.Restart();
    Populate(testVector1);
    tg.test1 += sw.ElapsedUs();


    sw.Restart();
    PopulateByAssign(testVector2);
    tg.test2 += sw.ElapsedUs();
  }

  cout << "Average Time to Fill Vector Without Reservation:" << (tg.test1 / 100) << endl;
  cout << "Average Time to Fill Vector With Reservation:" << (tg.test2 / 100) << endl;

  tg.Reset();

  return 0;
}
} // franka_driver
} // drake

int main(int argc, char** argv) {
    if (argc != 1) {
        std::cerr << "Usage: " << argv[0] << std::endl;
        return -1;
    }
    return drake::franka_driver::do_main();
}
