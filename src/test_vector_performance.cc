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
