///@file: util_conv.cc
#include "util_conv.h"
#include "util_math.h"

#include "momap_log.h"

#include <cmath>
#include <cstdint>

namespace utils {

    using momap::log;

    // Convenience Functions for converting between lcm, STL, and Eigen types
    //
    // void getStuffPlan, approachPlan, exitPlan, dispensePlane
    // void getGetStuffPlan(stuffgetter_plan_t, &traj, &times); setGetStuffPlan()
    // void getApproachPlan(), setApproachPlan
    // void getExitPlan(), setExitPlan
    // void getDispensePlan(), setDispensePlan()
    // setMetadata(robot_type, ...)
    // Eigen::Vector4d getQ(robot_state_t) & setQ(robot_state_t)
    // Eigen::Vector3d getPos(robot_state_t) & setPos(robot_state_t)  ---   EETraj


    // // @deprecated: no longer used.
    // template <> void PlanToTrajectory( std::vector<momap::robot_state_t> plan
    //                                  , std::vector<Eigen::VectorXd>& trajectory
    //                                  , std::vector<double>& times_in_seconds
    // ) {
    //     int num_states = plan.size();
    //     for (int i = 0; i < num_states; i++) {
    //         auto state = plan[i];
    //         assert(is_syntax_valid(state));
    //         times_in_seconds.push_back(double(state.utime) / 1.0e6);
    //         trajectory.push_back(Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(state.joint_position.data(), state.joint_position.size()));
    //     }
    // }


    drake::lcmt_iiwa_status EigenToLcmStatus(Eigen::VectorXd robot_state) {
        drake::lcmt_iiwa_status robot_status{};
        int num_joints = robot_state.size();
        struct timeval  tv;
        gettimeofday(&tv, NULL);

        robot_status.utime = int64_t(tv.tv_sec * 1e6 + tv.tv_usec); //int64_t(1000.0 * robot_state.time.toMSec());
        // q
        robot_status.num_joints = num_joints;
        robot_status.joint_position_measured = e_to_v(robot_state);
        robot_status.joint_position_commanded.resize(num_joints, 0);
        robot_status.joint_position_ipo.resize(num_joints, 0);
        robot_status.joint_velocity_estimated.resize(num_joints, 0);
        robot_status.joint_torque_measured.resize(num_joints, 0);
        robot_status.joint_torque_commanded.resize(num_joints, 0);
        robot_status.joint_torque_external.resize(num_joints, 0);

        return robot_status;
    }

    bool SortPairAscendingEigen(const std::pair<double, Eigen::VectorXd> &a,
                           const std::pair<double, Eigen::VectorXd> &b)
    {
        return (a.first < b.first);
    }

    bool SortPairAscendingInt(const std::pair<double, int> &a,
                           const std::pair<double, int> &b)
    {
        return (a.first < b.first);
    }

    bool SplitTrajectory( const std::vector<Eigen::VectorXd>& trajectory
                        , const Eigen::VectorXd& location_to_split
                        , std::vector<Eigen::VectorXd>& front_traj
                        , std::vector<Eigen::VectorXd>& back_traj
    ) {
        // clear output variables
        front_traj.clear();
        back_traj.clear();
        // verify that there is a trajectory to split
        if (trajectory.empty()) {
            log()->error("trajectory must not be empty!");
            // TODO @dmsj: format the error nicely
            // TODO @dmsj: test throwing exceptions here
            // throw std::runtime_error("SplitTrajectory: input trajectory is empty!");
            return false;
        }
        // verify that I haven't gone senile
        if (trajectory.back().size() < 1) {
            log()->error("trajectory dimension {} < 1", trajectory.back().size());
            // TODO @dmsj: format the error nicely
            // TODO @dmsj: test throwing exceptions here
            // throw std::runtime_error("SplitTrajectory: input trajectory element size is < 1!");
            return false;
        }
        // verify that split location has a matching number of dimensions
        if (trajectory.back().size() != location_to_split.size()) {
            log()->error("split point dim {} and trajectory dim {} must be the same!",
                        location_to_split.size(), trajectory.back().size());
            // TODO @dmsj: format the error nicely
            // TODO @dmsj: test throwing exceptions here
            // throw std::runtime_error("SplitTrajectory: split element size must match trajectory element size!");
            return false;
        }
        
        auto split_location = std::find_if( trajectory.begin(), trajectory.end()
                                          , CompareConfiguration(location_to_split));
        // avoid a copy??!?!
        std::vector<Eigen::VectorXd> first_part(trajectory.begin(), split_location);
        std::vector<Eigen::VectorXd> second_part(split_location, trajectory.end());
        front_traj = first_part;
        back_traj = second_part;
        if (split_location != std::end(trajectory) and split_location != std::begin(trajectory)) {
            return true;
        }
        if (split_location == std::end(trajectory) || split_location == std::begin(trajectory)) {
          // TODO @dmsj: format the error nicely
          log()->error("split_location == std::end(trajectory): {} or split_location == std::begin(trajectory): {}",
                        split_location == std::end(trajectory), split_location == std::begin(trajectory));
          // TODO @dmsj: test throwing exceptions here
          // throw std::runtime_error("SplitTrajectory failed to find a split location which was not"
          //                         " either the beginning or the end. Throwing!");
        }
        
        return false;
    }

    // Split at nearest found location,
    // else try splitting at the location_to_split parameter value
    bool SplitAtClosest( const std::vector<Eigen::VectorXd>& trajectory
                       , const Eigen::VectorXd& location_to_split
                       , const double tolerance
                       , std::vector<Eigen::VectorXd>& front_traj
                       , std::vector<Eigen::VectorXd>& back_traj
    ) {
        front_traj.clear();
        back_traj.clear();
        if (trajectory.empty()) {
            log()->error("trajectory must not be empty!");
            return false;
        }
        if (trajectory.back().size() < 1) {
            log()->error("trajectory dimension {} < 1", trajectory.back().size());
            return false;
        }
        if (trajectory.back().size() != location_to_split.size()) {
            log()->error("split point dim {} and trajectory dim {} must be the same!"
                        , location_to_split.size(), trajectory.back().size());
            return false;
        }
        std::vector<std::pair<double, Eigen::VectorXd>> sorted_dist_to_split_location;
        for (const auto& conf : trajectory) {
            double dist_to_split_location = (conf - location_to_split).norm();
            sorted_dist_to_split_location.push_back(std::make_pair(dist_to_split_location, conf) );
        }
        // sort in descending order for longest full_scoop first
        sort(sorted_dist_to_split_location.begin(), sorted_dist_to_split_location.end(), SortPairAscendingEigen);
        double shortest_dist = sorted_dist_to_split_location.front().first;
        if (shortest_dist > tolerance) {
            // If nearest location is over tolerance, try to split at the original location_to_split:
            return SplitTrajectory(trajectory, location_to_split, front_traj, back_traj);
        }
        auto new_split_location = sorted_dist_to_split_location.front().second;
        return SplitTrajectory(trajectory, new_split_location, front_traj, back_traj);
    }

}   // namespace utils
