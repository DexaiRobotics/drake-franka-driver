/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages contraining
/// a robot_plan_t message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

#include <iostream>
#include <memory>

#include <lcm/lcm-cpp.hpp>
#include <momap/momap_robot_plan_v1.h>
#include <lcmtypes/robot_spline_t.hpp>
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "iiwa_common.h" //eventually get from drake binary distro, when it is included
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "trajectory_solver.h"
#include "log_momap.h"
#include <dracula_utils.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace du = dracula_utils;

namespace drake {
namespace dynamap {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kLcmStopChannel = "STOP";
const int kNumJoints = 7;

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef PPType::PolynomialType PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

class RobotPlanRunner {
public:
    /// tree is aliased
    explicit RobotPlanRunner(const RigidBodyTree<double>& tree) : tree_(tree), plan_number_(0)
    {
        iiwa_drake::simulations::VerifyIiwaTree(tree);
        lcm_.subscribe(kLcmStatusChannel, &RobotPlanRunner::HandleStatus, this);
        lcm_.subscribe(kLcmPlanChannel, &RobotPlanRunner::HandlePlan, this);
        lcm_.subscribe(kLcmStopChannel, &RobotPlanRunner::HandleStop, this);
    }

    void Run() {
        int cur_plan_number = plan_number_;
        int64_t cur_time_us = -1;
        int64_t start_time_us = -1;

        // Initialize the timestamp to an invalid number so we can detect
        // the first message.
        iiwa_status_.utime = cur_time_us;

        lcmt_iiwa_command iiwa_command;
        iiwa_command.num_joints = kNumJoints;
        iiwa_command.joint_position.resize(kNumJoints, 0.);
        iiwa_command.num_torques = 0;
        iiwa_command.joint_torque.resize(kNumJoints, 0.);

        while (true) {
            // Call lcm handle until at least one status message is
            // processed.
            while (0 == lcm_.handleTimeout(10) || iiwa_status_.utime == -1) { }

            cur_time_us = iiwa_status_.utime;

            if (plan_) {
                if (plan_number_ != cur_plan_number) {
                    momap::log()->info("Starting new plan.");
                    start_time_us = cur_time_us;
                    cur_plan_number = plan_number_;
                }

                const double cur_traj_time_s =
                static_cast<double>(cur_time_us - start_time_us) / 1e6;
                const auto desired_next = plan_->value(cur_traj_time_s);

                iiwa_command.utime = iiwa_status_.utime;

                for (int joint = 0; joint < kNumJoints; joint++) {
                    iiwa_command.joint_position[joint] = desired_next(joint);
                }

                lcm_.publish(kLcmCommandChannel, &iiwa_command);
            }
        }
    }

private:
    void HandleStatus(const lcm::ReceiveBuffer*, const std::string&, const lcmt_iiwa_status* status)
    {
        iiwa_status_ = *status;
    }


    void HandlePlan(const lcm::ReceiveBuffer*, const std::string&,
        const robot_spline_t* rst)
    {
        momap::log()->info("New plan received.");
        if (iiwa_status_.utime == -1) {
            momap::log()->info("Discarding plan, no status message received yet");
            return;
        }

        piecewise_polynomial = TrajectorySolver::RobotSplineTToPPType(*rst);
        if (piecewise_polynomial.get_number_of_segments()<1)
        {
            momap::log()->info("Discarding plan, invalid piecewise polynomial.");
            return;
        }

        momap::log()->info("utime: {}", rst->utime);
        momap::log()->info("start time: {}", piecewise_polynomial.start_time());
        momap::log()->info("end time: {}", piecewise_polynomial.end_time());

        //Naive implementation of velocity check
        const int velocity_check_samples = 100;
        PPType piecewise_polynomial_derivative = piecewise_polynomial.derivative();
        double step_size = (piecewise_polynomial.end_time()-piecewise_polynomial.start_time())/velocity_check_samples; //FIXME: numer
        double test_time = piecewise_polynomial.start_time();
        while(test_time<piecewise_polynomial.end_time())
        {
            VectorXd sampled_velocity = piecewise_polynomial_derivative.value(test_time);
            //Check dimension
            if (sampled_velocity.size()!= rst->dof)
            {
                momap::log()->info("Discarding plan, invalid piecewise polynomial derivative.");
                return;
            }
            for (int joint = 0; joint < rst->dof; joint++)
            {
                if (sampled_velocity(joint)>rst->robot_joints[joint].velocity_upper_limit||
                sampled_velocity(joint)<rst->robot_joints[joint].velocity_lower_limit)
                {
                    momap::log()->info("Discarding plan, joint velocity out of bounds.");
                    momap::log()->info("sampled joint {} velocity: {}", joint, sampled_velocity(joint));
                    momap::log()->info("lower limit: {} upper limit: {}", rst->robot_joints[joint].velocity_lower_limit, rst->robot_joints[joint].velocity_upper_limit);
                    return;
                }
            }
            test_time+=step_size;
        }

        //Start position == goal position check
        VectorXd commanded_start = piecewise_polynomial.value(piecewise_polynomial.start_time());
        for (int joint = 0; joint < rst->dof; joint++)
        {
            if (!du::EpsEq(commanded_start(joint),iiwa_status_.joint_position_measured[joint], 0.05))//FIXME: non-arbitrary tolerance
            {
                momap::log()->info("Discarding plan, mismatched start position.");
                return;
            }
        }


        //TODO: add end position==goal position check (upstream)
        plan_.release();
        plan_.reset(&piecewise_polynomial);
        // PiecewisePolynomial<double>::Cubic(input_time, knots, knot_dot, knot_dot)));
        ++plan_number_;
    }

    void HandleStop(const lcm::ReceiveBuffer*, const std::string&,
        const robot_plan_t*) {
            momap::log()->info("Received stop command. Discarding plan.");
            plan_.reset();
        }

        ::lcm::LCM lcm_;
        const RigidBodyTree<double>& tree_;
        int plan_number_{};
        std::unique_ptr<PiecewisePolynomial<double>> plan_;
        lcmt_iiwa_status iiwa_status_;
        PPType piecewise_polynomial;
    };

    int do_main() {
        create_momap_log("kuka_plan_runner");
        auto tree = std::make_unique<RigidBodyTree<double>>();
        auto urdf_path = std::getenv("DEFAULT_IIWA_URDF");
        if (urdf_path == nullptr) {
            momap::log()->error("kuka_plan_runner requires the environment variable DEFAULT_IIWA_URDF to identify a valid URDF file!\n\texit(2)");
            exit(2);
        }

        // urdf_path = "drake/manipulation/models/iiwa_description/urdf/"
        //                         "iiwa14_primitive_collision.urdf";
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            urdf_path,
            multibody::joints::kFixed, tree.get()); //FindResourceOrThrow(urdf_path)

            RobotPlanRunner runner(*tree);
            runner.Run();
            return 0;
        }

        // }  // namespace
        // }  // namespace kuka_iiwa_arm
    }  // namespace dynamap
}  // namespace drake


int main() {
    return drake::dynamap::do_main();
}
