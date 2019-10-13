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
#include <franka/model.h>
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"


using drake::multibody::Parser;

namespace drake {
namespace franka_driver {

int do_main(std::string param_yaml="franka_test.yaml") {
    create_momap_log("franka_plan_runner");
    int verbose = 0;
    momap::log()->info("Loading parameters: {}", param_yaml);
    parameters::Parameters params = parameters::loadYamlParameters(param_yaml, verbose);
    FrankaPlanRunner frankaPlanRunner(params);
    return frankaPlanRunner.Run();
}

void FrankaPlanRunner::MultibodySetUp(drake::multibody::MultibodyPlant<double> &mb_plant_
                                , std::unique_ptr<drake::systems::Context<double>> &mb_plant_context_
                                , const std::string urdf_path){
    Parser parser(&mb_plant_);
    parser.AddModelFromFile(urdf_path, "mb_plant_");
    mb_plant_.Finalize();
    mb_plant_context_ = mb_plant_.CreateDefaultContext();
}

franka::Torques FrankaPlanRunner::InverseDynamicsControlCallback(const franka::RobotState& robot_state, franka::Duration period){
    franka::Torques output = robot_state.tau_J; // TODO: initialize to something else
    momap::log()->debug("entering callback");
    if ( plan_.mutex.try_lock() ) {
        // we got the lock, so try and do stuff.
        // momap::log()->info("got the lock!");
        // FrankaPlanRunner::check_franka_pause();

        if (pausing_) {
            if (target_stop_time_ == 0) { //if target_stop_time_ not set, set target_stop_time_
                std::array<double,7> vel = robot_state.dq;
                float temp_target_stop_time_ = 0;
                for (int i = 0; i < 7; i++) {
                    float stop_time = fabs(vel[i] / (this->max_accels_[i])); //sets target stop_time in plan as max(vel_i/max_accel_i), where i is each joint. real world stop time ~ 2x stop_time in plan
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

            if (new_stop >= period.toSec() * p.stop_epsilon) { // robot counts as "stopped" when new_stop is less than a fraction of period
                this->stop_duration_++;
            }
            else if (stop_margin_counter_ <= p.stop_margin) { // margin period after pause before robot is allowed to continue
                stop_margin_counter_ += period.toSec();
            }
            else{
                paused_ = true;
                QueuedCmd();
            }
            
            
        } else if (unpausing_) { //robot is unpausing_
            if (timestep_ >= 0) { //if robot has reached full speed again
                unpausing_ = false;
                QueuedCmd();
            }
            double new_stop = StopPeriod(period.toSec());
            franka_time_ += new_stop;
            momap::log()->debug("CONTINUE PERIOD: {}", new_stop);
            timestep_++;
            
        }
        else {
            franka_time_ += period.toSec();
        }

        if (plan_.plan && plan_number_ != cur_plan_number) {
            momap::log()->info("Starting new plan at {} s.", franka_time_);
            start_time_us_ = cur_time_us_; // implies that we should have call motion finished
            cur_plan_number = plan_number_;
            starting_conf_ = plan_.plan->value(0.0);
            starting_franka_q_ = robot_state.q_d; 
            momap::log()->warn("starting franka q = {}", du::v_to_e( ConvertToVector(starting_franka_q_) ).transpose()); 
            momap::log()->warn("difference between where we are and where we think = {}", 
                                ( du::v_to_e( ConvertToVector(starting_franka_q_) ) - starting_conf_ ).norm() );
        }   

        if (robot_data_.mutex.try_lock()) {
            robot_data_.has_data = true;
            robot_data_.robot_state = robot_state;
            robot_data_.mutex.unlock();
        }

        if(plan_.plan){//below is inverseDynamics code
            const Eigen::VectorXd kp = Eigen::VectorXd::Ones(kNumJoints)*0;
            const Eigen::VectorXd ki = Eigen::VectorXd::Ones(kNumJoints)*0;
            const Eigen::VectorXd kd = Eigen::VectorXd::Ones(kNumJoints)*0;

            if(franka_time_ == 0) integral_error =  Eigen::VectorXd::Zero(kNumJoints); // initialize integral error to 0 at start time

            Eigen::VectorXd desired_vd = Eigen::VectorXd::Zero(kNumJoints);
            // Eigen::Map<const Eigen::Matrix<double, 7, 1> > pos_desired(robot_state.q_d.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1> > pos_actual(robot_state.q.data());

            //Robert : for testing can maintain one position
            if(runonce){
                pos_start = pos_actual;
                momap::log()->info("set pos_start to = {}", pos_start);
                runonce = false;
            }
            Eigen::Matrix<double, 7, 1> pos_desired = pos_start;

            integral_error += pos_desired - pos_actual;
            Eigen::Map<const Eigen::Matrix<double, 7, 1> > vel_desired(robot_state.dq_d.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1> > vel_actual(robot_state.dq.data());
            // auto t1 = std::chrono::system_clock::now(); // for timing purposes
            // TODO : FIX SEGFAULT FOR REF_VD INITIALIZE/INSTANTIATE
            // TODO : need ref_vd because doing the derivative of polynomial in realtime to get ref acceleration is too slow and takes 2.3 ms
            // desired_vd = ref_vd_.value(franka_time) +
            desired_vd = kp.cwiseProduct(pos_desired - pos_actual)+ kd.cwiseProduct(vel_desired - vel_actual) + ki.cwiseProduct(integral_error) ;
            // for gravity compensation, desired_vd should be zero

            Eigen::VectorXd tau = Eigen::VectorXd::Zero(kNumJoints);
            Eigen::VectorXd current_state(14);
            current_state << pos_actual, vel_actual;
            mb_plant_.SetPositionsAndVelocities(mb_plant_context_.get(), current_state);

            multibody::MultibodyForces<double> external_forces(mb_plant_);
            // Isabelle's old version : account for all external forces ( including gravity )
            // Eigen::Map<const Eigen::Matrix<double, 7, 1> > tau_cmd(robot_state.tau_J_d.data());          // convert std::array to eigen vector
            // Eigen::Map<const Eigen::Matrix<double, 7, 1> > tau_measured(robot_state.tau_J.data());      // the signs are flipped on these because the link side is negative sign of motor side
            // external_forces.mutable_generalized_forces() = tau_cmd - tau_measured; // external_forces = external torques we sense on the robot


            // Robert;s version : only account for gravity
            mb_plant_.CalcForceElementsContribution( *mb_plant_context_, &external_forces); //takes care of reading gravity
            momap::log()->debug("gravity = {}", external_forces.generalized_forces().transpose());

            // potentially useful methods : get external force (not including gravity) from robot sensors
            // Eigen::Map<const Eigen::Matrix<double, 7, 1> > tau_ext(robot_state.tau_ext_hat_filtered.data());
            // external_forces.mutable_generalized_forces() = tau_ext;
            tau = mb_plant_.CalcInverseDynamics(*mb_plant_context_, desired_vd, external_forces); //calculates the M(q) + C(q) - tau_ap
            // auto t4 = std::chrono::system_clock::now();
            // momap::log()->info("gravity calc v2= {}",  mb_plant_.CalcGravityGeneralizedForces(*mb_plant_context_));
            // momap::log()->info("tau = {}", tau.transpose());

            //calling franka::limitrate is unnecessary because robot.control has limit_rate= default true?

            output = {{ tau[0], tau[1],
                        tau[2], tau[3],
                        tau[4], tau[5],
                        tau[6] }};
            // throw std::exception(); // stops robot before actually sending output
            // where we are now
            std::array<double, 7> current_q = robot_state.q;
            Eigen::VectorXd current_conf = du::v_to_e( ConvertToVector(current_q));
            // where we want to go
            Eigen::VectorXd desired_end_conf = plan_.plan->value(plan_.plan->end_time());
            // norm distance from desired_end_conf
            double dist_from_end = (current_conf - desired_end_conf).norm();
            if (franka_time_ > plan_.plan->end_time()) {
                if (dist_from_end < 0.007) {
                    plan_.plan.release();
                    plan_.has_data = false;
                    // plan_.utime = -1;
                    plan_.mutex.unlock();

                    PublishTriggerToChannel(plan_.utime, p.lcm_plan_complete_channel);
                    return franka::MotionFinished(output);
                } else {
                    momap::log()->info("Plan running overtime and not converged, dist: {}",
                                       dist_from_end);
                }

            }
        }
        plan_.mutex.unlock();
        momap::log()->debug("returning at end of callback");
        return output;
    }
    return franka::MotionFinished(output);
}
}  // namespace robot_plan_runner
}  // namespace drake


int main(int argc, char** argv) {
    if (argc != 1 && argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <params_filepath>" << std::endl;
        return -1;
    }

    if (argc == 1){
        momap::log()->info("Loading default parameters with sim robot: franka_test.yaml");
        return drake::franka_driver::do_main();
    } else {
        std::string param_yaml = argv[1];
        return drake::franka_driver::do_main(param_yaml);
    }
}


