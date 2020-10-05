franka::JointPositions FrankaPlanRunner::JointPositionCallback(
    const franka::RobotState& robot_state, franka::Duration period) {
  // check pause status and update franka_time_:
  IncreaseFrankaTimeBasedOnStatus(robot_state.dq, period.toSec());

  franka::RobotState robot_state_mutable = robot_state; // as reported by Franka

  // auto canonical_robot_state = ApplyJointOffsets(robot_state, joint_pos_offset_); 
  utils::ApplyOffsets(robot_state_mutable.q, joint_pos_offset_); // adds --> now cannonical
  utils::ApplyOffsets(robot_state_mutable.q_d, joint_pos_offset_); // q_desired

  // read out robot state
  franka::JointPositions output_to_franka = robot_state_mutable.q_d;
  auto q_d_vec = ArrayToVector(robot_state_mutable.q_d);
  Eigen::VectorXd current_commanded_cannonical_conf_franka = utils::v_to_e(q_d_vec);

  // the current (desired) position of franka is the starting position:
  auto q_vec = ArrayToVector(robot_state_mutable.q);
  

  // Set robot state for LCM publishing:
  // TODO @rkk: do not use franka robot state but use a generic Eigen instead

  static bool first_run = true;

  // if (first_run && status_ == RobotStatus::Reversing) {
  //   start_reversing_conf_franka_ = current_commanded_cannonical_conf_franka;
  //   first_run = false;
  // }

  if (comm_interface_->HasNewPlan()) { // && status_ != RobotStatus::Reversing) {
    // get the current plan from the communication interface
    comm_interface_->TakePlan(plan_, plan_utime_);

    // auto plan_received_time = std::chrono::high_resolution_clock::now();
    // int64_t plan_received_utime = (std::chrono::time_point_cast<
    // std::chrono::microseconds > (plan_received_time)
    // ).time_since_epoch().count(); 
    // auto plan_time_delta = plan_received_utime - plan_utime_; 
    // if(plan_time_delta ) {}

    start_conf_franka_ = utils::v_to_e(q_vec);
    
    // first time step of plan, reset time:
    franka_time_ = 0.0;
    start_conf_plan_ = plan_->value(franka_time_);  // TODO @rkk: fails

    if (!LimitJoints(start_conf_plan_)) {
      dexai::log()->warn(
          "JointPositionCallback: plan {} at franka_time_: {} seconds "
          "is exceeding the joint limits!",
          plan_utime_, franka_time_);
    }

    

    end_conf_plan_ = plan_->value(plan_->end_time());
    // TODO @rkk: move this print into another thread
    dexai::log()->debug("JointPositionCallback: starting franka q = {}",
                        utils::v_to_e(q_vec).transpose());
    dexai::log()->debug("JointPositionCallback: starting franka q_d = {}",
                        utils::v_to_e(q_d_vec).transpose());
    dexai::log()->debug("JointPositionCallback: starting plan q_d = {}",
                        start_conf_plan_.transpose());

    // Maximum change in joint angle between two confs
    auto max_ang_distance =
        utils::max_angular_distance(utils::v_to_e(q_d_vec), start_conf_plan_);
    if (max_ang_distance > params_.kMediumJointDistance) {
      dexai::log()->error(
          "JointPositionCallback: Discarding plan, mismatched start position."
          " Max distance: {} > {}",
          max_ang_distance, params_.kTightJointDistance);
      return franka::MotionFinished(output_to_franka);
    } else if (max_ang_distance > params_.kTightJointDistance) {
      dexai::log()->warn(
          "JointPositionCallback: max angular distance between franka and "
          "start of plan is larger than 'kTightJointDistance': {} > {}",
          max_ang_distance, params_.kTightJointDistance);
    }
  }

  if (!plan_) {
    dexai::log()->debug(
        "JointPositionCallback: No plan exists (anymore), exiting "
        "controller...");
    comm_interface_->TryToSetRobotData(robot_state_mutable, start_conf_franka_);
    return franka::MotionFinished(output_to_franka);
  }

  const auto plan_end_time = plan_->end_time();
  const auto plan_completion_fraction = std::min(1.0, std::max(0.0, franka_time_/plan_end_time));

  // read out plan for current franka time from plan:
  next_conf_plan_ = plan_->value(franka_time_);
  if (!LimitJoints(next_conf_plan_)) {
    dexai::log()->warn(
        "JointPositionCallback: plan at {}s is exceeding the joint limits!",
        franka_time_);
  }

  comm_interface_->TryToSetRobotData(robot_state_mutable, next_conf_plan_);

  // delta between conf at start of plan to conft at current time of plan:
  Eigen::VectorXd delta_conf_plan = next_conf_plan_ - start_conf_plan_;

  // add delta to current robot state to achieve a continuous motion:
  Eigen::VectorXd next_conf_franka = start_conf_franka_ + delta_conf_plan;
  // Eigen::VectorXd next_conf_franka = current_commanded_cannonical_conf_franka + delta_conf_plan;

  // Linear interpolation between next conf with offset and the actual next conf based on received plan
  Eigen::VectorXd next_conf_combined = (1.0 - plan_completion_fraction) * next_conf_franka + plan_completion_fraction * next_conf_plan_;
  

  // overwrite the output_to_franka of this callback:
  Eigen::VectorXd next_conf_combined_corrected = next_conf_combined - joint_pos_offset_;
  if (!LimitJoints(next_conf_combined_corrected)) {
      dexai::log()->warn(
          "JointPositionCallback: plan {} at franka_time_: {} seconds "
          "is exceeding the joint limits!",
          plan_utime_, franka_time_);
  }
  if(plan_completion_fraction < 0.01) {
    dexai::log()->debug("JointPositionCallback: starting plan q = {}",
                        start_conf_franka_.transpose());
    dexai::log()->debug("JointPositionCallback: next_conf_combined = {}",
                        next_conf_combined.transpose());
    dexai::log()->debug("JointPositionCallback: next_conf_combined_corrected = {}",
                        next_conf_combined_corrected.transpose());
    dexai::log()->debug("JointPositionCallback: current q_d = {}",
                        utils::v_to_e(ArrayToVector(robot_state.q_d)).transpose());
  }
  output_to_franka = utils::EigenToArray(next_conf_combined_corrected);

  if (franka_time_ > plan_->end_time()) {

    // Maximum change in joint angle between two confs
    double error_final = utils::max_angular_distance(end_conf_plan_, current_commanded_cannonical_conf_franka);

    if (error_final < allowable_max_angle_error_) {
      dexai::log()->info(
          "JointPositionCallback: Finished plan {}, exiting controller",
          plan_utime_);
      comm_interface_->PublishPlanComplete(plan_utime_, true /* = success */);
    } else {

      auto error_eigen = (end_conf_plan_ - current_commanded_cannonical_conf_franka).cwiseAbs();
      for (size_t joint_idx = 0; joint_idx < dof_; joint_idx++ ) {
        if (error_eigen(joint_idx) > allowable_max_angle_error_) {
          dexai::log()->warn(
              "JointPositionCallback: Overtimed plan {}: robot diverged, joint {} "
              "error: {} - {} = {} > max allowable: {}",
              plan_utime_, joint_idx, end_conf_plan_(joint_idx),
              current_commanded_cannonical_conf_franka(joint_idx), error_eigen(joint_idx),
              allowable_max_angle_error_);
        }
      }

      dexai::log()->info("JointPositionCallback: current_conf_franka: {}",
                         current_commanded_cannonical_conf_franka.transpose());
      dexai::log()->info("JointPositionCallback: next_conf_franka: {}",
                         next_conf_franka.transpose());
      dexai::log()->info("JointPositionCallback: next_conf_plan: {}",
                         next_conf_plan_.transpose());
      dexai::log()->info("JointPositionCallback: next_conf_combined_corrected: {}",
                         next_conf_combined_corrected.transpose());
      comm_interface_->PublishPlanComplete(plan_utime_, false /*  = failed*/,
                                           "diverged");
    }
    // releasing finished plan:
    plan_.release();
    plan_utime_ = -1;  // reset plan to -1
    return franka::MotionFinished(output_to_franka);
  }
  return output_to_franka;
}