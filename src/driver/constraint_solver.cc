/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Dexai Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// @file  constraint_solver.cc
#include "driver/constraint_solver.h"

#include <unistd.h>

#include <drake/geometry/drake_visualizer.h>
#include <drake/math/quaternion.h>
#include <drake/math/rotation_conversion_gradient.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>

#include <algorithm>
#include <filesystem>
#include <limits>

#include "utils/utils.h"

namespace fs = std::filesystem;

using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace franka_driver {

ConstraintSolver::ConstraintSolver(const RobotParameters* params)
    : params_ {params}, urdf_path_ {params->urdf_filepath} {
  log()->debug("CS:ConstraintSolver: ctor: BEGIN");
  if (urdf_path_.empty()) {
    std::string error_msg =
        "CS::ConstraintSolver constructor: empty parameters->urdf_filepath: "
        + urdf_path_;  // add it anyway
    log()->error(error_msg);
    throw std::runtime_error(error_msg);
  }
  if (!fs::exists(urdf_path_)) {
    std::string error_msg =
        "CS::ConstraintSolver: URDF NOT FOUND: " + urdf_path_;
    log()->error(error_msg);
    throw std::runtime_error(error_msg);
  }

  static const double time_step {0.1};
  auto [mb_plant, scene_graph] {AddMultibodyPlantSceneGraph(
      &builder_, std::make_unique<MultibodyPlant<double>>(time_step))};
  mb_plant_ = &mb_plant;
  scene_graph_ = &scene_graph;
  try {
    robot_model_idx_ = drake::multibody::Parser(mb_plant_, scene_graph_)
                           .AddModelFromFile(urdf_path_, "robot_arm");
  } catch (std::exception const& ex) {
    dexai::log()->error("AddModelFromFile failed: {}", ex.what());
    throw;
  }

  {
    // get child frame of urdf that is used as root
    auto robot_root_link {params->world_frame};
    try {
      dexai::log()->debug(
          "CS::ConstraintSolver: "
          "Creating robot model parser with URDF {} \n"
          "using root link name in URDF: {}",
          urdf_path_, robot_root_link);
      dexai::log()->debug("CS:ctor: welding robot frame {} to world frame: {}",
                          robot_root_link, mb_plant.world_frame().name());
      auto& child_frame {
          mb_plant.GetFrameByName(robot_root_link, robot_model_idx_)};
      mb_plant.WeldFrames(mb_plant.world_frame(), child_frame);
    } catch (std::exception const& err) {
      dexai::log()->error(
          "CS:Error: Exception in dual robot collision ctor: \n"
          "Problem parsing this robot's ROOT LINK named: {}\n"
          "Exception message from Drake Multibody tree is: {}",
          robot_root_link, err.what());
      throw;  // rethrow exception again
    }
  }

  mb_plant.Finalize();
  robot_dof_ = mb_plant.num_positions(robot_model_idx_);
  dexai::log()->info("CS::ConstraintSolver: robot_dof_: {}", robot_dof_);

  // TODO(@anyone): Check if Collision Filter Groups work...
  // remove collisions within each set - do not check for collisions against
  // robot itself
  {
    using drake::multibody::Body;
    std::vector<const Body<double>*> robot_bodies;
    for (const auto& i : mb_plant.GetBodyIndices(robot_model_idx_)) {
      robot_bodies.push_back(&mb_plant_->get_body(i));
    }
    // cannot call {} because no {GeometrySet} constructor is available
    drake::geometry::GeometrySet set_robot(
        mb_plant.CollectRegisteredGeometries(robot_bodies));
    scene_graph.ExcludeCollisionsWithin(set_robot);
  }

  // allow for visualization
  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder_, scene_graph);
  // build a diagram
  diagram_ = builder_.Build();  // system ownership transferred
  // create a context
  context_ = diagram_->CreateDefaultContext();
  // auto scene_context {scene_graph.AllocateContext()};
  // auto output {diagram_->AllocateOutput()};

  // Following is needed if simulator is removed
  // plant_context_ = &diagram_->GetMutableSubsystemContext(
  //   *mb_plant_, context_);

  // TODO: simulator and derived plant_context_ from simulator is
  // only needed so that something gets published to the drake visualizer.
  // If we don't care about drake visualizing (which is not expensive),
  // then we replace the three commands below with the command above this
  // comment
  simulator_ = std::make_unique<drake::systems::Simulator<double>>(*diagram_);
  simulator_->Initialize();
  plant_context_ = &diagram_->GetMutableSubsystemContext(
      mb_plant, &simulator_->get_mutable_context());
  // diagram_->SetDefaultContext(context_.get());

  this->UpdateModel(Eigen::VectorXd::Zero(robot_dof_));
  /**
   * All robot_dof_ should be actuatable but
   * mb_plant.num_actuated_dofs() relies on the URDF being complete,
   * which is often not the case. Instead of
   *   num_actuatable_joints_ = mb_plant.num_actuated_dofs(robot_model_idx_);
   * we check that typename == revolute to pick out the N_revolute
   * and verify N_revolute = N_dof
   */
  {
    size_t i {};
    for (const auto& joint_idx : mb_plant.GetJointIndices(robot_model_idx_)) {
      const auto& joint {mb_plant.get_joint(joint_idx)};
      dexai::log()->debug(
          "joint #{}, name: {}, typename: {}, n_pos: {}, lim size: {}", ++i,
          joint.name(), joint.type_name(), joint.num_positions(),
          joint.position_lower_limits().size());
      if (joint.type_name() == "weld") {
        continue;
      } else if (joint.type_name() == "revolute") {
        joint_names_.push_back(joint.name());
      } else {
        std::string err_msg {"Unrecognised joint type: {}" + joint.type_name()};
        dexai::log()->error(err_msg);
        throw std::runtime_error(err_msg);
      }
    }
  }

  if (joint_names_.size() != robot_dof_) {
    std::string err_msg {"Number of joint mismatch. Double check URDF."};
    dexai::log()->error(err_msg);
    throw std::runtime_error(err_msg);
  }
  dexai::log()->info(
      "CS:ConstraintSolver: Number of dof (actuatable joints): {}", robot_dof_);
  // fill joint limits matrix
  joint_limits_.resize(robot_dof_, 2);
  for (size_t i {}; i < joint_names_.size(); i++) {
    const auto& joint {mb_plant.GetJointByName(joint_names_.at(i))};
    joint_limits_(i, 0) = joint.position_lower_limits()[0];
    joint_limits_(i, 1) = joint.position_upper_limits()[0];
  }

  q_nominal_.resize(robot_dof_, 1);
  q_guess_.resize(robot_dof_, 1);
  q_nominal_.setZero();
  q_guess_.setZero();
  log()->trace("CS:ConstraintSolver: END");
}

ConstraintSolver::~ConstraintSolver() {
  log()->trace("~ConstraintSolver: BEG...");
  unlink("snopt.out");
  log()->trace("~ConstraintSolver: ...END");
}

void ConstraintSolver::UpdateModel(
    const Eigen::Ref<const Eigen::VectorXd> pos_robot) {
  try {
    mb_plant_->SetPositions(plant_context_, robot_model_idx_, pos_robot);
  } catch (std::exception const& err) {
    dexai::log()->error(
        "ConstraintSolver::UpdateModel: "
        "failed with error: {}. Robot reports conf: {}",
        err.what(), pos_robot.transpose());
    throw;  // rethrow it again
  }

  // TODO: set this based on visualization level
  // this performs the magic of making the diagram publish to the visualizer.
  // not needed for the actual collision check.
  simulator_->get_system().Publish(simulator_->get_context());
}

}  // namespace franka_driver
