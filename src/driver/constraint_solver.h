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

/// @file constraint_solver.h
#pragma once

#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>

#include "utils/robot_parameters.h"

namespace franka_driver {

class ConstraintSolver {
 private:
  const RobotParameters* params_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_nominal_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_guess_;
  const std::string urdf_path_;
  std::vector<std::string> joint_names_;
  Eigen::MatrixXd joint_limits_;
  size_t robot_dof_ {};

  drake::systems::DiagramBuilder<double> builder_;
  /**
   * DiagramBuilder takes ownership of system pointers when they are added,
   * and then transfers the ownership of the system pointers to the Diagram
   * when build() is called.
   * Raw pointers below are owned by Diagram, their lifetime guaranteed by it.
   * AddMultibodyPlantSceneGraphResult has semantics for giving back
   * both raw pointers, MultibodyPlant and SceneGraph.
   * We keep Diagram which is the owner of everything. Also keep builder
   * for access to raw pointers if needed.
   */
  drake::multibody::MultibodyPlant<double>* mb_plant_ {};
  drake::geometry::SceneGraph<double>* scene_graph_ {};
  drake::multibody::ModelInstanceIndex robot_model_idx_ {};
  std::unique_ptr<drake::systems::Diagram<double>> diagram_ {};
  std::unique_ptr<drake::systems::Context<double>> context_ {};
  drake::systems::Context<double>* plant_context_ {};
  std::unique_ptr<drake::systems::Simulator<double>> simulator_ {};

 protected:
  void UpdateModel(const Eigen::Ref<const Eigen::VectorXd> pos_robot);

  ///////////////////// Ctor, Dtor, Accessors
  //////////////////////////////////////
 public:
  explicit ConstraintSolver(const RobotParameters* parameters);
  ~ConstraintSolver();

  inline const RobotParameters* GetParameters() const { return params_; }
  inline const std::string& GetUrdfPath() const { return urdf_path_; }
  inline const Eigen::MatrixXd& GetJointLimits() const { return joint_limits_; }
};

}  // namespace franka_driver
