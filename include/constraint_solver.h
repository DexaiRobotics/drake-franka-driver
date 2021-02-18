/// @file constraint_solver.h
#pragma once

#include "robot_parameters.h"

// MultibodyPlant:
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>

using drake::geometry::SceneGraph;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
namespace franka_driver {

class ConstraintSolver {
 private:
  const RobotParameters* params_ = nullptr;
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_nominal_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_guess_;
  size_t num_actuatable_joints_;
  const std::string urdf_path_;
  Eigen::MatrixXd joint_limits_;

  // added for MultibodyPlant functionality
  size_t robot_dof_;
  drake::systems::DiagramBuilder<double> builder_;
  std::unique_ptr<SceneGraph<double>> scene_graph_;
  std::unique_ptr<MultibodyPlant<double>> robots_plant_;
  ModelInstanceIndex robot_model_idx_;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  drake::systems::Context<double>* plant_context_;
  std::unique_ptr<drake::systems::Simulator<double>> simulator_;

 protected:
  void UpdateModel(const Eigen::Ref<const Eigen::VectorXd> pos_robot);

  ///////////////////// Ctor, Dtor, Accessors
  //////////////////////////////////////
 public:
  explicit ConstraintSolver(const RobotParameters* parameters);
  ~ConstraintSolver();

  inline const RobotParameters* GetParameters() const { return params_; }
  inline size_t GetNumActuatableJoints() const {
    return num_actuatable_joints_;
  }
  inline const std::string& GetUrdfPath() const { return urdf_path_; }
  inline const Eigen::MatrixXd& GetJointLimits() const { return joint_limits_; }
};

}  // namespace franka_driver
