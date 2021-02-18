/// @file constraint_solver.h
#pragma once

#include "robot_parameters.h"

// MultibodyPlant:
#include <drake/multibody/parsing/parser.h>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::SceneGraph;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;

namespace franka_driver {

class ConstraintSolver {
 private:
  const RobotParameters* p_ = nullptr;
  Eigen::VectorXd joint_indices_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_nominal_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_guess_;
  uint num_actuatable_joints_;
  const std::string urdf_path_;
  std::unique_ptr<RigidBodyTree<double>> rigid_body_tree_ = nullptr;

  // added for MultibodyPlant functionality
  int robot_dof_;
  drake::systems::DiagramBuilder<double> builder_;
  SceneGraph<double>* scene_graph_;                 // is owned by builder_
  MultibodyPlant<double>* robots_plant_;            // is owned by builder_
  MultibodyPlant<double>* robots_plant_non_final_;  // is owned by builder_
  std::unique_ptr<Parser> parser_;
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
  ConstraintSolver(const RobotParameters* parameters);
  ~ConstraintSolver();

  inline const RobotParameters* GetParameters() const { return p_; }
  inline RigidBodyTree<double>* GetRigidBodyTreePtr() const {
    return rigid_body_tree_.get();
  }
  inline RigidBodyTree<double>& GetRigidBodyTreeRef() const {
    return *rigid_body_tree_.get();
  }
  inline unsigned GetNumActuatableJoints() const {
    return num_actuatable_joints_;
  }
  const std::string& GetUrdfPath() const { return urdf_path_; }
  std::vector<std::string> GetJointNames() const;
  Eigen::MatrixXd GetJointLimits() const;
  inline Eigen::MatrixXd GetJointLimitsMatrixXd() const {
    return GetJointLimits();
  }
  std::pair<Eigen::VectorXd, Eigen::VectorXd> GetJointLimitsVectorXdPair()
      const;
  int CheckJointLimits(const Eigen::VectorXd& posture) const;
};

}  // namespace franka_driver
