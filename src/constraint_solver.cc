// @file  constraint_solver.cc
#include "constraint_solver.h"

#include <unistd.h>

#include <algorithm>
#include <limits>

#include "drake/geometry/geometry_visualization.h"
#include "drake/math/quaternion.h"
#include "drake/math/rotation_conversion_gradient.h"
#include "utils.h"

using dexai::log;
using drake::manipulation::planner::DifferentialInverseKinematicsParameters;

using drake::geometry::GeometrySet;
using drake::geometry::QueryObject;
using drake::math::RigidTransform;
using drake::multibody::Body;
using drake::multibody::BodyIndex;
using drake::multibody::Parser;
using namespace Eigen;

namespace franka_driver {

ConstraintSolver::ConstraintSolver(const RobotParameters* params)
    : p_(params), urdf_path_(params->urdf_filepath) {
  log()->debug("CS:ConstraintSolver: ctor: BEGIN");
  if (urdf_path_.empty()) {
    std::string error_msg =
        "CS::ConstraintSolver constructor: empty parameters->urdf_filepath: "
        + urdf_path_;  // add it anyway
    log()->error(error_msg);
    throw std::runtime_error(error_msg);
  }
  if (!utils::file_exists(urdf_path_)) {
    std::string error_msg =
        "CS::ConstraintSolver: URDF NOT FOUND: " + urdf_path_;
    log()->error(error_msg);
    throw std::runtime_error(error_msg);
  }

  auto robot_urdf_filepath = urdf_path_;
  auto robot_root_link_in_urdf = params->world_frame;
  // create a scene graph that contains all the geometry of the system.
  scene_graph_ = builder_.AddSystem<SceneGraph>();
  scene_graph_->set_name("scene_graph");

  // TODO: investigate difference of a discrete system
  // (0.1) vs a continuous system (0) in performance
  // Can't use continuous b/c: "Currently MultibodyPlant does not handle
  // joint limits for continuous models. However a limit was specified for joint
  // ``franka_joint1`."
  double time_step = 0.1;
  // created a plant that will later receive the two robots.
  robots_plant_ = builder_.AddSystem<MultibodyPlant>(time_step);
  // attach plant as source for the scenegraph
  robots_plant_->RegisterAsSourceForSceneGraph(scene_graph_);

  dexai::log()->debug(
      "CS::ConstraintSolver: "
      "Creating robot model parser with URDF {} \n"
      "using root link name in URDF: {}",
      robot_urdf_filepath, robot_root_link_in_urdf);

  parser_ = std::make_unique<Parser>(robots_plant_, scene_graph_);

  dexai::log()->info("Name of world frame before adding model: {}",
                     robots_plant_->world_frame().name());

  try {
    // Parse the robot's urdf
    robot_model_idx_ =
        parser_->AddModelFromFile(robot_urdf_filepath, "robot_arm");
  } catch (std::exception const& err) {
    dexai::log()->error(
        "CS:ConstraintSolver: Error: Exception in CTOR: \n"
        "Problem parsing this robot's urdf file: {}\n"
        "Exception message from Drake Multibody tree is: {}",
        robot_urdf_filepath, err.what());
    // no clean up is needed before throwing, no heap memory was allocated,
    // DiagramBuilder (builder_) cleans up after itself.
    throw;  // rethrow it again
  }

  // obtain vector of Body Indices for each model for the Geometry Sets
  std::vector<BodyIndex> robot_body_indices =
      robots_plant_->GetBodyIndices(robot_model_idx_);

  std::vector<const Body<double>*> robot_bodies;
  for (size_t k = 0; k < robot_body_indices.size(); k++) {
    robot_bodies.push_back(&robots_plant_->get_body(robot_body_indices[k]));
  }

  dexai::log()->info(
      "CS:ctor: Trying to weld robot frame: {} ... \n"
      "... to world frame: {}",
      robot_root_link_in_urdf, robots_plant_->world_frame().name());

  // // use translation and orientation parameters to place in world frame
  // RigidTransform<double> X_WorldToRobot(
  // drake::math::RollPitchYaw<double>(urdf_offset_rpy)
  //                                      , urdf_offset_xyz);

  try {
    // get child frame of urdf that is used as root
    auto& child_frame = robots_plant_->GetFrameByName(robot_root_link_in_urdf,
                                                      robot_model_idx_);
    robots_plant_->WeldFrames(robots_plant_->world_frame(), child_frame);
    //   robots_plant_->WeldFrames(robots_plant_->world_frame(), child_frame,
    //   X_WorldToRobot2);
  } catch (std::exception const& err) {
    dexai::log()->error(
        "CS:Error: Exception in dual robot collision ctor: \n"
        "Problem parsing this robot's ROOT LINK named: {}\n"
        "Exception message from Drake Multibody tree is: {}",
        robot_root_link_in_urdf, err.what());
    // no clean up is needed before throwing, no heap memory was allocated,
    // DiagramBuilder (builder_) cleans up after itself.
    throw;  // rethrow exception again
  }
  // ... collision checking setup is done in regards to this robot

  // make a copy of model before finalizing, copy can then be used to
  // make changes to the model later
  robots_plant_non_final_ = robots_plant_;

  try {
    // Now the model is complete.
    robots_plant_->Finalize();
  } catch (std::exception const& err) {
    dexai::log()->error(
        "CS:Error: Exception in dual robot collision ctor: \n"
        "Problem finalizing this robot: {}\n"
        "Exception message from Drake Multibody tree is: {}",
        err.what());
    // no clean up is needed before throwing, no heap memory was allocated,
    // DiagramBuilder (builder_) cleans up after itself.
    throw;  // rethrow exception again
  }

  robot_dof_ = robots_plant_->num_positions(robot_model_idx_);
  dexai::log()->info("CS::ConstraintSolver: robot_dof_: {}", robot_dof_);

  // Register Geometry Sets for robot plants for the purpose of collision
  // checking
  GeometrySet set_robot = robots_plant_->CollectRegisteredGeometries(
      const_cast<const std::vector<const Body<double>*>&>(robot_bodies));
  // ToDo: Check if Collision Filter Groups work...
  // remove collisions within each set - do not check for collisions against
  // robot itself
  scene_graph_->ExcludeCollisionsWithin(set_robot);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!robots_plant_->get_source_id());

  // Connect robot poses as input to scene graph
  builder_.Connect(robots_plant_->get_geometry_poses_output_port(),
                   scene_graph_->get_source_pose_port(
                       robots_plant_->get_source_id().value()));

  // Connect scene graph's geometry as a query object to robot plant
  builder_.Connect(scene_graph_->get_query_output_port(),
                   robots_plant_->get_geometry_query_input_port());

  // allow for visualization
  drake::geometry::ConnectDrakeVisualizer(&builder_, *scene_graph_);

  // build a diagram
  diagram_ = builder_.Build();
  // create a context
  context_ = diagram_->CreateDefaultContext();
  diagram_->SetDefaultContext(context_.get());

  auto scene_context = scene_graph_->AllocateContext();
  auto output = diagram_->AllocateOutput();

  // Following is needed if simulator is removed
  // plant_context_ = &diagram_->GetMutableSubsystemContext(
  //   *robots_plant_, context_.get());

  // TODO: simulator and derived plant_context_ from simulator is
  // only needed so that something gets published to the drake visualizer.
  // If we don't care about drake visualizing (which is not expensive),
  // then we replace the three commands below with the command above this
  // comment
  simulator_ = std::make_unique<drake::systems::Simulator<double>>(*diagram_);
  simulator_->Initialize();
  plant_context_ = &diagram_->GetMutableSubsystemContext(
      *robots_plant_, &simulator_->get_mutable_context());

  Eigen::VectorXd pos_robot = Eigen::VectorXd::Zero(robot_dof_);
  this->UpdateModel(pos_robot);

  // TODO: remove this instance
  rigid_body_tree_ = std::make_unique<RigidBodyTree<double>>();
  // TODO: make this into a list of SDF files or a single SDF with <include>
  // robot + end-effector created in software
  // environment as list of objects with free-floating joints? ice-cream
  // container, people

  log()->trace(
      "CS:ConstraintSolver: "
      "drake::parsers::urdf::AddModelInstanceFromUrdfFile:");
  log()->trace(
      "CS:ConstraintSolver: AddModelInstanceFromUrdfFile({}, kFixed, NIL, "
      "rbt.get())",
      urdf_path_);
  try {
    // NOTE: The number of bodies, number of joints/DOF, and so on, are set in
    // RBT here.
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        urdf_path_, drake::multibody::joints::kFixed, nullptr,
        GetRigidBodyTreePtr());
  } catch (const std::exception& ex) {
    log()->error("Error in CS::ConstraintSolver ctor: {}", ex.what());
    throw;
  }

  log()->trace("CS:ConstraintSolver: Number of positions: {}",
               GetRigidBodyTreeRef().get_num_positions());
  log()->trace("CS:ConstraintSolver: Number of bodies: {}",
               GetRigidBodyTreeRef().get_num_bodies());

  num_actuatable_joints_ = GetRigidBodyTreeRef().get_num_positions();
  log()->trace("CS:ConstraintSolver: Number of actuatable joints: {}",
               num_actuatable_joints_);

  joint_indices_.resize(num_actuatable_joints_, 1);
  joint_indices_.setZero();
  for (uint i = 0; i < num_actuatable_joints_; i++) {
    joint_indices_(i) = i;
  }

  q_nominal_.resize(num_actuatable_joints_, 1);
  q_guess_.resize(num_actuatable_joints_, 1);
  q_nominal_.setZero();
  q_guess_.setZero();  // NOTE @sprax: Wondering if this is good enough?

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
    robots_plant_->SetPositions(plant_context_, robot_model_idx_, pos_robot);
  } catch (std::exception const& err) {
    dexai::log()->error(
        "ConstraintSolver::UpdateModel: "
        "failed with error {}. Robot reports conf: {}",
        err.what(), pos_robot.transpose());
    throw;  // rethrow it again
  }

  // TODO: set this based on visualization level
  // this performs the magic of making the diagram publish to the visualizer.
  // not needed for the actual collision check.
  simulator_->get_system().Publish(simulator_->get_context());
}

std::vector<std::string> ConstraintSolver::GetJointNames() const {
  std::vector<std::string> joint_names;
  for (size_t i = 0; i < num_actuatable_joints_; i++) {
    auto name = GetRigidBodyTreeRef().get_position_name(i);
    joint_names.push_back(name);
  }
  return joint_names;
}

Eigen::MatrixXd ConstraintSolver::GetJointLimits() const {
  auto joint_names = GetJointNames();
  Eigen::MatrixXd joint_limits;
  joint_limits.resize(num_actuatable_joints_, 2);
  for (size_t j = 0; j < joint_names.size(); j++) {
    joint_limits(j, 0) = GetRigidBodyTreeRef()
                             .FindChildBodyOfJoint(joint_names.at(j))
                             ->getJoint()
                             .getJointLimitMin()[0];
    joint_limits(j, 1) = GetRigidBodyTreeRef()
                             .FindChildBodyOfJoint(joint_names.at(j))
                             ->getJoint()
                             .getJointLimitMax()[0];
  }
  return joint_limits;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
ConstraintSolver::GetJointLimitsVectorXdPair() const {
  auto joint_names = GetJointNames();
  Eigen::VectorXd lower_limits(num_actuatable_joints_);
  Eigen::VectorXd upper_limits(num_actuatable_joints_);
  for (size_t j = 0; j < joint_names.size(); j++) {
    double lower_limit = GetRigidBodyTreeRef()
                             .FindChildBodyOfJoint(joint_names.at(j))
                             ->getJoint()
                             .getJointLimitMin()[0];
    lower_limits(j) = (double)round(lower_limit * 1e4) / 1.0e4;
    double upper_limit = GetRigidBodyTreeRef()
                             .FindChildBodyOfJoint(joint_names.at(j))
                             ->getJoint()
                             .getJointLimitMax()[0];
    upper_limits(j) = (double)round(upper_limit * 1e4) / 1.0e4;
  }
  return std::make_pair(lower_limits, upper_limits);
}

int ConstraintSolver::CheckJointLimits(const Eigen::VectorXd& posture) const {
  Eigen::MatrixXd joint_limits = GetJointLimits();
  Eigen::VectorXd joint_limits_lower = joint_limits.col(0);
  Eigen::VectorXd joint_limits_upper = joint_limits.col(1);

  auto lower_limit = posture.array() <= joint_limits_lower.array();
  auto upper_limit = posture.array() >= joint_limits_upper.array();

  if (lower_limit.any()) {
    return -1;
  } else if (upper_limit.any()) {
    return 1;
  }
  return 0;
}

}  // namespace franka_driver
