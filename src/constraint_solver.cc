// @file  constraint_solver.cc
#include "constraint_solver.h"

#include <unistd.h>

#include <drake/geometry/geometry_visualization.h>
#include <drake/math/quaternion.h>
#include <drake/math/rotation_conversion_gradient.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>

#include <algorithm>
#include <filesystem>
#include <limits>

#include "utils.h"

using dexai::log;

using drake::geometry::GeometrySet;
using drake::geometry::QueryObject;
using drake::math::RigidTransform;
using drake::multibody::Body;
using drake::multibody::BodyIndex;
using fs = filesystem;

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

  // create a scene graph that contains all the geometry of the system.
  scene_graph_ = builder_.AddSystem<SceneGraph>();
  scene_graph_->set_name("scene_graph");

  {
    // TODO: investigate difference of a discrete system
    // (0.1) vs a continuous system (0) in performance
    // Can't use continuous b/c: "Currently MultibodyPlant does not handle
    // joint limits for continuous models. However a limit was specified for
    // joint
    // ``franka_joint1`."
    double time_step {0.1};
    // created a plant that will later receive the two robots.
    robots_plant_ = builder_.AddSystem<MultibodyPlant>(time_step);
  }

  // attach plant as source for the scenegraph
  robots_plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  try {
    robot_model_idx_ = drake::multibody::Parser(robots_plant_, scene_graph_)
                .AddModelFromFile(urdf_path_, "robot_arm"
  };
}
catch (std::exception const& ex) {
  dexai::log()->error("AddModelFromFile failed: {}", ex.what());
  throw;
}

try {
  // get child frame of urdf that is used as root
  auto robot_root_link {params->world_frame};
  dexai::log()->debug(
      "CS::ConstraintSolver: "
      "Creating robot model parser with URDF {} \n"
      "using root link name in URDF: {}",
      urdf_path_, robot_root_link);
  dexai::log()->info(
      "CS:ctor: Trying to weld robot frame: {} ... \n"
      "... to world frame: {}",
      robot_root_link, robots_plant_->world_frame().name());
  auto& child_frame {
      robots_plant_->GetFrameByName(robot_root_link, robot_model_idx_)};
  robots_plant_->WeldFrames(robots_plant_->world_frame(), child_frame);
} catch (std::exception const& err) {
  dexai::log()->error(
      "CS:Error: Exception in dual robot collision ctor: \n"
      "Problem parsing this robot's ROOT LINK named: {}\n"
      "Exception message from Drake Multibody tree is: {}",
      robot_root_link, err.what());
  // no clean up is needed before throwing, no heap memory was allocated,
  // DiagramBuilder (builder_) cleans up after itself.
  throw;  // rethrow exception again
}
// ... collision checking setup is done in regards to this robot

// make a copy of model before finalizing, copy can then be used to
// make changes to the model later
robots_plant_non_final_ = robots_plant_;
robots_plant_->Finalize();
robot_dof_ = robots_plant_->num_positions(robot_model_idx_);
dexai::log()->info("CS::ConstraintSolver: robot_dof_: {}", robot_dof_);

// ToDo: Check if Collision Filter Groups work...
// remove collisions within each set - do not check for collisions against
// robot itself
scene_graph_->ExcludeCollisionsWithin(set_robot);

// Connect robot poses as input to scene graph
builder_.Connect(
    robots_plant_->get_geometry_poses_output_port(),
    scene_graph_->get_source_pose_port(robots_plant_->get_source_id().value()));
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

auto scene_context {scene_graph_->AllocateContext()};
auto output {diagram_->AllocateOutput()};

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

this->UpdateModel(Eigen::VectorXd::Zero(robot_dof_));

num_actuatable_joints_ =
    robots_plant_->num_actuated_dofs(robot_model_idx_) log()->trace(
        "CS:ConstraintSolver: Number of actuatable joints: {}",
        num_actuatable_joints_);

q_nominal_.resize(num_actuatable_joints_, 1);
q_guess_.resize(num_actuatable_joints_, 1);
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
