// @file  constraint_solver.cc
#include "constraint_solver.h"

#include <unistd.h>

#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/geometry_visualization.h>
#include <drake/math/quaternion.h>
#include <drake/math/rotation_conversion_gradient.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>

#include <algorithm>
#include <filesystem>
#include <limits>

#include "utils.h"

// using drake::multibody::BodyIndex;
namespace fs = std::filesystem;

using drake::geometry::SceneGraph;
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

  drake::systems::DiagramBuilder<double> builder;
  // create a scene graph that contains all the geometry of the system.
  scene_graph_ = builder.AddSystem<SceneGraph>();
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
    mb_plant_ = builder.AddSystem<MultibodyPlant>(time_step);
  }

  // attach plant as source for the scenegraph
  auto source_id {mb_plant_->RegisterAsSourceForSceneGraph(scene_graph_)};
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
      dexai::log()->info(
          "CS:ctor: Trying to weld robot frame: {} ... \n"
          "... to world frame: {}",
          robot_root_link, mb_plant_->world_frame().name());
      auto& child_frame {
          mb_plant_->GetFrameByName(robot_root_link, robot_model_idx_)};
      mb_plant_->WeldFrames(mb_plant_->world_frame(), child_frame);
    } catch (std::exception const& err) {
      dexai::log()->error(
          "CS:Error: Exception in dual robot collision ctor: \n"
          "Problem parsing this robot's ROOT LINK named: {}\n"
          "Exception message from Drake Multibody tree is: {}",
          robot_root_link, err.what());
      // no clean up is needed before throwing, no heap memory was allocated,
      // DiagramBuilder (builder) cleans up after itself.
      throw;  // rethrow exception again
    }
  }

  // make a copy of model before finalizing, copy can then be used to
  // make changes to the model later
  mb_plant_->Finalize();
  robot_dof_ = mb_plant_->num_positions(robot_model_idx_);
  dexai::log()->info("CS::ConstraintSolver: robot_dof_: {}", robot_dof_);

  // ToDo: Check if Collision Filter Groups work...
  // remove collisions within each set - do not check for collisions against
  // robot itself
  {
    using drake::multibody::Body;
    std::vector<const Body<double>*> robot_bodies;
    for (const auto& i : mb_plant_->GetBodyIndices(robot_model_idx_)) {
      robot_bodies.push_back(&mb_plant_->get_body(i));
    }
    // cannot call {} because no {GeometrySet} constructor is available
    drake::geometry::GeometrySet set_robot(
        mb_plant_->CollectRegisteredGeometries(robot_bodies));
    scene_graph_->ExcludeCollisionsWithin(set_robot);
  }

  // Connect scene graph's geometry as a query object to robot plant
  builder.Connect(scene_graph_->get_query_output_port(),
                  mb_plant_->get_geometry_query_input_port());
  // Connect robot poses as input to scene graph
  auto registerd {mb_plant_->geometry_source_is_registered()};
  // cannot use mb_plant_->get_source_id() as it returns junk values
  builder.Connect(mb_plant_->get_geometry_poses_output_port(),
                  scene_graph_->get_source_pose_port(source_id));

  // allow for visualization
  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph_);

  // build a diagram
  diagram_ = builder.Build();
  // create a context
  context_ = diagram_->CreateDefaultContext();
  diagram_->SetDefaultContext(context_.get());

  auto scene_context {scene_graph_->AllocateContext()};
  auto output {diagram_->AllocateOutput()};

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
      *mb_plant_, &simulator_->get_mutable_context());

  this->UpdateModel(Eigen::VectorXd::Zero(robot_dof_));

  num_actuatable_joints_ = mb_plant_->num_actuated_dofs(robot_model_idx_);
  dexai::log()->trace("CS:ConstraintSolver: Number of actuatable joints: {}",
                      num_actuatable_joints_);

  q_nominal_.resize(num_actuatable_joints_, 1);
  q_guess_.resize(num_actuatable_joints_, 1);
  q_nominal_.setZero();
  q_guess_.setZero();

  // set joint names and joint_limits

  joint_limits_.resize(num_actuatable_joints_, 2);
  for (const auto& i : mb_plant_->GetJointIndices(robot_model_idx_)) {
    const auto& joint {mb_plant_->get_joint(i)};
    auto ll {joint.position_lower_limits()};
    joint_limits_(i, 0) = joint.position_lower_limits()[0];
    joint_limits_(i, 1) = joint.position_upper_limits()[0];
  }
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
        "failed with error {}. Robot reports conf: {}",
        err.what(), pos_robot.transpose());
    throw;  // rethrow it again
  }

  // TODO: set this based on visualization level
  // this performs the magic of making the diagram publish to the visualizer.
  // not needed for the actual collision check.
  simulator_->get_system().Publish(simulator_->get_context());
}

}  // namespace franka_driver
