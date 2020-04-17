// @file  robot_parameters.cc
#include "robot_parameters.h"
#include "util_string.h"

#include <pwd.h>
#include <stdexcept>

// DONE @sprax: Throw exception: YamlValueNotFound
// DONE @sprax: Throw exception: YamlKeyNotFound
// TODO @sprax: Move some namespaced functions to RobotParameters static methods and protect them.

namespace franka_driver {

    const std::string kBaseLinkWorldFrame = "base_link"; // World Geometry

    std::string RobotParameters::s_def_dump_str_ = "-";

    void LogMissingYamlKey(const std::string& key, int verbose)
    {
        if (verbose > 0) {
            std::string msg = fmt::format("RobotParameters::get_yaml_val* FAILED for key = '{}'", key);
            // Invert part of the scale, so if want to be warned, call with verbose > 3,
            // but if you only want to see this message in debug, call with verbose = 2, etc.
            if (verbose > 3) {
                log()->warn(msg);
            } else if (verbose > 2) {
                log()->info(msg);
            } else if (verbose > 1) {
                log()->debug(msg);
            } else {
                log()->trace(msg);
            }
        }
    }

    // Find the specified key in the given YAML Node (in its top-level iterator)
    // and return the corresponding value as a YAML Node,
    // else throw YamlKeyNotFound.
    // @see also FindNode below.
    const YAML::Node FindYamlKeyOrThrow( const YAML::Node& parent_node // source node
                                       , const std::string& find_key   // key of top-level node
                                       , const std::string& yaml_file  // file name for logging
                                       , int verbose
    ) {
        for (const auto& kv : parent_node) {
            std::string key = kv.first.as<std::string>();
            if (key == find_key) {
                return kv.second;   // the found value
            }
        }
        std::string error_msg = fmt::format("FindYamlKeyOrThrow: key({}) NOT FOUND (file: {}), so THROW"
                                           , find_key, yaml_file);
        if (verbose > 1) {
            log()->info(error_msg);
        }
        throw YamlKeyNotFound(error_msg);
    }


    // FindNode return true if the given YAML Node contains an entry for node_name,
    // otherwise it returns false.  It tries to catch most exceptions,
    // and also returns false if it does catch one.
    //
    // FindNode is an earlier implementation providing functionality
    // similar to FindYamlKeyOrThrow.  But instead of returning the
    // found node or throwing, it returns true or false.
    // NOTE @sprax: It could be renamed to ContainsNode.
    bool FindNode(YAML::Node node, std::string node_name, drake::logging::logger *logger)
    {
        try {
            // NOTE: A later version of YAML-cpp may provide a better check than the following.
            //       But Drake still uses an older version as of 2019.12.26.  (Re-check that?)
            if (node[node_name]) {  // Try simple access, which may throw.
                return true;
            }
            logger->warn("P:FindNode: name not found: <{}>", node_name);
            return false;
        } catch(const YAML::InvalidNode& invalid_node) {
            log()->error( "P:FindNode: InvalidNode exception in FindNode(tag={}, key={}): {}"
                        , node.Tag(), node_name, invalid_node.what());
        } catch (const std::exception& ex) {
            log()->error( "P:FindNode: Standard Exception in FindNode(tag={}, key={}): {}"
                        , node.Tag(), node_name, ex.what());
        }
        return false;
    }

    template <> bool get_yaml_val(const YAML::Node& yaml, const std::string& key, Eigen::VectorXd& val)
    {
        auto found = yaml[key];
        if (found) {
            std::vector<double> vec = found.as< std::vector<double> >();
            val = utils::v_to_e(vec);
            return true;
        }
        return false;
    }

    template <> bool get_yaml_val_verbose( const YAML::Node& yaml
                                         , const std::string& key
                                         , Eigen::VectorXd& val
                                         , const int verbose
    ) {
        bool found = get_yaml_val(yaml, key, val);
        if  (found) {
            if (verbose > 2) {
                log()->debug("Loaded [{}] == ({})", key, val.transpose());
            }
        } else if (verbose > 1) {
            log()->warn("Failed: yaml[{}] not found.", key);
        } else if (verbose > 0) {
            log()->info("Failed: yaml[{}] not found.", key);
        }
        return found;
    }

    template <> bool get_yaml_val(const YAML::Node& yaml, const std::string& key, Eigen::Vector3d& val)
    {
        auto found = yaml[key];
        if (found) {
            try {
                std::vector<double> vec = found.as< std::vector<double> >();
                val = utils::v_to_e(vec);
                return true;
            } catch (const std::exception& ex) {
                log()->error( "Error in get_yaml_val(tag={}, key={}, Eigen::Vector3d&): {}"
                            , yaml.Tag(), key, ex.what());
            }
        }
        return false;
    }

    template <> bool get_yaml_val_verbose( const YAML::Node& yaml
                                         , const std::string& key
                                         , Eigen::Vector3d& val
                                         , const int verbose
    ) {
        if (get_yaml_val(yaml, key, val)) {
            if (verbose > 2) {
                log()->debug("Loaded [{}] == ({})", key, val.transpose());
            }
            return true;
        }
        if (verbose > 2) {
            log()->warn("Failed: yaml[{}] not found.", key);
        } else if (verbose > 1) {
            log()->info("Failed: yaml[{}] not found.", key);
        }
        return false;
    }

    bool RobotParameters::GenerateParametersBasedOnRobotName(const std::string& new_robot_name)
    {
        if (new_robot_name.empty()) {
            log()->error("GenerateParametersBasedOnRobotName: empty robot name!");
            return false;
        }

        // update LCM Channels
        lcm_status_channel               = new_robot_name + "_STATUS";
        lcm_plan_channel                 = new_robot_name + "_PLAN";
        lcm_plan_received_channel        = new_robot_name + "_PLAN_RECEIVED";
        lcm_plan_complete_channel        = new_robot_name + "_PLAN_COMPLETE";
        lcm_stop_channel                 = new_robot_name + "_STOP";
        lcm_actuate_tool_channel         = new_robot_name + "_ACTUATE_TOOL";
        lcm_open_tool_channel            = new_robot_name + "_OPEN_TOOL";
        lcm_close_tool_channel           = new_robot_name + "_CLOSE_TOOL";
        lcm_lock_tool_channel            = new_robot_name + "_LOCK_TOOL";
        lcm_unlock_tool_channel          = new_robot_name + "_UNLOCK_TOOL";
        lcm_tool_server_response_channel = new_robot_name + "_TOOL_SERVER_RESPONSE";
        lcm_tool_open_channel            = new_robot_name + "_TOOL_OPEN";
        lcm_tool_locked_channel          = new_robot_name + "_TOOL_LOCKED";

        return true;
    }

    bool RobotParameters::UpdateRobotName(const std::string& new_robot_name)
    {
        log()->trace("RobotParameters::UpdateRobotName({})", new_robot_name);
        robot_name = new_robot_name;
        GenerateParametersBasedOnRobotName(robot_name);
        log()->debug("RobotParameters::UpdateRobotName: updated robot_name to: {}", new_robot_name);
        return true;
    }

    // TODO @sprax: Verify the new urdf_filepath and URDF file. 
    bool RobotParameters::UpdateUrdf(const std::string& new_urdf_file_name)
    {
        log()->trace("RobotParameters::UpdateUrdf({})", new_urdf_file_name);
        urdf = new_urdf_file_name;
        if (urdf_dir.empty()) {
            urdf_filepath = urdf; // KEEP
        } else {
            if (urdf_dir.back() != '/') {
                urdf_dir += "/";
            }
            urdf_filepath = urdf_dir + urdf; // KEEP
        }
        log()->debug("RobotParameters::UpdateUrdf: updated URDF to: {}", urdf_filepath); // KEEP
        return true;
    }

    bool RobotParameters::UpdateUrdfRobotOnly(const std::string& new_urdf_robot_only_file_name)
    {
        log()->trace("RobotParameters::UpdateUrdfRobotOnly({})", new_urdf_robot_only_file_name);
        urdf_robot_only = new_urdf_robot_only_file_name;
        if (urdf_dir.empty()) {
            urdf_robot_only_filepath = urdf_robot_only;
        } else {
            if (urdf_dir.back() != '/') {
                urdf_dir += "/";
            }
            urdf_robot_only_filepath = urdf_dir + urdf_robot_only;
        }
        log()->debug("RobotParameters::UpdateUrdfRobotOnly: updated URDF_ROBOT_ONLY to: {}"
            , urdf_robot_only_filepath
        );
        return true;
    }

    size_t RobotParameters::log_dump( drake::logging::logger *logger
                               , int verbose
                               , std::string& dumps
    ) const {
        YAML::Emitter yout;
        yout << YAML::BeginMap;
        yout << YAML::Key << "gravity_vector";
        yout << YAML::Value << utils::e_to_v(gravity_vector);
        yout << YAML::Key << "world_frame";
        yout << YAML::Value << world_frame;
        yout << YAML::Key << "safe_z";
        yout << YAML::Value << safe_z;

        yout << YAML::Comment("\nGeometric Configuration");
        yout << YAML::Key << "urdf";
        yout << YAML::Value << urdf;
        yout << YAML::Key << "urdf_robot_only";
        yout << YAML::Value << urdf_robot_only;
        yout << YAML::Key << "urdf_simple_geo";
        yout << YAML::Value << urdf_simple_geo;
        yout << YAML::Key << "urdf_utensil_in_bin";
        yout << YAML::Value << urdf_utensil_in_bin;
        yout << YAML::Key << "urdf_no_utensil";
        yout << YAML::Value << urdf_no_utensil;
        yout << YAML::Key << "urdf_utensil_attached";
        yout << YAML::Value << urdf_utensil_attached;
        yout << YAML::Key << "urdf_dir";
        yout << YAML::Value << urdf_dir;
        yout << YAML::Key << "robot_dof";
        yout << YAML::Value << robot_dof;
        yout << YAML::Key << "robot_name";
        yout << YAML::Value << robot_name;
        yout << YAML::Key << "simulated";
        yout << YAML::Value << simulated;
        yout << YAML::Key << "workstation_name";
        yout << YAML::Value << workstation_name;

        yout << YAML::Comment("\n Utensil");
        yout << YAML::Key << "utensil_type";
        yout << YAML::Value << utensil_type;
        yout << YAML::Key << "utensil_size";
        yout << YAML::Value << utensil_size;
        yout << YAML::Key << "tool_frame";
        yout << YAML::Value << tool_frame;
        yout << YAML::Key << "tool_axis";
        yout << YAML::Value << utils::e_to_v(tool_axis);
        yout << YAML::Key << "tool_face_normal";
        yout << YAML::Value << utils::e_to_v(tool_face_normal);
        yout << YAML::Key << "tool_radius";
        yout << YAML::Value << tool_radius;
        yout << YAML::Key << "action_overlap_factor";
        yout << YAML::Value << action_overlap_factor;
        yout << YAML::Key << "action_type";
        yout << YAML::Value << action_type;

        yout << YAML::Comment("\n Material");
        yout << YAML::Key << "material";
        yout << YAML::Value << material;
        yout << YAML::Key << "material_type";
        yout << YAML::Value << material_type;
        yout << YAML::Key << "container_frame";
        yout << YAML::Value << container_frame;
        yout << YAML::Key << "container_corner_transform";
        yout << YAML::Value << utils::e_to_v(container_corner_transform);
        yout << YAML::Key << "container_dimensions";
        yout << YAML::Value << utils::e_to_v(container_dimensions);
        yout << YAML::Key << "container_edge";
        yout << YAML::Value << container_edge;

        yout << YAML::Comment("\n Work Station Geometry");
        yout << YAML::Key << "tool_changer_frame";
        yout << YAML::Value << tool_changer_frame;
        yout << YAML::Key << "approach_frame";
        yout << YAML::Value << approach_frame;
        yout << YAML::Key << "exit_frame";
        yout << YAML::Value << exit_frame;
        yout << YAML::Key << "dispense_frame";
        yout << YAML::Value << dispense_frame;
        yout << YAML::Key << "view_conf";
        yout << YAML::Value << utils::e_to_v(view_conf);

        yout << YAML::Comment("\n Action Search");
        yout << YAML::Key << "action_index_spatial_map_discretization";
        yout << YAML::Value << action_index_spatial_map_discretization;
        yout << YAML::Key << "container_state_discretization";
        yout << YAML::Value << container_state_discretization;
        yout << YAML::Key << "sme_voxel_size";
        yout << YAML::Value << sme_voxel_size;

        yout << YAML::Comment("\n Stored IK Solution Files");
        yout << YAML::Key << "log_dir_name";
        yout << YAML::Value << log_dir_name;
        yout << YAML::Key << "traj_lib_dir_name";
        yout << YAML::Value << traj_lib_dir_name;


        yout << YAML::Comment("\n Robot Control RobotParameters");
        yout << YAML::Key << "prompt_robot_motion";
        yout << YAML::Value << prompt_robot_motion;
        yout << YAML::Key << "lcm_status_channel";
        yout << YAML::Value << lcm_status_channel;
        yout << YAML::Key << "lcm_plan_channel";
        yout << YAML::Value << lcm_plan_channel;
        yout << YAML::Key << "lcm_plan_received_channel";
        yout << YAML::Value << lcm_plan_received_channel;
        yout << YAML::Key << "lcm_plan_complete_channel";
        yout << YAML::Value << lcm_plan_complete_channel;
        yout << YAML::Key << "lcm_stop_channel";
        yout << YAML::Value << lcm_stop_channel;
        yout << YAML::Key << "lcm_actuate_tool_channel";
        yout << YAML::Value << lcm_actuate_tool_channel;
        yout << YAML::Key << "lcm_open_tool_channel";
        yout << YAML::Value << lcm_open_tool_channel;
        yout << YAML::Key << "lcm_close_tool_channel";
        yout << YAML::Value << lcm_close_tool_channel;
        yout << YAML::Key << "lcm_lock_tool_channel";
        yout << YAML::Value << lcm_lock_tool_channel;
        yout << YAML::Key << "lcm_unlock_tool_channel";
        yout << YAML::Value << lcm_unlock_tool_channel;
        yout << YAML::Key << "lcm_tool_server_response_channel";
        yout << YAML::Value << lcm_tool_server_response_channel;
        yout << YAML::Key << "lcm_tool_open_channel";
        yout << YAML::Value << lcm_tool_open_channel;
        yout << YAML::Key << "lcm_tool_locked_channel";
        yout << YAML::Value << lcm_tool_locked_channel;
        yout << YAML::Key << "lcm_url";
        yout << YAML::Value << lcm_url;
        yout << YAML::Key << "robot_ip";
        yout << YAML::Value << robot_ip;
        yout << YAML::Key << "ros_wrist_camera_ns";
        yout << YAML::Value << ros_wrist_camera_ns;
        yout << YAML::Key << "ros_head_camera_ns";
        yout << YAML::Value << ros_head_camera_ns;

        yout << YAML::Comment("\n Collision");
        yout << YAML::Key << "kCollisionEpsilonConservative";
        yout << YAML::Value << kCollisionEpsilonConservative;
        yout << YAML::Key << "kCollisionEpsilonMin";
        yout << YAML::Value << kCollisionEpsilonMin;
        yout << YAML::Key << "kCollisionEpsilonContainer";
        yout << YAML::Value << kCollisionEpsilonContainer;

        yout << YAML::Comment("\n Motion Control");
        yout << YAML::Key << "kMaxJointDistanceBetweenKnots";
        yout << YAML::Value << kMaxJointDistanceBetweenKnots;
        yout << YAML::Key << "kSplineSampleFreq";
        yout << YAML::Value << kSplineSampleFreq;
        yout << YAML::Key << "splineType";
        yout << YAML::Value << splineType;

        yout << YAML::Comment("\n IK RobotParameters");
        yout << YAML::Key << "kLoosePositionEpsilon";
        yout << YAML::Value << kLoosePositionEpsilon;
        yout << YAML::Key << "kTightPositionEpsilon";
        yout << YAML::Value << kTightPositionEpsilon;
        yout << YAML::Key << "kMediumPositionEpsilon";
        yout << YAML::Value << kMediumPositionEpsilon;
        yout << YAML::Key << "kLooseOrientationEpsilon";
        yout << YAML::Value << kLooseOrientationEpsilon;
        yout << YAML::Key << "kMediumOrientationEpsilon";
        yout << YAML::Value << kMediumOrientationEpsilon;
        yout << YAML::Key << "kTightOrientationEpsilon";
        yout << YAML::Value << kTightOrientationEpsilon;
        yout << YAML::Key << "kLooseJointDistance";
        yout << YAML::Value << kLooseJointDistance;
        yout << YAML::Key << "kMediumJointDistance";
        yout << YAML::Value << kMediumJointDistance;
        yout << YAML::Key << "kTightJointDistance";
        yout << YAML::Value << kTightJointDistance;

        yout << YAML::Comment("\n Compound Params (run-specific)");
        yout << YAML::Key << "urdf_filepath"; // KEEP
        yout << YAML::Value << urdf_filepath; // KEEP
        yout << YAML::Key << "log_base_dir";
        yout << YAML::Value << log_base_dir;
        yout << YAML::Key << "workstation_dir";
        yout << YAML::Value << workstation_dir;

        yout << YAML::Key << "utensil_dir";
        yout << YAML::Value << utensil_dir;
        yout << YAML::Key << "static_conf_dir";
        yout << YAML::Value << static_conf_dir;
        yout << YAML::Key << "static_traj_dir";
        yout << YAML::Value << static_traj_dir;
        yout << YAML::Key << "container_dir";
        yout << YAML::Value << container_dir;
        yout << YAML::EndMap;

        // Position and derivative constraints
        yout << YAML::Key << "robot_high_joint_limits";
        yout << YAML::Value << utils::e_to_v(robot_high_joint_limits);
        yout << YAML::Key << "robot_low_joint_limits";
        yout << YAML::Value << utils::e_to_v(robot_low_joint_limits);
        yout << YAML::Key << "robot_max_velocities";
        yout << YAML::Value << utils::e_to_v(robot_max_velocities);
        yout << YAML::Key << "robot_max_accelerations";
        yout << YAML::Value << utils::e_to_v(robot_max_accelerations);
        yout << YAML::Key << "robot_max_jerks";
        yout << YAML::Value << utils::e_to_v(robot_max_jerks);

        yout << YAML::Key << "safety_scale_factor";
        yout << YAML::Value << safety_scale_factor;

        yout << YAML::Key << "stop_epsilon";
        yout << YAML::Value << stop_epsilon;
        yout << YAML::Key << "stop_margin";
        yout << YAML::Value << stop_margin;
        yout << YAML::Key << "live_collision_distance_threshold";
        yout << YAML::Value << live_collision_distance_threshold;
        yout << YAML::Key << "plan_collision_distance_threshold";
        yout << YAML::Value << plan_collision_distance_threshold;
        yout << YAML::Key << "trajectory_divisions_to_check";
        yout << YAML::Value << trajectory_divisions_to_check;

        time_t log_time = time(NULL);
        size_t num_bytes = yout.size();
        if (dumps.empty()) {
            dumps = std::string(yout.c_str());
        }
        if (verbose > 1) {
            logger->info("RobotParameters::log -- dumping all YAML params ({} bytes)\n"
                        "# BEGIN dump {}\n{}\n# END dump {}"
                        , num_bytes, log_time, yout.c_str(), log_time);
        } else {
            logger->debug("RobotParameters::log -- dumping all YAML params ({} bytes)\n"
                         "# BEGIN dump {}\n{}\n# END dump {}"
                         , num_bytes, log_time, yout.c_str(), log_time);
        }
        return num_bytes;
    }

    // TODO @sprax #future: verbosity and exit_code can be changed once we
    // log all params and accept default values.
    RobotParameters loadYamlParameters( const std::string& yaml_full_path
                                 , int verbose
                                 , drake::logging::logger *logger
                                 , int exit_code    // calls exit on error if exit_code != 0
    ) {
        if (logger == nullptr) {
            logger = log();
            assert( logger && "logger was nullptr, but is STILL null!");
        }

        RobotParameters p;   // to be returned
        struct stat result;
        if (! utils::file_stat(yaml_full_path, result)) {
            std::string error_msg = "P:loadYamlParameters: Unable to find parameters YAML("
                                  + yaml_full_path + ")";
            log()->error(error_msg);
            throw std::runtime_error(error_msg);
            return p;               // would return empty/default, except for throw.
        }

        YAML::Node config = YAML::LoadFile(yaml_full_path);
        if ( ! config["parameters"]) {
            std::string error_msg = "P:loadYamlParameters: No parameters found in ("
                                  + yaml_full_path + ")";
            logger->error(error_msg);
            throw std::runtime_error(error_msg);
        }
        int config_length = static_cast<int>(config["parameters"].size());
        logger->debug("P:loadYamlParameters: parameters length: {}", config_length);

        YAML::Node param = config["parameters"];

        std::string home_dir = utils::get_home_dir() + "/";

        // ******************** RobotParameters ********************

        //  Try to get RobotParameters that have no default values.  Don't ignore failures:
        try {

            // World Configuration
            get_yaml_val_or_die(param, "gravity_vector", p.gravity_vector, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "world_frame", p.world_frame, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "safe_z", p.safe_z, yaml_full_path, verbose, exit_code);

            //// Geometric Configuration
            get_yaml_val_or_die(param, "urdf", p.urdf, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "urdf_robot_only", p.urdf_robot_only, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "urdf_simple_geo", p.urdf_simple_geo, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "urdf_utensil_in_bin", p.urdf_utensil_in_bin, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "urdf_no_utensil", p.urdf_no_utensil, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "urdf_utensil_attached", p.urdf_utensil_attached, yaml_full_path, verbose, exit_code);

            // Ensure that p.urdf_dir is either "" or else begins with $HOME ends in "/":
            std::string udr;
            get_yaml_val_or_die(param, "urdf_dir", udr, yaml_full_path, verbose, exit_code);
            p.urdf_dir = (udr == "") ? "" : home_dir + ((udr[udr.size()-1] == '/') ? udr : udr + "/");

            // Robot properties: Degrees Of Freedom (num joints), and so on.
            get_yaml_val_or_die(param, "simulated", p.simulated, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "robot_dof", p.robot_dof, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "workstation_name", p.workstation_name, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "traj_lib_dir_name", p.traj_lib_dir_name, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "log_dir_name", p.log_dir_name, yaml_full_path, verbose, exit_code);

            try {
                get_yaml_val_or_throw(param, "robot_name", p.robot_name, yaml_full_path, verbose);
            } catch (const YamlValueNotFound &ignore) {
                // use default based on hostname
                std::string hostname = utils::hostname_string();
                std::string hostname_uppercase = utils::to_uppercase(hostname);
                std::string first_letter(1, hostname_uppercase[0]);
                p.robot_name = "FRANKA_" + first_letter;
                log()->warn("P:loadYamlParameters: Robot name not found! Using default based on hostname: {}"
                           , p.robot_name);
            }

            // utensil
            get_yaml_val_or_die(param, "utensil_type", p.utensil_type, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "utensil_size", p.utensil_size, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "tool_frame", p.tool_frame, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "tool_axis", p.tool_axis, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "tool_face_normal", p.tool_face_normal, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "tool_radius", p.tool_radius, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "action_overlap_factor", p.action_overlap_factor, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "action_type", p.action_type, yaml_full_path, verbose, exit_code);

            // material
            get_yaml_val_or_die(param, "material", p.material, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "container_frame", p.container_frame, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "container_corner_transform", p.container_corner_transform, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "container_dimensions", p.container_dimensions, yaml_full_path, verbose, exit_code);

            // work station geometry
            get_yaml_val_or_die(param, "tool_changer_frame", p.tool_changer_frame, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "approach_frame", p.approach_frame, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "exit_frame", p.exit_frame, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "dispense_frame", p.dispense_frame, yaml_full_path, verbose, exit_code);

            p.view_conf = Eigen::VectorXd::Zero(p.robot_dof);

            // Robot Control RobotParameters
            get_yaml_val_or_die(param, "prompt_robot_motion", p.prompt_robot_motion, yaml_full_path, verbose, exit_code);

            p.lcm_status_channel               = p.robot_name + "_STATUS";
            p.lcm_plan_channel                 = p.robot_name + "_PLAN";
            p.lcm_plan_received_channel        = p.robot_name + "_PLAN_RECEIVED";
            p.lcm_plan_complete_channel        = p.robot_name + "_PLAN_COMPLETE";
            p.lcm_stop_channel                 = p.robot_name + "_STOP";
            p.lcm_actuate_tool_channel         = p.robot_name + "_ACTUATE_TOOL";
            p.lcm_open_tool_channel            = p.robot_name + "_OPEN_TOOL";
            p.lcm_close_tool_channel           = p.robot_name + "_CLOSE_TOOL";
            p.lcm_lock_tool_channel            = p.robot_name + "_LOCK_TOOL";
            p.lcm_unlock_tool_channel          = p.robot_name + "_UNLOCK_TOOL";
            p.lcm_tool_server_response_channel = p.robot_name + "_TOOL_SERVER_RESPONSE";
            p.lcm_tool_open_channel            = p.robot_name + "_TOOL_OPEN";
            p.lcm_tool_locked_channel          = p.robot_name + "_TOOL_LOCKED";

            get_yaml_val_or_die(param, "lcm_url", p.lcm_url, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "robot_ip", p.robot_ip, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "ros_head_camera_ns", p.ros_head_camera_ns, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "ros_wrist_camera_ns", p.ros_wrist_camera_ns, yaml_full_path, verbose, exit_code);

            // Collision
            get_yaml_val_or_die(param, "kCollisionEpsilonConservative", p.kCollisionEpsilonConservative, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "kCollisionEpsilonMin", p.kCollisionEpsilonMin, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "kCollisionEpsilonContainer", p.kCollisionEpsilonContainer, yaml_full_path, verbose, exit_code);

            // Motion Control
            get_yaml_val_or_die( param, "kMaxJointDistanceBetweenKnots"
                               , p.kMaxJointDistanceBetweenKnots, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "kSplineSampleFreq", p.kSplineSampleFreq, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "splineType", p.splineType, yaml_full_path, verbose, exit_code);

            // IK RobotParameters
            get_yaml_val_or_die(param, "kLoosePositionEpsilon", p.kLoosePositionEpsilon, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "kTightPositionEpsilon", p.kTightPositionEpsilon, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "kMediumPositionEpsilon", p.kMediumPositionEpsilon, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "kLooseOrientationEpsilon", p.kLooseOrientationEpsilon, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "kMediumOrientationEpsilon", p.kMediumOrientationEpsilon, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "kTightOrientationEpsilon", p.kTightOrientationEpsilon, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "kLooseJointDistance", p.kLooseJointDistance, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "kMediumJointDistance", p.kMediumJointDistance, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "kTightJointDistance", p.kTightJointDistance, yaml_full_path, verbose, exit_code);

            // Derivative constraints
            get_yaml_val_or_die(param, "robot_high_joint_limits", p.robot_high_joint_limits, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "robot_low_joint_limits", p.robot_low_joint_limits, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "robot_max_velocities", p.robot_max_velocities, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "robot_max_accelerations", p.robot_max_accelerations, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "robot_max_jerks", p.robot_max_jerks, yaml_full_path, verbose, exit_code);

            get_yaml_val_or_die(param, "stop_epsilon", p.stop_epsilon, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "stop_margin", p.stop_margin, yaml_full_path, verbose, exit_code);

            if (param["adjacent_robots"]) {
                YAML::Node adjacent_robots = param["adjacent_robots"];
                for (std::size_t i = 0; i < adjacent_robots.size(); i++) {
                    std::string robot_name;
                    Eigen::Vector3d base_xyz, base_rpy;
                    const YAML::Node& adjacent_robot = adjacent_robots[i];
                    get_yaml_val_or_die(adjacent_robot, "robot_name", robot_name, yaml_full_path, verbose, exit_code);
                    get_yaml_val_or_die(adjacent_robot, "base_xyz", base_xyz, yaml_full_path, verbose, exit_code);
                    get_yaml_val_or_die(adjacent_robot, "base_rpy", base_rpy, yaml_full_path, verbose, exit_code);
                    p.adjacent_robot_names.push_back(robot_name);
                    p.adjacent_robot_bases_xyz.push_back(base_xyz);
                    p.adjacent_robot_bases_rpy.push_back(base_rpy);
                }
            }
        } catch(const YamlValueNotFound &ex) {
            log()->error("loadYamlParameters({}) got exception({}); RETHROW", yaml_full_path, ex.what());
            throw;
        }

        //  Try to get RobotParameters that have default values:
        try {
            get_yaml_val_or_throw(param, "safety_scale_factor", p.safety_scale_factor, yaml_full_path, verbose);
        } catch (const YamlValueNotFound &ignore) {
            // use default established in RobotParameters.h
        }

        try {
            get_yaml_val_or_throw(param, "live_collision_distance_threshold", p.live_collision_distance_threshold, yaml_full_path, verbose);
        } catch (const YamlValueNotFound &ignore) {
            // use default established in RobotParameters.h
            log()->warn("Unable to find in YAML the parameter 'live_collision_distance_threshold',"
                "instead using default live_collision_distance_threshold = {}", p.live_collision_distance_threshold);
        }
        try {
            get_yaml_val_or_throw(param, "plan_collision_distance_threshold", p.plan_collision_distance_threshold, yaml_full_path, verbose);
        } catch (const YamlValueNotFound &ignore) {
            // use default established in RobotParameters.h
            log()->warn("Unable to find in YAML the parameter 'plan_collision_distance_threshold',"
                "instead using default plan_collision_distance_threshold = {}", p.plan_collision_distance_threshold);
        }
        try {
            get_yaml_val_or_throw(param, "trajectory_divisions_to_check", p.trajectory_divisions_to_check, yaml_full_path, verbose);
        } catch (const YamlValueNotFound &ignore) {
            // use default established in RobotParameters.h
        }

        ConstructPaths(p);
        p.yaml_source_full_path = std::string(yaml_full_path);

        logger->debug("loadYamlParameters: urdf_filepath: {}", p.urdf_filepath); // KEEP
        logger->debug("loadYamlParameters: urdf_robot_only_filepath: {}", p.urdf_robot_only_filepath); // KEEP
        return p;
    }


    void ConstructPaths(RobotParameters &p, const std::string& _traj_lib_base_dir) // KEEP
    {
        //some general combinations which are useful
        std::string home_dir = utils::get_home_dir() + "/";
        p.urdf_filepath = p.urdf_dir + p.urdf; // KEEP
        p.urdf_robot_only_filepath = p.urdf_dir + p.urdf_robot_only; // KEEP
        std::string ldn = p.log_dir_name;
        // <path/to/log_dir>, typically $HOME/log_robot, where logs are $HOME/log_robot/<program>/<date>
        p.log_base_dir = (ldn == "") ? "" : home_dir + ((ldn.back() == '/') ? ldn : ldn + "/");
        /** Stored IK Solution Files
         * (1) By default all directory paths are defined relative to <base_dir>
         * (2) All direction paths have a trailing slash '/'
         * (3) <base_dir> is defined relative to the run_file path or absolute if defined with a leading '/'
         * (4) folder structure: <path_to_base_dir>/<workstation>/<utensil>/<container>/<action_type>/<type>
         */

        // std::string local_path;
        // std::string utensil_path = p.utensil_type + "_" + p.utensil_size;
        // if (p.stored_ik_dir != "") {
        //     local_path = p.stored_ik_dir + "/" + p.workstation_name + "/" + generator_type + "/" + utensil_path + "/" + p.container_frame;
        // } else {
        //     local_path = p.workstation_name + "/" + generator_type + "/" + utensil_path + "/" + p.container_frame;
        // }
        // return local_path;

        // FIXED @sprax: this way of building up the directory structure is cumbersome and inflexible.
        // DONE @sprax: rewrite it, keeping the base dir separate and not necessarily based on HOME.
        // NOTE p.traj_lib_base_dir remains empty if p.traj_lib_dir_name is empty.
        // NOTE: Non-empty values of variables whose names end with '_dir' should end with '/'.
        // NOTE: Non-empty values of variables whose names end with '_name' should not end with '/'.
        // But sometimes we fix-up wrongly-valued variables just to be nice...
        if ( _traj_lib_base_dir.size() ) {
            p.traj_lib_base_dir = _traj_lib_base_dir.back() == '/' ? _traj_lib_base_dir : _traj_lib_base_dir + "/";
        } else if (p.traj_lib_dir_name.size()) {
            // Don't be so nice:
            // const std::string& tldn = p.traj_lib_dir_name;
            // p.traj_lib_base_dir = tldn.back() == '/' ? home_dir + tldn : home_dir + tldn + "/";
            p.traj_lib_base_dir = home_dir + p.traj_lib_dir_name + "/";
        }

        p.workstation_dir = p.traj_lib_base_dir + p.workstation_name + "/";     // "<path_to_base_dir>/<workstation>/"
        std::string utensil_name = p.utensil_type + "_" + p.utensil_size;
        p.utensil_dir = p.workstation_dir + utensil_name + "/";       // DEFINED BELOW: "<path_to_base_dir>/<workstation>/<utensil_name>/"
        p.static_conf_dir = p.workstation_dir + "static_conf" + "/";
        p.static_traj_dir = p.workstation_dir + "static_traj" + "/";
        p.container_dir = p.utensil_dir + p.container_frame + "/";

        // TODO: switch based on file type enum from client
        p.attach_states_file_name = p.container_frame + "_attach_states.json";
        p.approach_states_file_name = p.container_frame + "_approach_states.json";
        p.finish_states_file_name = p.container_frame + "_finish_states.json";
        p.dispense_states_file_name = p.container_frame + "_dispense_states.json";
        p.detach_states_file_name = p.container_frame + "_detach_states.json";

        p.tag_poses_file_name = p.container_frame + "_tag_poses.json";
        p.tc_poses_file_name = p.container_frame + "_tc_poses.json";

        p.UpdateUrdf(p.urdf);
        p.UpdateUrdfRobotOnly(p.urdf_robot_only);
    }

}  // namespace franka_driver
