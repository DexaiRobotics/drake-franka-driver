/// @file: robot_parameters.h
#pragma once

#include "momap_log.h"
#include "util_io.h"
#include "util_math.h"
#include "util_string.h"

#include "yaml-cpp/yaml.h"

#include <typeinfo>

namespace franka_driver {

    using momap::log;

    extern const std::string kBaseLinkWorldFrame; // World Geometry

    /// Log a failure to obtain a value for the given key.
    /// Re-maps verbosity to log levels.
    void LogMissingYamlKey(const std::string& key, const int verbose);

    /// Find the specified key in the given YAML Node (in its top-level iterator)
    /// and return the corresponding value as a YAML Node,
    /// else throw YamlKeyNotFound.
    /// @see also FindNode below.
    const YAML::Node FindYamlKeyOrThrow( const YAML::Node& parent_node // source node
                                       , const std::string& find_key   // key of top-level node
                                       , const std::string& yaml_file  // file name for logging
                                       , int verbose = 1
    );

    /// FindNode return true if the given YAML Node contains an entry for node_name,
    /// otherwise it returns false.  It tries to catch most exceptions,
    /// and also returns false if it does catch one.
    ///
    /// FindNode is an earlier implementation providing functionality
    /// similar to FindYamlKeyOrThrow.  But instead of returning the
    /// found node or throwing, it returns true or false.
    /// NOTE @sprax: It could be renamed to ContainsNode.
    bool FindNode(YAML::Node node, std::string node_name, drake::logging::logger *logger);

    template <typename T>
    bool get_yaml_val(const YAML::Node& yaml, const std::string& key, T& val)
    {
        const auto found = yaml[key]; // no throw
        if (found) {
            try {
                val = found.as<T>();
                return true;
            } catch (const std::exception& ex) {
                log()->error( "Error in get_yaml_val({}, key={}, T={}): {}"
                            , yaml.Tag(), key, std::string(typeid(T).name()), ex.what()
                );
            }
        }
        return false;
    }

    template <typename T>
    bool get_yaml_val_verbose(const YAML::Node& yaml
        , const std::string& key, T& val, const int verbose = 1
    ) {
        if (get_yaml_val(yaml, key, val)) {
            if (verbose > 2) {
                log()->debug("Loaded [{}] == ({})", key, val);
            }
            return true;
        }
        LogMissingYamlKey(key, verbose);
        return false;
    }

    template <typename T>
    bool get_yaml_val(const YAML::Node& yaml, const std::string& key, std::vector<T>& val)
    {
        auto found = yaml[key];
        if (found) {
            try {
                std::vector<T> vec = found.as< std::vector<T> >();
                val = vec;
                return true;
            } catch (const std::exception& ex) {
                log()->error( "Error in get_yaml_val({}, key={}, Eigen::Vector3d&): {}"
                            , yaml.Tag(), key, ex.what()
                );
            }
        }
        return false;
    }

    template <typename T>
    bool get_yaml_val_verbose( const YAML::Node& yaml
                             , const std::string& key
                             , std::vector<T>& val
                             , const int verbose
    ) {
        if (get_yaml_val(yaml, key, val)) {
            if (verbose > 2) {
                log()->debug("Loaded [{}] == ({})", key, utils::v_to_e(val).transpose());
            }
            return true;
        }
        LogMissingYamlKey(key, verbose);
        return false;
    }

    //  NOTE: explicit specialization must be at namespace scope, not inside a class.
    template <> bool get_yaml_val(const YAML::Node& yaml, const std::string& key, Eigen::Vector3d& val);
    template <> bool get_yaml_val_verbose(const YAML::Node& yaml, const std::string& key, Eigen::Vector3d& val, const int verbose);
    template <> bool get_yaml_val(const YAML::Node& yaml, const std::string& key, Eigen::VectorXd& val);
    template <> bool get_yaml_val_verbose(const YAML::Node& yaml, const std::string& key, Eigen::VectorXd& val, const int verbose);

    class YamlKeyNotFound : public std::runtime_error {
        using std::runtime_error::runtime_error;
        using std::runtime_error::operator=;
    };

    class YamlValueNotFound : public std::runtime_error {
        using std::runtime_error::runtime_error;
        using std::runtime_error::operator=;
    };

    /// Put the value for the given key into val;
    /// else throw YamlValueNotFound with key name and its (mangled) type name.
    template <typename T>
    void get_yaml_val_or_throw( const YAML::Node& yaml
                              , const std::string& key
                              , T& val
                              , const std::string& yaml_file_spec
                              , const int verbose = 1
    ) {
        if (not get_yaml_val_verbose(yaml, key, val, verbose)) {
            std::string debug_msg = fmt::format("No value for key({})/type({}) in YAML file({})"
                                               , key, typeid(T).name(), yaml_file_spec);
            log()->debug(debug_msg);
            throw YamlValueNotFound(debug_msg);
        }
    }

    /// Put the value for the given key into val;
    /// else exit with non-zero exit_code,
    /// or throw YamlValueNotFound with the key name and a given YAML file spec
    template <typename T>
    void get_yaml_val_or_die( const YAML::Node& yaml
                            , const std::string& key
                            , T& val
                            , const std::string& yaml_file_spec
                            , int verbose = 1
                            , int exit_code = 0
    ) {
        if (not get_yaml_val_verbose(yaml, key, val, verbose)) {
            std::string  error_msg = fmt::format("No value for key({}) in YAML file({})"
                                                , key, yaml_file_spec);
            log()->error(error_msg);
            if (exit_code) {        // exit with a non-zero error code, or don't exit at all.
                exit(exit_code);
            }
            throw YamlValueNotFound(error_msg);
        }
    }

    /// RobotParameters
    class RobotParameters {
    public:
        // meta-information
        std::string yaml_source_full_path;

        // World Configuration
        Eigen::Vector3d gravity_vector;
        std::string world_frame;
        double safe_z;
        // Geometric Configuration
        std::string urdf;
        std::string urdf_robot_only;        //$ this only includes the robot arm, no other geometry
        std::string urdf_simple_geo;        //$ use only primitive shapes, no dependency on meshes
        std::string urdf_utensil_in_bin;    //$ no end effector utensil, utensil racked in bin
        std::string urdf_no_utensil;        //$ no end effector utensil, empty bin
        std::string urdf_utensil_attached;  //$ end effector utensil, empty bin
        std::string urdf_dir;               // this is /path/to/urdf_filename.urdf
        std::string urdf_filepath; // KEEP
        std::string urdf_robot_only_filepath;

        // Robot
        int         robot_dof;      // @deprecated.  We take the truth from the URDF anyway.
        std::string robot_name; // KEEP
        std::string workstation_name;
        // tool
        std::string utensil_type;
        std::string utensil_size;
        std::string utensil;
        std::string utensil_tc_frame;
        std::string utensil_tag_frame;
        std::string utensil_face_frame;
        double      utensil_max_serving_volume = 0.0; // initialized from WorkstationParameters
        int         utensil_get_stuff_actuations = 0;
        int         utensil_dispense_actuations = 0;
        // TODO: change all naming to 'tool' from 'utensil'
        std::string tool_frame;
        Eigen::Vector3d tool_axis;
        Eigen::Vector3d tool_face_normal;
        double tool_radius;
        double action_overlap_factor;
        int              action_type;      // current action type (must be initialized)
        std::vector<int> action_types;     // list of action types available (e.g. scoop types)
        bool simulated;

        // container
        std::string container_frame;
        std::string container_type;
        std::string material;
        std::string material_type;
        double material_density; // [g/mL]
        double ingredient_serving_size; // initialized from WorkstationParameters
        std::pair<float, float> mass_limits;
        std::vector<double> surface_mass_fit_coeffs; // coefficients of polynomial fit function to predict mass
                                                     // from below surface integral from highest order term to
                                                     // to zeroth order term
        std::pair<float, float> bsi_limits;   // threshold below surface integral values to filter candidate actions
        Eigen::Vector3d container_corner_transform;
        Eigen::Vector3d container_dimensions;
        Eigen::Vector3d container_inner_dimensions;
        double container_edge;
        // workstation geometry
        // THESE FRAME FIELDS ARE USELESS. DON'T USE THEM FOR ANYTHING SERIOUS.
        std::string tool_changer_frame;
        std::string approach_frame;
        std::string exit_frame;
        std::string dispense_frame;

        Eigen::VectorXd view_conf;

        //$ action search RobotParameters
        double action_index_spatial_map_discretization;
        double container_state_discretization;
        double sme_voxel_size;

        /** Stored IK Solution Files
         * (1) By default all directory paths are defined relative to <base_dir>
         * (2) All direction paths have a trailing slash '/'
         * (3) <base_dir> is defined relative to the run_file path or absolute if defined with a leading '/'
         * (4) folder structure: <path_to_base_dir>/<workstation>/<utensil>/<container>/<action_type>/<type>
         */

        std::string traj_lib_dir_name;      // FROM YAML: name of folder relative to home or base directory
        std::string traj_lib_base_dir;      // <path_to_base_dir>/" absolute or relative to cwd
        std::string workstation_dir;        // traj_lib_base_dir + p.workstation_name   + "/"
        std::string utensil_dir;            // workstation_dir   + p.utensil_name       + "/"
        std::string static_conf_dir;        // p.workstation_dir + "static_conf"        + "/"
        std::string static_traj_dir;        // p.workstation_dir + "static_traj"        + "/"
        std::string container_dir;          // p.utensil_dir     + container_frame      + "/"

        std::string log_dir_name;           // FROM YAML: base log file dir, under $HOME
        std::string log_base_dir;           // <path/to/log_dir>, typically $HOME/log_robot,
                                            // where logs are $HOME/log_robot/<program>/<date>

        std::string attach_states_file_name; //name_only
        std::string approach_states_file_name; //name_only
        std::string finish_states_file_name; //name_only
        std::string dispense_states_file_name; //name_only
        std::string detach_states_file_name; //name_only

        std::string tag_poses_file_name; //name_only
        std::string tc_poses_file_name; //name_only

        //$ Robot Control RobotParameters
        bool prompt_robot_motion; // require user input before executing trajectory
        std::string lcm_status_channel; // KEEP
        std::string lcm_plan_channel; // KEEP
        std::string lcm_plan_received_channel; // KEEP
        std::string lcm_plan_complete_channel; // KEEP
        std::string lcm_stop_channel; // KEEP
        std::string lcm_actuate_tool_channel;
        std::string lcm_open_tool_channel;
        std::string lcm_close_tool_channel;
        std::string lcm_lock_tool_channel;
        std::string lcm_unlock_tool_channel;
        std::string lcm_tool_server_response_channel;
        std::string lcm_tool_open_channel;
        std::string lcm_tool_locked_channel;
        std::string lcm_url; // KEEP
        std::string robot_ip;
        std::string ros_head_camera_ns;
        std::string ros_wrist_camera_ns;
        std::string ros_force_torque_topic;
        std::string head_camera_color_topic;
        std::string head_camera_depth_topic;
        std::string head_camera_info_topic;
        std::string wrist_camera_color_topic;
        std::string wrist_camera_depth_topic;
        std::string wrist_camera_info_topic;

        // Collision
        double kCollisionEpsilonConservative;
        double kCollisionEpsilonMin;
        double kCollisionEpsilonContainer;
        // Motion Control
        double kSplineSampleFreq; // Hz
        double kMaxJointDistanceBetweenKnots; //radians
        std::string splineType; // allowed options: ZeroOrderHold, FirstOrderHold, Cubic
        // IK RobotParameters
        double kLoosePositionEpsilon; // meters
        double kTightPositionEpsilon; // meters
        double kMediumPositionEpsilon; // meters

        double kLooseOrientationEpsilon; // radians
        double kMediumOrientationEpsilon; // radians
        double kTightOrientationEpsilon; // radians

        //$ TODO AFTER GERMANY: standardize on naming convention for RobotParameters
        //$ are the quantifiers a prefix or a suffix?
        double kLooseJointDistance; // KEEP
        double kMediumJointDistance; // KEEP
        double kTightJointDistance; // KEEP

        int kDefaultTrajNumPoints;

        //Position, velocity, acceleration, and jerk limit
        Eigen::VectorXd robot_high_joint_limits;
        Eigen::VectorXd robot_low_joint_limits;
        Eigen::VectorXd robot_max_velocities;
        Eigen::VectorXd robot_max_accelerations;
        Eigen::VectorXd robot_max_jerks;

        //$ scales velocity and acceleration scale factors passed to toppra
        double safety_scale_factor = 0.5;

        double stop_epsilon;  // used to calculate when robot counts as "stopped" // KEEP
        double stop_margin;   // how long robot sleeps before ready to recieve continue // KEEP

        //$ adjacent robot names and locations
        std::vector<std::string>     adjacent_robot_names;
        std::vector<Eigen::Vector3d> adjacent_robot_bases_xyz;
        std::vector<Eigen::Vector3d> adjacent_robot_bases_rpy;

        //$ multi-robot collision checking RobotParameters
        // threshold distance between two collision body pairs for live collision check:
        double live_collision_distance_threshold = 0.01;
        // threshold distance between two collision body pairs for plan collision check:
        double plan_collision_distance_threshold = 0.05;

        int trajectory_divisions_to_check = 50; // subdivision of a trajectory for the purpose of checking colliding poses

        bool GenerateParametersBasedOnRobotName(const std::string& new_robot_name);

        bool UpdateRobotName(const std::string& new_robot_name);

        // Given just the file_name, not a full_path, update the urdf_filepath.
        // Should also verify it before return true.
        bool UpdateUrdf(const std::string& new_urdf_file_name); // KEEP
        bool UpdateUrdfRobotOnly(const std::string& new_urdf_robot_only_file_name); // KEEP

        inline const std::string& YamlSourceFullPath() const { return yaml_source_full_path; }

        static std::string s_def_dump_str_;

        // Log the params using the provided logger.
        // Set the string dump to the output string IFF it is empty.
        // Returns the size of the dump string, even if not set.
        size_t log_dump( drake::logging::logger *logger
                       , int verbose=1
                       , std::string& dumps=s_def_dump_str_
        ) const;
    };

    // It is no longer recommended to instantiate a
    // RobotParameters::RobotParameters by calling
    // RobotParameters::loadYamlParameters(...). Instead, you should create
    // a RobotParameters::WorkstationParameters, initialize it with
    // RobotParameters::WorkstationParameters::LoadFromYaml(...), then get
    // the desired container-specific RobotParameters::RobotParameters with
    // RobotParameters::WorkstationParameters::GetContainerParamsMap()[CONTAINER]

    /// Read YAML file and set RobotParameters to found values or to defaults.
    /// On error: throws std::runtime_error or YamlValueNotFound on error,
    ///           or, if exit_code != 0, calls exit(exit_code)
    RobotParameters loadYamlParameters( const std::string& params_yaml_path
                                 , int verbose = 1
                                 , drake::logging::logger *logger = nullptr
                                 , int exit_code = 0    // calls exit on error if exit_code != 0
    );

    void ConstructPaths(RobotParameters &p, const std::string& _traj_lib_base_dir = "");


    /// Put the value for the given key into val;
    /// else throw YamlValueNotFound with key name and its (mangled) type name.
    /// TODO @sprax: deprecate get_yaml_val_or_throw usage and replace it with this.
    template <typename T>
    void GetYamlValOrThrow( T& val
                          , const std::string& key
                          , const YAML::Node& yaml_node
                          , const fs::path& yaml_path
                          , const int verbose = 1
    ) {
        get_yaml_val_or_throw(yaml_node, key, val, yaml_path, verbose);
    }


    /// Given a template argument T of some container type,
    /// such as a list or vector, and a YAML root or parent node
    /// containing a named list of items that all have an ID field
    /// given by item_id_key, and a value field given by item_val_key,
    /// try to place each corresponding item value in the supplied collection.
    /// If an item's ID matches the default_key ("default" by default), then
    /// its mapped value replaces any existing default value
    /// for the item_val_key.
    template <class T>
    bool GetYamlListWithLabeledDefault( T& collection                   // target std list, set, or vector
                                       , const YAML::Node& parent_node  // source
                                       , const std::string& list_name   // plural yaml key to get the list
                                       , const std::string& item_id_key // single yaml key to ID each item
                                       , const std::string& item_val_key // unique label or "default"
                                       , const fs::path& yaml_path
                                       , const std::string& default_key = "default"
                                       , int verbose = 1
    ) {
        YAML::Node yaml_list = parent_node[list_name];
        auto list_size = yaml_list.size();
        if (list_size == 0) {
            log()->warn("GetYamlListWithLabeledDefault: list_name({}) got size({})"
                , list_name, list_size
            );
            return false;
        };
        unsigned ord = 0;
        typename T::value_type default_item_value, item_value;
        typename T::value_type *default_item_value_ptr = nullptr;
        for (const auto& yaml_item : yaml_list) {
            std::string item_id_value;
            GetYamlValOrThrow(item_id_value, item_id_key, yaml_item, yaml_path, verbose);
            if (item_id_value == default_key) {
                // If the label for this list item is "default" (or the given default_key),
                // save its item value as the default value.
                GetYamlValOrThrow(item_value, item_val_key, yaml_item, yaml_path, verbose);
                default_item_value = item_value;
                default_item_value_ptr = &default_item_value;
            } else {
                // If the label for this list item is not 'default',
                // save its value as a specific value.
                try {
                    GetYamlValOrThrow(item_value, item_val_key, yaml_item, yaml_path, verbose);
                } catch (const YAML::RepresentationException& yaml_ex) {
                    log()->info("GetYamlListWithLabeledDefault: Caught "
                        " YAML::RepresentationException: {}; checking for label instead value."
                        , yaml_ex.what()
                    );
                    // YAML-listed value had wrong type; try std::string label instead.
                    std::string item_value_label;
                    GetYamlValOrThrow(item_value_label, item_val_key, yaml_item, yaml_path, verbose);
                    if (item_value_label == default_key and default_item_value_ptr) {
                        log()->warn("GetYamlListWithLabeledDefault: Found LABEL"
                            " == {} and default value for key: {}"
                            , item_value_label, item_val_key
                        );
                        item_value = *default_item_value_ptr;
                    }
                } catch (const std::exception& std_ex) {
                    log()->error("GetYamlListWithLabeledDefault: Caught std::exception: {}; RE-THROW"
                        , std_ex.what()
                    );
                    throw;
                }
                collection.push_back(item_value);
            }
            if (verbose > 3) {
                log()->info("GetYamlListWithLabeledDefault: ITEM {}: ({})", ++ord, item_value);
            }
        }
        return true;
    }

}  // namespace franka_driver
