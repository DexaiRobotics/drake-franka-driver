/// @file: robot_parameters.h
#pragma once

#include "dexai_log.h"
#include "util_io.h"
#include "util_math.h"
#include "util_string.h"

#include "yaml-cpp/yaml.h"

#include <typeinfo>

namespace franka_driver {

    using dexai::log;

    /// Log a failure to obtain a value for the given key.
    /// Re-maps verbosity to log levels.
    void LogMissingYamlKey(const std::string& key, const int verbose);

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

        // Geometric Configuration
        std::string urdf;
        std::string urdf_dir;               // this is /path/to/urdf_filename.urdf
        std::string urdf_filepath;

        // Robot
        std::string robot_name;
        bool simulated;

        // Robot Control Parameters - generated based on hostname
        std::string lcm_status_channel; 
        std::string lcm_plan_channel;
        std::string lcm_plan_received_channel;
        std::string lcm_plan_complete_channel;
        std::string lcm_stop_channel;

        std::string lcm_url;
        std::string robot_ip;

        double kLooseJointDistance;
        double kMediumJointDistance;
        double kTightJointDistance;

        //Position, velocity, acceleration, and jerk limit
        Eigen::VectorXd robot_high_joint_limits;
        Eigen::VectorXd robot_low_joint_limits;
        Eigen::VectorXd robot_max_velocities;
        Eigen::VectorXd robot_max_accelerations;
        Eigen::VectorXd robot_max_jerks;

        //$ scales velocity and acceleration scale factors passed to toppra
        double stop_epsilon;  // used to calculate when robot counts as "stopped" // KEEP
        double stop_margin;   // how long robot sleeps before ready to recieve continue // KEEP

        bool GenerateParametersBasedOnRobotName(const std::string& new_robot_name);

        bool UpdateRobotName(const std::string& new_robot_name);

        // Given just the file_name, not a full_path, update the urdf_filepath.
        // Should also verify it before return true.
        bool UpdateUrdf(const std::string& new_urdf_file_name); // KEEP

        inline const std::string& YamlSourceFullPath() const { return yaml_source_full_path; }
    };

    /// Read YAML file and set RobotParameters to found values or to defaults.
    /// On error: throws std::runtime_error or YamlValueNotFound on error,
    ///           or, if exit_code != 0, calls exit(exit_code)
    RobotParameters loadYamlParameters( const std::string& params_yaml_path
                                 , int verbose = 1
                                 , drake::logging::logger *logger = nullptr
                                 , int exit_code = 0    // calls exit on error if exit_code != 0
    );

    void ConstructPaths(RobotParameters &p, const std::string& _traj_lib_base_dir = "");

}  // namespace franka_driver
