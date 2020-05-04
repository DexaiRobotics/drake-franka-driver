// @file  robot_parameters.cc
#include "robot_parameters.h"
#include "util_string.h"

#include <pwd.h>
#include <stdexcept>

namespace franka_driver {

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
            std::string error_msg = "P:loadYamlParameters: Unable to find robot_parameters YAML("
                                  + yaml_full_path + ")";
            log()->error(error_msg);
            throw std::runtime_error(error_msg);
            return p;               // would return empty/default, except for throw.
        }

        YAML::Node config = YAML::LoadFile(yaml_full_path);
        if ( ! config["robot_parameters"]) {
            std::string error_msg = "P:loadYamlParameters: No robot_parameters found in ("
                                  + yaml_full_path + ")";
            logger->error(error_msg);
            throw std::runtime_error(error_msg);
        }
        int config_length = static_cast<int>(config["robot_parameters"].size());
        logger->debug("P:loadYamlParameters: robot_parameters length: {}", config_length);

        YAML::Node param = config["robot_parameters"];

        std::string home_dir = utils::get_home_dir() + "/";

        // ******************** RobotParameters ********************

        //  Try to get RobotParameters that have no default values.  Don't ignore failures:
        try {

            // World Configuration
            get_yaml_val_or_die(param, "gravity_vector", p.gravity_vector, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "world_frame", p.world_frame, yaml_full_path, verbose, exit_code);

            //// Geometric Configuration
            get_yaml_val_or_die(param, "urdf", p.urdf, yaml_full_path, verbose, exit_code);

            // Ensure that p.urdf_dir is either "" or else begins with $HOME ends in "/":
            std::string udr;
            get_yaml_val_or_die(param, "urdf_dir", udr, yaml_full_path, verbose, exit_code);
            p.urdf_dir = (udr == "") ? "" : home_dir + ((udr[udr.size()-1] == '/') ? udr : udr + "/");

            // Robot properties: Degrees Of Freedom (num joints), and so on.
            get_yaml_val_or_die(param, "simulated", p.simulated, yaml_full_path, verbose, exit_code);

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

            // Robot Control RobotParameters
            p.lcm_status_channel               = p.robot_name + "_STATUS";
            p.lcm_plan_channel                 = p.robot_name + "_PLAN";
            p.lcm_plan_received_channel        = p.robot_name + "_PLAN_RECEIVED";
            p.lcm_plan_complete_channel        = p.robot_name + "_PLAN_COMPLETE";
            p.lcm_stop_channel                 = p.robot_name + "_STOP";

            get_yaml_val_or_die(param, "lcm_url", p.lcm_url, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "robot_ip", p.robot_ip, yaml_full_path, verbose, exit_code);

            get_yaml_val_or_die(param, "kLooseJointDistance", p.kLooseJointDistance, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "kMediumJointDistance", p.kMediumJointDistance, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "kTightJointDistance", p.kTightJointDistance, yaml_full_path, verbose, exit_code);

            get_yaml_val_or_die(param, "robot_high_joint_limits", p.robot_high_joint_limits, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "robot_low_joint_limits", p.robot_low_joint_limits, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "robot_max_velocities", p.robot_max_velocities, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "robot_max_accelerations", p.robot_max_accelerations, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "robot_max_jerks", p.robot_max_jerks, yaml_full_path, verbose, exit_code);

            get_yaml_val_or_die(param, "stop_epsilon", p.stop_epsilon, yaml_full_path, verbose, exit_code);
            get_yaml_val_or_die(param, "stop_margin", p.stop_margin, yaml_full_path, verbose, exit_code);

        } catch(const YamlValueNotFound &ex) {
            log()->error("loadYamlParameters({}) got exception({}); RETHROW", yaml_full_path, ex.what());
            throw;
        }

        ConstructPaths(p);
        p.yaml_source_full_path = std::string(yaml_full_path);
        logger->debug("loadYamlParameters: urdf_filepath: {}", p.urdf_filepath);
        return p;
    }

    void ConstructPaths(RobotParameters &p, const std::string& _traj_lib_base_dir)
    {
        std::string home_dir = utils::get_home_dir() + "/";
        p.urdf_filepath = p.urdf_dir + p.urdf; // KEEP
        p.UpdateUrdf(p.urdf);
    }

}  // namespace franka_driver
