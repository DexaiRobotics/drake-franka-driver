///@file util_conv.h
#pragma once

#include "types.h"
// #include "momap/momap_robot_state_v1.h"
#include "Eigen/Geometry"
#include "drake/lcmt_iiwa_status.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <time.h>
#include <sys/time.h>


/// NOTE @sprax: I doubt there is still a need to organized these functions here.
/// They could be elsewhere or be eliminated.
namespace utils {

    /// Convenience Functions for converting between lcm, STL, and Eigen types

    drake::lcmt_iiwa_status EigenToLcmStatus(Eigen::VectorXd robot_state);

    // Sort pairs by first element of pair in ascending order
    bool SortPairAscendingInt(const std::pair<double, int> &a,
                           const std::pair<double, int> &b);
    bool SortPairAscendingEigen(const std::pair<double, Eigen::VectorXd> &a,
                           const std::pair<double, Eigen::VectorXd> &b);

    /** 
    * Comparison function to return the joint space difference between two robot_conf_t
    * TODO @dmsj: move this method into the definition of robot_conf_t
    * @param[in] tol - tolerance that norm of the joint space difference must be within to equate the confs
    * @param[in] target_configuration - the conf to which other confs are compared
    * @param[in] arg - the conf to compare  to target_configuration
    * @return (arg - target_configuration).norm() < tolerance
    */
    struct CompareConfiguration : public std::unary_function<Eigen::VectorXd, bool>
    {
        explicit CompareConfiguration(const Eigen::VectorXd& conf, double tol = 0.001)
                                     : target_configuration(conf), tolerance(tol)
        { }
        bool operator() (const Eigen::VectorXd& arg)
        {
            return (arg - target_configuration).norm() < tolerance;
        }

        Eigen::VectorXd target_configuration;
        double tolerance;
    };


    /**
    * Splits a std::vector of Eigen::VectorXd, aka a robot_conf_vector_t into
    * two vectors with the location_to_split included in both new vectors.
    * The two new vectors when combined contain one more element than the original
    * vector. The two vectors are constructed as 
    * first_part(trajectory.begin(), split_location) and 
    * second_part(split_location, trajectory.end());
    */
    bool SplitTrajectory( const std::vector<Eigen::VectorXd>& trajectory
                        , const Eigen::VectorXd& location_to_split
                        , std::vector<Eigen::VectorXd>& front_traj
                        , std::vector<Eigen::VectorXd>& back_traj
    );

    /** 
    * Split a std::vector of Eigen::VectorXd, aka a robot_conf_vector_t into
    * two vectors with the element which is nearest to location_to_split,
    * but still within the supplied tolerance (as measured by joint_space norm)
    * included in both new vectors. Calls SplitTrajectory after finding the
    * closest element.
    */ 
    bool SplitAtClosest( const std::vector<Eigen::VectorXd>& trajectory
                       , const Eigen::VectorXd& location_to_split
                       , const double tolerance
                       , std::vector<Eigen::VectorXd>& front_traj
                       , std::vector<Eigen::VectorXd>& back_traj
    );

}   //  namespace utils
