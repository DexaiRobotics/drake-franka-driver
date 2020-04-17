/// @file: util_math.h
#pragma once

/// Namespaced utilities for Drake and Eigen math.
#include "types.h"
#include "momap_log.h"

#include "lcmtypes/robot_spline_t.hpp"
#include "lcmtypes/robot_joint.hpp"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/math/rotation_matrix.h"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <algorithm>    // std::max

using drake::math::RotationMatrix;
using drake::math::RotationMatrixd;
using Eigen::Matrix3d;
using lcmtypes::robot_spline_t;
using lcmtypes::robot_joint;

#define kDefaultTrajectoryTime 10000000     // microseconds
#define kDefaultPointsInTrajectory 50
// IK parameters
#define kSafetyCollisionEpsilon 0.001       // meters
#define kDefaultCollisionEpsilon 0.002      // meters

// TODO: A) better name?  B) Or can we express it in terms of global max speed?  C) Move it to YAML config.
#define kAngularLocalSlowDownFactor 1.1   // maximum local slow-down/speed-up factor (slow-down if value > 1.0)

#define kDefaultMajorIterations 500

typedef drake::trajectories::PiecewisePolynomial<double> PPType;

namespace std {
    string to_string(const drake::Isometry3<double>& pose);
    ostream &operator<<(ostream &os, drake::Isometry3<double> const &pose);
}

namespace utils
{
    using momap::log;

    static Eigen::IOFormat CleanFmt(3, 0, ", ", "\n", "[", "]");

    Eigen::MatrixXd polynominalConfigurationSpline( int num_actuatable_joints_
                                                  , std::vector<Eigen::VectorXd> postures
                                                  , std::vector<double> trajectory_times
    );

    void printConfigurations(std::vector<Eigen::VectorXd> result);

    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    std::vector<double> e_to_v(Eigen::VectorXd e);
    std::vector<float> e_to_vf(Eigen::VectorXd e);
    std::string e_to_s(const Eigen::MatrixXd& mat);

    template <typename T> Eigen::VectorXd v_to_e(std::vector<T> v);
    template <typename T> std::vector<Eigen::VectorXd> to_evx(T t);
    template <typename T> std::vector<Eigen::Vector3d> to_ev3(T t);
    template <typename T> Eigen::MatrixXd ev_to_m(std::vector<T> ev);
    std::vector<Eigen::VectorXd> m_to_ev(Eigen::MatrixXd m);
    std::vector<drake::MatrixX<double>> m_to_dm(const Eigen::MatrixXd m);

    double angle_error( const Eigen::Vector3d& gaze_0, const Eigen::Vector3d& gaze_1);
    double quaternion_error( const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);
    double quaternion_error(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);

    template <typename T> double max_angular_distance(T t1, T t2)
    {
        T delta = t1 - t2;
        double max = delta.cwiseAbs().maxCoeff();
        return max;
    }

    template <typename T> double sum_angular_distance(T start_conf, T end_conf);
    double linear_distance(Eigen::VectorXd vA, Eigen::VectorXd vB);
    double squared_distance(Eigen::VectorXd vA, Eigen::VectorXd vB);

    /// @deprecated: not used.  Declared but no longer implemented (yaml-cpp changes).
    Eigen::VectorXd y_to_e(YAML::Node y);

    Eigen::Vector4d equat_to_evec(Eigen::Quaterniond q);
    Eigen::Quaterniond evec_to_equat(Eigen::Vector4d e);

    drake::Isometry3<double> iso_from_ev_q(const Eigen::Vector3d& vec3, const Eigen::Quaterniond& quat);


    /// TODO: Move EpsEq and VectorEpsEq to a separate header.
    /// Determines if two values are 'close enough' based on a scaled tolerance
    template<typename T>
    bool EpsEq( T a, T b, T rel_epsilon = std::numeric_limits<T>::epsilon()
              , typename std::enable_if<std::is_floating_point<T>::value, T>::type* = 0
    ) {
        T value_range[] = {T(1.0), a, b};
        return (std::abs(a - b) <= rel_epsilon * *std::max_element(value_range, value_range + 3));
    }

    /// TODO: change to use iterators so can handle more than just Eigen types.
    /// NOTE: Container of type ContainerT should contain values of type ValueT.
    /// template<typename ContainerT, typename ValueT>
    /// bool VectorEpsEq(ContainerT a, ContainerT b, ValueT relTol =
    ///         std::numeric_limits<ValueT>::epsilon()) {

    template<typename T>
    bool VectorEpsEq(T a, T b, double relTol = std::numeric_limits<double>::epsilon())
    {
        if (a.size() != b.size()) {
            return false;
        }
        for (int i = 0; i < a.size(); i++) {
            if (! EpsEq(double(a(i)), double(b(i)), relTol)) {
                return false;
            }
        }
        return true;
    }

    template<typename T>
    T IsometryError(drake::Isometry3<T> I_A, drake::Isometry3<T> I_B)
    {
        T delta_translation = (I_A.translation() - I_B.translation()).norm();
        Eigen::Quaternion<T> q_A = Eigen::Quaternion<T>(I_A.linear());
        Eigen::Quaternion<T> q_B = Eigen::Quaternion<T>(I_B.linear());
        Eigen::AngleAxis<T> angle_axis = Eigen::AngleAxis<T>(q_A*(q_B.inverse()));
        T delta_angle = fabs(angle_axis.angle());
        // log()->info("deltas: translation: {} rotation: {}", delta_translation, delta_angle);
        return delta_translation + delta_angle;
    }

    template<typename T>
    drake::Isometry3<T> IsometrySubtract(drake::Isometry3<T> I_A, drake::Isometry3<T> I_B)
    {
      Eigen::Quaternion<T> q_A = Eigen::Quaternion<T>(I_A.linear());
      Eigen::Quaternion<T> q_B = Eigen::Quaternion<T>(I_B.linear());
      auto delta_q  = q_A*(q_B.inverse());
      // Eigen::AngleAxis<T> angle_axis = Eigen::AngleAxis<T>(q_A*(q_B.inverse()));
      // Eigen::Matrix<T, 3, 1> r_A =  I_A.translation();
      // Eigen::Matrix<T, 3, 1> r_B =  I_B.translation();
      return drake::Translation3<T>(I_A.translation() - I_B.translation()) * drake::AngleAxis<T>(delta_q);
      // return (r_A-r_B) * angle_axis;
    }

    template<typename T>
    drake::Isometry3<T> IsometryAdd(drake::Isometry3<T> I_A, drake::Isometry3<T> I_B)
    {
      Eigen::Quaternion<T> q_A = Eigen::Quaternion<T>(I_A.linear());
      Eigen::Quaternion<T> q_B = Eigen::Quaternion<T>(I_B.linear());
      auto delta_q  = q_A*q_B;
      return drake::Translation3<T>(I_A.translation() + I_B.translation()) * drake::AngleAxis<T>(delta_q);
      // Eigen::AngleAxis<T> angle_axis = Eigen::AngleAxis<T>(q_A*q_B);
      // // Eigen::Matrix<T, 3, 1> r_A =  I_A.translation();
      // // Eigen::Matrix<T, 3, 1> r_B =  I_B.translation();
      // return (r_A + r_B) * angle_axis;
    }

    /// compare two Eigen vectors at a given index.
    class ev_compare
    {
        int index;
    public:
        ev_compare(int i) : index(i) {}

        bool operator()(Eigen::VectorXd a, Eigen::VectorXd b) {
            assert (index < a.size());
            assert (a.size() == b.size());
            return (a(index) < b(index));
        }
    };

    template<typename T>
    T median_odd(std::vector<T> vec)
    {
        const auto median_it = vec.begin() + vec.size() / 2;
        std::nth_element(vec.begin(), median_it , vec.end());
        return *median_it;
    }

    /** returns median value of even-length vector */
    template<typename T>
    T median_even(std::vector<T> vec)
    {
        const auto median_it1 = vec.begin() + int(vec.size() / 2) - 1;
        const auto median_it2 = vec.begin() + int(vec.size() / 2);
        assert(median_it1 == median_it2-1);
        return (*median_it1 + *median_it2) / 2;
    }

    /** returns median value of any-length vector.  But the template type must be floating_point
     * (float, double, long double) or it will not compile (as in: `error: no matching function`).
     */
    template<typename T>
    T median(std::vector<T> vec, typename std::enable_if<std::is_floating_point<T>::value, T>::type* = 0)
    {
        if (vec.empty()) {
            return std::numeric_limits<T>::signaling_NaN();
        }
        return (vec.size() % 2) ? median_odd(vec) : median_even(vec);
    }

    /// checks if all necessary fields in have been initialized
    template<typename T>
    bool is_syntax_valid(const T type);

        /* splits trajectory name into start and end conf name by delimiter "_to_" */
    void split_trajectory_name(const std::string traj_name, std::string& start_conf, std::string& end_conf);

    inline bool InsertInSet(std::set<std::string>& setOfStrs, std::string str) {
        std::pair<std::set<std::string>::iterator, bool> result; // A pair of set iterator and bool
        result = setOfStrs.insert(str); // Insert Returns a pair of iterator and bool
        return result.second; // Check if element added sucessfuly
    };


    /// Makes the %RotationMatrix `R_AB` associated with rotating a frame B
    /// relative to a frame A by an angle `theta` about unit vector `Ax = Bx`.
    /// @param[in] theta radian measure of rotation angle about Ax.
    /// @note Orientation is same as Eigen::AngleAxis<T>(theta, Vector3d::UnitX().
    /// @note `R_AB` relates two frames A and B having unit vectors Ax, Ay, Az and
    /// Bx, By, Bz.  Initially, `Bx = Ax`, `By = Ay`, `Bz = Az`, then B undergoes
    /// a right-handed rotation relative to A by an angle `theta` about `Ax = Bx`.
    /// ```
    ///        ⎡ 1       0                 0  ⎤
    /// R_AB = ⎢ 0   cos(theta)   -sin(theta) ⎥
    ///        ⎣ 0   sin(theta)    cos(theta) ⎦
    /// ```
    template<typename T>
    RotationMatrix<T> MakeProjectedXRotation(const T& theta, T *quality_factor = nullptr)
    {
        const T c = std::cos(theta), s = std::sin(theta);
        drake::Matrix3<T> rot_mat;
        // clang-format off
        rot_mat <<  1,  0,  0,
                    0,  c, -s,
                    0,  s,  c;
        // clang-format on
        return RotationMatrix<T>::ProjectToRotationMatrix(rot_mat, quality_factor);
    }

    template<typename T>
    RotationMatrix<T> MakeProjectedYRotation(const T& theta, T *quality_factor = nullptr)
    {
        const double c = std::cos(theta), s = std::sin(theta);
        drake::Matrix3<T> rot_mat;
        // clang-format off
        rot_mat <<   c,  0,  s,
                     0,  1,  0,
                    -s,  0,  c;
        // clang-format on
        return RotationMatrix<T>::ProjectToRotationMatrix(rot_mat, quality_factor);
    }


    template<typename T>
    RotationMatrix<T> MakeProjectedZRotation(const T& theta, T *quality_factor = nullptr)
    {
        const double c = std::cos(theta), s = std::sin(theta);
        drake::Matrix3<T> rot_mat;
        // clang-format off
        rot_mat << c, -s,  0,
                   s,  c,  0,
                   0,  0,  1;
        // clang-format on
        // return RotationMatrix<T>(rot_mat);
        return RotationMatrix<T>::ProjectToRotationMatrix(rot_mat, quality_factor);
    }

    /// A possibly more expensive way of making a RotationMatrix; uses SVD decomp.
    /// Used to avoid std::logic_error: Error: Rotation matrix is not orthonormal.
    /// Apparently only needed on (some?) Darwin Macs.
    template<typename T>
    RotationMatrix<T> MakeProjectedRotation( const drake::Matrix3<T>& rot_mat
                                           , T *quality_factor = nullptr    // receives value near 1.0
    ) {
        return RotationMatrix<T>::ProjectToRotationMatrix(rot_mat, quality_factor);
    }

    /// Generic RotationMatrix creator that works on Darwin and Ubuntu:
    template<typename T>
    RotationMatrix<T> MakeRotationMatrix(const drake::Matrix3<T>& rot_mat)
    {
    #ifdef __APPLE__
        return RotationMatrix<T>::ProjectToRotationMatrix(rot_mat);
    #else
        return RotationMatrix<T>(rot_mat);
    #endif
    }


    /// Creates the rotation matrix R, where R*a = b
    /// sourced from https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
    template<typename T>
    RotationMatrix<T> R_vec2vec( const Eigen::Matrix<T, 3, 1> v_a_inp
                               , const Eigen::Matrix<T, 3, 1> v_b_inp
                               , double *quality_factor = nullptr   // expect value set to near 1.0, if not NULL
    ) {
        Eigen::Matrix<T, 3, 1> v_a = v_a_inp;
        Eigen::Matrix<T, 3, 1> v_b = v_b_inp;
        if (quality_factor != nullptr) {
            *quality_factor = 1.0;
        }
        if ((v_a - v_a/v_a.norm()).norm() > 1.0e-6) {
            throw std::runtime_error("utils::R_vec2vec: v_a is not normalized!");
        }
        if ((v_b - v_b/v_b.norm()).norm() > 1.0e-6) {
            throw std::runtime_error("utils::R_vec2vec: v_b is not normalized!");
        }
        T c = v_a.dot(v_b);
        if (EpsEq(c, -1.0, 1.0e-6) ) {
            // make a random orthogonal vector and rotate around it by pi radians
            Eigen::Matrix<T, 3, 1> v_rand = Eigen::Matrix<T, 3, 1>::Random();
            v_rand -= (v_rand.dot(v_a) * v_a) / v_a.squaredNorm();
            v_rand.normalize();
            ///////////////////////////////////////////////////////////////////
            /// NOTE: The old way, commented below, got this Drake exception as
            /// of 2019.02:
            /// "Error: Rotation matrix is not orthonormal.  Measure of orthonormality error: 4.9989E-10
            ///  (near-zero is good).  To calculate the proper orthonormal rotation matrix closest to
            ///  the alleged rotation matrix, use the SVD (expensive) method
            ///  RotationMatrix::ProjectToRotationMatrix(),
            ///  or for a less expensive (but not necessarily closest) rotation matrix,
            ///  use the constructor RotationMatrix<T>(ToQuaternion(your_Matrix3)).
            ///  Alternately, if using quaternions, ensure the quaternion is normalized."
            ///
            /// Eigen::AngleAxis<T> pi_v_rand = Eigen::AngleAxis<T>(M_PI, v_rand);
            /// return drake::math::RotationMatrix<T>(pi_v_rand);
            ///
            /// So we will adopt ProjectToRotationMatrix for our purposes here.
            /// Instead of insisting on the low tolerance above, it can return a measure
            /// of the non-orthonormality for reference (*quality_factor).
            auto pi_v_rand = Eigen::AngleAxis<T>(M_PI, v_rand).matrix();
            return drake::math::RotationMatrix<T>::ProjectToRotationMatrix(pi_v_rand, quality_factor);
        }
        Eigen::Matrix<T, 3, 1> v_v = v_a.cross(v_b);
        // T s = v_v.norm();

        Eigen::Matrix<T, 3, 3> R_vx = Eigen::Matrix<T, 3, 3>::Zero();
        R_vx <<  0,      -v_v(2),   v_v(1)
               , v_v(2),  0     ,   -v_v(0)
               , -v_v(1), v_v(0),   0;
        Eigen::Matrix<T, 3, 3> R = Eigen::Matrix<T, 3, 3>::Identity() + R_vx + R_vx * R_vx/ (1 + c);
        return drake::math::RotationMatrix<T>(R);
    }

    template<typename T>
    bool is_normalized(const T t);

    bool TransformPoseByOffset(Eigen::Vector3d& xyz, Eigen::Quaterniond& quat, const Eigen::Vector3d xyz_offset);

    namespace vc {
        // a simple struct to define a datapoint
        struct valid_conf {
            std::string name;
            int index;
            std::vector<double> conf;
            std::vector<double> xyz;
            std::vector<double> q;
        };

        std::ostream& operator << (std::ostream &o, const valid_conf &vc);

        /**
         * Search vector of confs to find the conf which is closest in cartesian distance.
         * Return the distance between found and target.
        */

        /// WARNING: This one used to have unspecified (and undesirable) side effects:
        ///          if input states were valid, i.e. had distance < the unspedified magid number 10.0,
        ///          then they were appened to a "valid_states" vector, which was actually never returned...
        double findClosestState( Eigen::Vector3d point
                               , const std::vector<valid_conf>& valid_confs
                               , valid_conf &closest_valid_conf
                               , double min_dist = 10.0
        );
        double findClosestState( Eigen::Vector3d point
                               , Eigen::Vector4d q
                               , std::vector<valid_conf>& states
                               , valid_conf &closest_state
        );
        double findClosestState( Eigen::Vector3d point
                               , std::vector<std::vector<valid_conf> > state_families
                               , std::vector<valid_conf>& closest_state_family
                               , int index_to_compare
        );

        /**
         * Search vector of confs to find the conf which is closest in cartesian and quaternion distance.
         * Return cartesian distance and quaternion distance??
        */
        inline double findAlignedClosestState( Eigen::Vector3d point
                                        ////, Eigen::Vector4d quaternion                  //// #unused
                                            , std::vector<valid_conf> states
                                            , valid_conf &closest_state
        ) {
            return findClosestState(point, states, closest_state);
        }

        /**
         * Construct a valid_conf object from Eigen types
        */
        valid_conf buildValidState(int index, Eigen::VectorXd conf, Eigen::Vector3d xyz, Eigen::Vector4d q);
        valid_conf buildValidState(std::string name, int index, Eigen::VectorXd conf, Eigen::Vector3d xyz, Eigen::Vector4d q);

        template <typename T> bool remove(T member, std::vector<T>& set);

        bool matchByIndex(valid_conf conf_to_match, valid_conf &matching_conf, std::vector<valid_conf> possible_matching_confs);
    }   // namespace vc


    double measurePlanDistance( Eigen::VectorXd last_configuration
                              , std::vector<Eigen::VectorXd> trajectory
    );
    /**
     * computes distance in joint space
     */
    double TrajPathLength(std::vector<Eigen::VectorXd> trajectory);

    /**
    * Specify the container in the same way as SampleContainer()
    * origin = center of the bottom surface in world coordinates
    * dimenions = max extent in X, Y, Z
    * surface_normal = vector (will be normalized) describing the plane ax + by + cz = d
    * height = point the plane passes through over the origin on the container
    * nx * ny = n; nx = (n^0.5) * dx/dy;
    */
    std::vector<Eigen::Vector3d> make_container_state( Eigen::Vector3d start_point
                                                     , Eigen::Vector3d dimensions
                                                     , Eigen::Vector3d surface_normal
                                                     , double height
                                                     , int num_points = 1024
    );

    /// Methods to check a trajectory of confs or robot_states for discontinuities,
    /// that is, jumps between neighboring points that exceed the given distance
    /// tolerance.  Distance between neighbors is measured as the
    /// max absolute difference between components, that is:
    /// (pointA - pointB).cwiseAbs().maxCoeff().
    /// CAUTION: These methods return true for trajectories that are empty or of size 1.
    /// Migrated here from ConstraintSolver.
    bool IsContinuous(const std::vector<Eigen::VectorXd>& traj, double tol = 0.05, double *last = nullptr);


    Eigen::MatrixXd ShiftMatrix(const Eigen::MatrixXd& mat, const Eigen::Vector3i& shift);

    Eigen::MatrixXd Gaussian2d( const int rows
                              , const int cols
                              , const int x_0
                              , const int y_0
                              , const double sigma
    );

    //$ return a - b for indices where a and b are nonnegative, otherwise 0
    size_t SparseDiff( const Eigen::MatrixXd& a
                     , const Eigen::MatrixXd& b
                     , Eigen::MatrixXd& result
    );

    double SparseMse( const Eigen::MatrixXd& a
                    , const Eigen::MatrixXd& b
    );

    std::vector<Eigen::Vector3i> UniqueUnitVectorsXYZ();

    Eigen::Vector3i AlignMatrices( const Eigen::MatrixXd& a
                                 , const Eigen::MatrixXd& b
                                 , const Eigen::Vector3i shift = Eigen::Vector3i(0, 0, 0)
    );

    inline double MicrosecondsToSeconds(const int64_t& utime) {
      return double(utime) / 1e6;
    }

    inline int64_t SecondsToMicroseconds(const double& time_s) {
      return int64_t(time_s * 1e6);
    }

  /**
  * Computes derivatives via finite difference and generates a time series which
  * sets the fastest joint at the desired RadPerSec. Does not work well. Replaced
  * with TrajectorySolver::SplineTrajectory
  * @deprecated - this is still in use in a test repo which is intended to be
  * updated to more modern code, but that project is on the back-burner.
  */
  bool ApplySpeedLimit( const robot_conf_vector_t& trajectory
                      , std::vector<double>& times_in_sec
                      , double RadPerSec
                      , double min_duration
  );

  /**
  * Computes derivatives via finite difference and verifies that none of these
  * exceed the specifed max joint velocities.
  * @deprecated - it is preferred to use the polynomial version and the
  * robot_conf_vector_t version is scheduled for removal.
  * TODO: @dmsj review full code base and verify that this overload can be removed.
  */
  bool ObeysSpeedLimit( const robot_conf_vector_t& trajectory
                      , const std::vector<double>& times_in_seconds
                      , const Eigen::VectorXd max_joint_velocities
  );

  /**
  * Verifies that no joint velocity, as computed by the polynomial derivative,
  * at any of the polynomial knot points, exceeds the specifed max joint velocities.
  * NOTE: TOPPRA oftens fails to generate trajectories which meet the provided
  * acceleration limit, so that testing is explicitly not present in this function.
  * TODO: @dmsj - rename to ObeysDynamicsLimits and add functionality for testing
  * acceleration, jerk, and joint limits. Once TOPPRA generates trajectories which
  * meet those limits.
  */
  bool ObeysSpeedLimit( const PPType& polynomial
                      , const Eigen::VectorXd max_joint_velocities
  );


  /**
  * Conversion function which returns the conf_vector_t of the polynomial
  * knot points.
  * TODO @dmsj: Convert a polynomial with derivative and second derivative
  * information to a conf_vector_t
  */
  conf_vector_t PolynomialToConfVector(const PPType& polynomial);

  /**
  * Calculate mean and standard deviation of a vector.
  */
  bool MeanStandardDev(const std::vector<double> v, double& mean, double& stdev);


}   // namespace utils
