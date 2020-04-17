///@file: util_math.cc
#include "util_math.h"
#include "momap_log.h"

using drake::trajectories::PiecewisePolynomial;

namespace std {
    string to_string(const drake::Isometry3<double>& pose) {
        Eigen::Vector3d    vec3(pose.translation());
        Eigen::Quaterniond quat(pose.linear());
        return fmt::format("iso3[v_xyz({:5.4} {:5.4} {:5.4}) q_wxyz({:.4} {:.4} {:.4} {:.4})]"
                          , vec3(0), vec3(1), vec3(2), quat.w(), quat.x(), quat.y(), quat.z());
    }

    ostream &operator<<(ostream &os, drake::Isometry3<double> const &pose) {
        return os << to_string(pose);
    }
}

namespace utils {

Eigen::MatrixXd polynominalConfigurationSpline( int num_actuatable_joints_
                                              , std::vector<Eigen::VectorXd> postures
                                              , std::vector<double> trajectory_times
) {
    int num_points = trajectory_times.size();
    //// #unused: double duration = trajectory_times.at(num_points-1);

    Eigen::MatrixXd q_seed;
    q_seed.resize(num_actuatable_joints_, num_points);
    q_seed.setZero();

    for (size_t i = 0; i < postures.size(); i++) {
        q_seed.col(i) = postures[i];
    }
    //log()->info("q_seed: {}", q_seed);
    return q_seed;
}


void printConfigurations(std::vector<Eigen::VectorXd> result)
{
    if (result.size() > 0) {
        log()->info("\n***** Results *****");
        for (size_t i = 0; i < result.size(); i++) {
            log()->info("result {}: {}", i, result.at(i).transpose());
        }
    } else {
        log()->warn("No valid configuration given.");
    }
}

std::vector<double> e_to_v(Eigen::VectorXd e) {
    std::vector<double> v;
    v.resize(e.size());
    Eigen::VectorXd::Map(&v[0], e.size()) = e;
    return v;
}

std::string e_to_s(const Eigen::MatrixXd& mat)
{
    std::stringstream ss;
    ss << mat;
    return ss.str();
}

std::vector<float> e_to_vf(Eigen::VectorXd e) {
    Eigen::VectorXf ef = e.cast<float>();
    std::vector<float> v;
    v.resize(ef.size());
    Eigen::VectorXf::Map(&v[0], ef.size()) = ef;
    return v;
}

template <> Eigen::VectorXd v_to_e(std::vector<double> v) {
    return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v.data(), v.size());
}

template <> Eigen::VectorXd v_to_e(std::vector<float> v)
{
    Eigen::VectorXf evf = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(v.data(), v.size());
    return evf.template cast <double> ();
}

template <> Eigen::VectorXd v_to_e(std::vector<int> v)
{
    Eigen::VectorXi evi = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(v.data(), v.size());
    return evi.template cast <double> ();
}

template <> Eigen::MatrixXd ev_to_m(const std::vector<Eigen::VectorXd> ev) {
    assert(ev.size() && "evec empty");
    Eigen::MatrixXd m(ev.front().size(), ev.size());
    for (int i = 0; i < m.cols(); i++) {
        m.col(i) = ev[i];
        // log()->info("col: {} = {}", i, v.transpose());
    }
    return m;
}

template <> Eigen::MatrixXd ev_to_m(const std::vector<Eigen::Vector3d> ev)
{
    assert(ev.size() && "evec empty");
    Eigen::MatrixXd m(ev.front().size(), ev.size());
    for (int i = 0; i < m.cols(); i++) {
        m.col(i) = ev[i];
    }
    return m;
}

std::vector<Eigen::VectorXd> m_to_ev(const Eigen::MatrixXd m)
{
    std::vector<Eigen::VectorXd> ev;
    for (int i = 0; i < m.cols(); i++) {
        ev.push_back(m.col(i));
    }
    return ev;
}

std::vector<drake::MatrixX<double>> m_to_dm(const Eigen::MatrixXd m)
{
    std::vector<drake::MatrixX<double>> dm;
    for (int i = 0; i < m.cols(); i++) {
        dm.push_back(m.col(i));
    }
    return dm;
}

template <> std::vector<Eigen::VectorXd> to_evx(std::vector<Eigen::Vector3d> t)
{
    std::vector<Eigen::VectorXd> out(t.begin(), t.end());
    return out;
}

template <> std::vector<Eigen::VectorXd> to_evx(Eigen::MatrixXd t) {
    std::vector<Eigen::VectorXd> ev;
    for (int i = 0; i < t.cols(); i++) {
        Eigen::VectorXd v = t.col(i);
        // log()->info("col: {} = {}", i, v.tranpose());
        ev.push_back(v);
    }
    return ev;
}

template <> std::vector<Eigen::Vector3d> to_ev3(std::vector<Eigen::VectorXd> t) {
    std::vector<Eigen::Vector3d> out(t.begin(), t.end());
    return out;
}

template <> std::vector<Eigen::Vector3d> to_ev3(Eigen::Matrix3Xd t) {
    std::vector<Eigen::Vector3d> ev;
    if (t.rows() != 3) {
        log()->warn("to_ev3(Eigen::MatrixXd t) invoked with wrong size matrix");
        log()->info("rows: {}  cols: {}", t.rows(), t.cols());
    }
    for (int i = 0; i < t.cols(); i++) {
        Eigen::Vector3d v = t.col(i);
        // log()->info("col: {} = {}", i, v.tranpose());
        ev.push_back(v);
    }
    return ev;
}

template <> double sum_angular_distance(Eigen::VectorXd t1, Eigen::VectorXd t2)
{
    Eigen::VectorXd delta = t1 - t2;
    double sum = delta.cwiseAbs().sum();
    return sum;
}

/// Euclidean distance
double linear_distance(Eigen::VectorXd vA, Eigen::VectorXd vB)
{
    Eigen::VectorXd delta = vA - vB;
    return delta.norm();
}

/// Euclidean distance squared
double squared_distance(Eigen::VectorXd vA, Eigen::VectorXd vB)
{
    Eigen::VectorXd delta = vA - vB;
    return delta.squaredNorm();
}

// NOTE: Eigen::Quaterniond(equat_to_evec(q)) != q;
// NOTE: e = equat_to_evec(q); Eigen::Quaterniond(e(0), e(1), e(2), e(3)) == q
// This is a genuine WTF moment. See drake documentation (or search Drake source for "gross").
Eigen::Vector4d equat_to_evec(Eigen::Quaterniond equat) {
    Eigen::Vector4d evec(equat.w(), equat.x(), equat.y(), equat.z());
    return evec;
}

Eigen::Quaterniond evec_to_equat(Eigen::Vector4d evec) {
    Eigen::Quaterniond equat(evec(0), evec(1), evec(2), evec(3));
    return equat;
}

drake::Isometry3<double> iso_from_ev_q(const Eigen::Vector3d& vec3, const Eigen::Quaterniond& quat)
{
    return drake::Translation3<double>(vec3) * drake::AngleAxis<double>(quat);
}

template <> bool is_normalized(const Eigen::Vector3d t) {
    return EpsEq(t.norm(), 1.0, 1.0e-6);
}

template <> bool is_normalized(const Eigen::Quaterniond t) {
    return EpsEq(t.norm(), 1.0, 1.0e-6);
}


bool TransformPoseByOffset(Eigen::Vector3d& xyz, Eigen::Quaterniond& quat, const Eigen::Vector3d xyz_offset) {

    drake::math::RotationMatrixd rot_matrix(quat);

    drake::math::RigidTransformd T_AB(rot_matrix, xyz);

    drake::math::RotationMatrixd identity_rot = drake::math::RotationMatrixd::Identity();
    drake::math::RigidTransformd T_BC(identity_rot, xyz_offset);

    drake::math::RigidTransformd T_AC = T_AB * T_BC;

    Eigen::Quaterniond quat_AC(T_AC.rotation().matrix());

    xyz = T_AC.translation();
    quat = quat_AC;

    return true;
}


/* splits trajectory name into start and end conf name by delimiter "_to_" */
void split_trajectory_name(const std::string traj_name, std::string& start_conf, std::string& end_conf)
{
    auto start = 0U;
    std::string delim = "_to_";
    auto end = traj_name.find(delim);
    start_conf = traj_name.substr(start, end-start);
    start = end + delim.length();
    end = traj_name.find(delim, start);
    end_conf = traj_name.substr(start, end-start);
    log()->info("start: {} end: {}", start_conf, end_conf);
}


std::ostream& vc::operator << (std::ostream &o, const valid_conf &v) {
    o << "name: " << v.name <<", ID: " << v.index << std::endl;
    o << "point: ";
    for (auto& element : v.xyz) {
        o << element << ", ";
    }
    o << std::endl;
    o << "gaze: ";
    for (auto& element : v.q) {
        o << element << ", ";
    }
    o << std::endl;
    return o;
}


double vc::findClosestState( Eigen::Vector3d point
                           , const std::vector<valid_conf>& valid_confs
                           , valid_conf &closest_valid_conf
                           , double min_dist
)  {
    for (auto& valid_conf : valid_confs) {
        if (v_to_e(valid_conf.xyz).cwiseAbs().sum() > 0.1 &&
            v_to_e(valid_conf.conf).cwiseAbs().sum() > 0.1 &&
            v_to_e(valid_conf.q).norm() > 0.9
        ) {
            Eigen::Vector3d delta_xyz = point - v_to_e(valid_conf.xyz);
            double dist = delta_xyz.norm();
            if (min_dist > dist) {
                min_dist = dist;
                closest_valid_conf = valid_conf;
            }
        }
    }
    return min_dist;
}

double vc::findClosestState( Eigen::Vector3d point
                           , Eigen::Vector4d q
                           , std::vector<valid_conf>& states
                           , valid_conf &closest_state
)  {
    double min_dist = std::numeric_limits<double>::max();
    std::vector<valid_conf> valid_states = states;
    for (auto& state : states) {
        if ( v_to_e(state.xyz).cwiseAbs().sum() > 0.1
                && v_to_e(state.conf).cwiseAbs().sum() > 0.1
                && v_to_e(state.q).norm()>0.9) {
            Eigen::Vector3d delta_xyz = point - v_to_e(state.xyz);
            Eigen::Vector4d q1 = v_to_e(state.q);
            double q1_dot_q = q1.dot(q);
            double q_error = 1.0 - q1_dot_q * q1_dot_q;
            double dist = delta_xyz.norm() + q_error;
            if (min_dist > dist) {
                min_dist = dist;
                closest_state = state;
            }
            valid_states.push_back(state);
        }
    }
    states = valid_states;
    return min_dist;
}

double vc::findClosestState( Eigen::Vector3d point
                           , std::vector<std::vector<valid_conf> > state_families
                           , std::vector<valid_conf>& closest_state_family
                           , int index_to_compare
) {
  int idx = index_to_compare;
  double min_dist = 1.0;
  for (auto& state_family : state_families) {
      Eigen::Vector3d delta = point - v_to_e(state_family[idx].xyz);
      double dist = delta.norm();
      if (dist < min_dist) {
          min_dist = dist;
          closest_state_family = state_family;
      }
  }
  return min_dist;
}

vc::valid_conf vc::buildValidState(int index, Eigen::VectorXd conf, Eigen::Vector3d xyz, Eigen::Vector4d q) {
    return {"name", index, e_to_v(conf), e_to_v(xyz), e_to_v(q)};
}

vc::valid_conf vc::buildValidState(std::string name, int index, Eigen::VectorXd conf, Eigen::Vector3d xyz, Eigen::Vector4d q) {
    return {name, index, e_to_v(conf), e_to_v(xyz), e_to_v(q)};
}

namespace vc{
    template <> bool remove(valid_conf member, std::vector<valid_conf>& set) {
        int id = member.index;
        for (size_t i = 0; i < set.size(); i++) {
            if (id == set.at(i).index) {
                set.erase(set.begin()+i);
                return true;
            }
        }
        return false;
    }

    template <> bool remove(std::vector<valid_conf> member, std::vector<std::vector<valid_conf> >& set) {
        int id = member[1].index;
        log()->info("id: ");
        for (size_t i = 0; i < set.size(); i++) {
            if (id == set[i].at(1).index) {
                set.erase(set.begin()+i);
                return true;
            }
        }
        return false;
    }

    bool matchByIndex( valid_conf conf_to_match
                     , valid_conf& matching_conf
                     , std::vector<valid_conf> possible_matching_confs
    ) {
        for (auto& possible_conf : possible_matching_confs) {
            if (conf_to_match.index == possible_conf.index) {
                matching_conf = possible_conf;
                return true;
            }
        }
        return false;
    }

}   //  namespace vc

/**
 * computes distance in joint space
 */
double TrajPathLength(std::vector<Eigen::VectorXd> trajectory)
{
  double path_length = 0;
    for (size_t i = 1; i < trajectory.size(); i++) {
        // start at 1, because taking successive differences
        auto current_conf = trajectory[i-1];
        auto next_conf = trajectory[i];
        double delta_path_length = (next_conf - current_conf).array().cwiseAbs().sum();
        path_length += delta_path_length;
    }
    return path_length;
}

double angle_error( const Eigen::Vector3d& gaze_0, const Eigen::Vector3d& gaze_1)
{
    return acos(gaze_0.dot(gaze_1));    // a*b*cos(theta)
}

double quaternion_error( const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2)
{
    double q1_dot_q2 = q1.dot(q2);
    //// FIXME @sprax: return sqrt(1.0 - q1_dot_q2 * q1_dot_q2);
    return pow( pow( 1.0 - q1_dot_q2 * q1_dot_q2, 2), 0.5);
}

double quaternion_error(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2)
{
    Eigen::Quaterniond q1_eigen;
    Eigen::Quaterniond q2_eigen;
    q1_eigen.w() = q1(0);
    q1_eigen.x() = q1(1);
    q1_eigen.y() = q1(2);
    q1_eigen.z() = q1(3);

    q2_eigen.w() = q2(0);
    q2_eigen.x() = q2(1);
    q2_eigen.y() = q2(2);
    q2_eigen.z() = q2(3);
    return quaternion_error(q1_eigen, q2_eigen);
}

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
                                                 , int num_points
) {
    std::vector<Eigen::Vector3d> container_state;
    Eigen::Vector3d n_surf = surface_normal / (surface_normal.norm());
    log()->debug("DRUM:make_container_state: n_surf: {}", n_surf.transpose());
    double dx = dimensions(0);
    double dy = dimensions(1);
    int nx = int(round(sqrt((double)num_points) * (dx / dy) ) );
    int ny = int(round(sqrt((double)num_points) * (dy / dx) ) );
    double dzdx = n_surf(0) / n_surf(2);
    double dzdy = n_surf(1) / n_surf(2);
    log()->debug("DRUM:make_container_state:  dz/dx {}   dz/dy {}", dzdx, dzdy);
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            double x_local = dx * double(i) / double(nx);
            double y_local = dy * double(j) / double(ny);
            // n0*x + n1*y + n2*z = h*n2 --> (0,0) = h
            double z_local = (x_local - dx/2) * dzdx + (y_local - dy/2) * dzdy + height;
            if (z_local > dimensions(2)) {
                log()->info("z: {} > container depth: {}. setting to {}", z_local, dimensions(2), dimensions(2));
                z_local = dimensions(2);
            }
            if (z_local < 0.0) {
                log()->info("z: {} below bottom of container. setting to 0.0", z_local);
                 z_local = 0.0;
            }
            Eigen::Vector3d pt_local(x_local, y_local, z_local);
            Eigen::Vector3d pt_global = pt_local + start_point;
            container_state.push_back(pt_global);
        }
    }
    return container_state;
}

/// NOTE: returns true if trajectory is empty or of size 1.
bool IsContinuous(const std::vector<Eigen::VectorXd>& trajectory, double tol, double *last)
{
    bool result = true;
    double maxd = 0.0;
    for (size_t i = 1; i < trajectory.size(); i++) {
        maxd = (trajectory[i] - trajectory[i-1]).cwiseAbs().maxCoeff();
        if (tol < maxd) {
            momap::log()->debug("IsContinuous FALSE: tol < max_abs_dif: {} < {}", tol, maxd);
            result = false;
            break;
        }
    }
    if (last) {
        *last = maxd;
    }
    return result;
}

Eigen::MatrixXd ShiftMatrix(const Eigen::MatrixXd& mat, const Eigen::Vector3i& shift)
{
    Eigen::MatrixXd result = -Eigen::MatrixXd::Ones(mat.rows(), mat.cols());
    //$ z shift

    int x_shift = shift[0];
    int y_shift = shift[1];
    int x_width = mat.rows() - abs(x_shift);
    int y_width = mat.cols() - abs(y_shift);

    int target_x_origin = x_shift >= 0 ? x_shift : 0;
    int source_x_origin = x_shift >= 0 ? 0 : -x_shift;

    int target_y_origin = y_shift >= 0 ? y_shift : 0;
    int source_y_origin = y_shift >= 0 ? 0 : -y_shift;

    // std::cout << "target_x_origin: " << target_x_origin << std::endl;
    // std::cout << "target_y_origin: " << target_y_origin << std::endl;
    // std::cout << "source_x_origin: " << source_x_origin << std::endl;
    // std::cout << "source_x_origin: " << source_x_origin << std::endl;

    result.block(target_x_origin, target_y_origin, x_width, y_width) = mat.block(source_x_origin, source_y_origin, x_width, y_width);

    result += shift[2] * Eigen::MatrixXd::Ones(mat.rows(), mat.cols());
    return result;
}

Eigen::MatrixXd Gaussian2d( const int rows
                          , const int cols
                          , const int x_0
                          , const int y_0
                          , const double sigma
) {
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(rows, cols);
    for (long i = 0; i < rows; i++) {
        for (long j = 0; j < cols; j++) {
            result(i, j) = (1.0 / sqrt(2.0 * sigma)) * exp( - (pow((i - x_0), 2) + pow((j - y_0), 2) / pow(sigma, 2)));
        }
    }
    return result;
}

size_t SparseDiff( const Eigen::MatrixXd& a
                 , const Eigen::MatrixXd& b
                 , Eigen::MatrixXd& result
) {
    result = Eigen::MatrixXd::Zero(a.rows(), a.cols());
    size_t num_nonzero = 0;
    for (long i = 0; i < a.rows(); i++) {
        for (long j = 0; j < a.cols(); j++) {
            if (a(i, j) >= 0 and b(i, j) >= 0) {
                result(i, j) = a(i, j) - b(i, j);
                num_nonzero++;
            }
        }
    }
    return num_nonzero;
}

double SparseMse( const Eigen::MatrixXd& a
                , const Eigen::MatrixXd& b
) {
    Eigen::MatrixXd sparse_diff;
    size_t num_nonzero = SparseDiff(a, b, sparse_diff);
    log()->debug("nonzeros: {}", num_nonzero);
    return sparse_diff.cwiseProduct(sparse_diff).sum() / num_nonzero;
}

std::vector<Eigen::Vector3i> UniqueUnitVectorsXYZ() {
    std::vector<Eigen::Vector3i> vectors;
    vectors.push_back(Eigen::Vector3i( 1, 0, 0));
    vectors.push_back(Eigen::Vector3i(-1, 0, 0));
    vectors.push_back(Eigen::Vector3i( 0, 1, 0));
    vectors.push_back(Eigen::Vector3i( 0,-1, 0));
    vectors.push_back(Eigen::Vector3i( 0, 0, 1));
    vectors.push_back(Eigen::Vector3i( 0, 0,-1));
    return vectors;
}

Eigen::Vector3i AlignMatrices( const Eigen::MatrixXd& a
                             , const Eigen::MatrixXd& b
                             , const Eigen::Vector3i shift
) {
    double loss_init = SparseMse(a, ShiftMatrix(b, shift));
    Eigen::Vector3i best_shift = shift;
    double lowest_grad = 0;
    std::vector<double> grads;
    std::vector<Eigen::Vector3i> shifts;

    for (const auto uv : UniqueUnitVectorsXYZ()) {
        Eigen::Vector3i new_shift = shift + uv;
        if ((abs(new_shift[0]) <= a.rows()) and
            (abs(new_shift[1]) <= a.cols()))
            // (abs(new_shift.sum()) > 0))
        {
            //$ TODO: add to attempted shifts?
            double loss_shift = SparseMse(a, ShiftMatrix(b, new_shift));
            double d_loss = loss_shift - loss_init;
            grads.push_back(d_loss);
            shifts.push_back(new_shift);
        }
    }

    int lowest_grad_idx = std::min_element(grads.begin(), grads.end()) - grads.begin();
    lowest_grad = grads[lowest_grad_idx];
    log()->debug("AlignMatrices: min gradient: {}", lowest_grad);

    if (lowest_grad < 0) {
        best_shift = shifts[lowest_grad_idx];
        log()->debug("AlignMatrices: best shift: {}", best_shift.transpose());
        return AlignMatrices(a, b, best_shift);
    }
    else {
    log()->info("AlignMatrices: RESULT: {}", best_shift.transpose());
        return best_shift;
    }
}

bool ApplySpeedLimit( const robot_conf_vector_t& trajectory
                    , std::vector<double>& times_in_sec
                    , double RadPerSec
                    , double min_duration
) {
    size_t size = trajectory.size();
    if (size == 0) {
        return false;
    }
    times_in_sec = std::vector<double>();
    double last_time = 0.0;
    times_in_sec.push_back(last_time);
    for (size_t j = 1; j < size; j++) {
        double joint_distance = utils::max_angular_distance(trajectory[j-1], trajectory[j]);
        double duration = 1.001 * (joint_distance / RadPerSec);

        // @dmsj commented as done with debugging
        // log()->info("max angle: {}, duration: {}", joint_distance, duration);

        // Adjust time interval to comply with speed limit
        if (duration < min_duration) {
            duration = min_duration;
        }
        last_time = last_time + duration;
        times_in_sec.push_back(last_time);
    }
    assert(times_in_sec.front() < 0.001);
    assert(times_in_sec.size() == trajectory.size());
    return true;
}

bool ObeysSpeedLimit( const robot_conf_vector_t& trajectory
                    , const std::vector<double>& times_in_seconds
                    , const Eigen::VectorXd max_joint_velocities
) {
    if (trajectory.size() < 2 || times_in_seconds.size() < 2) {
        log()->warn("ObeysSpeedLimit: input trajectory has fewer than 2 points!");
        return false;
    }
    log()->debug("duration is: {}", times_in_seconds.back() - times_in_seconds.front());
    // std::cout << trajectory.front().transpose() << std::endl;
    // std::cout << max_joint_velocities.transpose() << std::endl;
    assert(trajectory.front().size() == max_joint_velocities.size());

    // log()->info("trajectory[0].transpose(): {}", trajectory[0].transpose());
    for (size_t i = 1; i < trajectory.size(); ++i) {
        // check if knot exceeds max velocity
        double t_delta = (times_in_seconds[i] - times_in_seconds[i-1]);
        Eigen::VectorXd ang_speed = ((trajectory[i] - trajectory[i-1]) / t_delta).cwiseAbs();
        if ( ((ang_speed - max_joint_velocities).array() > 0).any()) {
            log()->info("ObeysSpeedLimit: joint velocity:\n{}\n{}"
                , ang_speed.transpose(), max_joint_velocities.transpose()
            );
            log()->error("TS:ObeysSpeedLimit: plan has motions: {} at point_{}/{} > max velocity: {}!"
                              , ang_speed.array().maxCoeff(), i, trajectory.size(), max_joint_velocities.array().maxCoeff()
            );
            return false;
        }
    }
    log()->debug("ObeysSpeedLimit -> true");
    return true;
}

bool ObeysSpeedLimit( const PPType& polynomial
                    , const Eigen::VectorXd max_joint_velocities
) {
    // calculate derivatives of polynomial (it does it for all segments):
  PPType polynomial_derivative = polynomial.derivative(1);
  PPType polynomial_second_derivative = polynomial.derivative(2);
  PPType polynomial_third_derivative = polynomial.derivative(3);

  // walk through all segments to compare derivatives to limits
  for (int segment = 0; segment < polynomial.get_number_of_segments();
       segment++) {
    // go through each segment time:
    double knot_time = polynomial.get_segment_times()[segment];
    // knot point in configuration space:
    // matrix is only a single column vector
    Eigen::VectorXd p_knot_joint = polynomial.value(knot_time).col(0);
    // matrix is only a single column vector
    Eigen::VectorXd v_knot_joint =
        polynomial_derivative.value(knot_time).col(0);
    // matrix is only a single column vector
    Eigen::VectorXd a_knot_joint =
        polynomial_second_derivative.value(knot_time).col(0);

    assert(v_knot_joint.size() == max_joint_velocities.size());

    if ( ((v_knot_joint - max_joint_velocities).array() > 0).any()) {
        log()->info("TS: ObeysSpeedLimit: joint velocity:\n{}\n{}"
            , v_knot_joint.transpose(), max_joint_velocities.transpose()
        );
        log()->error("TS:ObeysSpeedLimit: plan has motions: {} at point_{}/{} > max velocity: {}!"
                          , v_knot_joint.array().maxCoeff(), segment, polynomial.get_number_of_segments(), max_joint_velocities.array().maxCoeff()
        );
        return false;
    }

  }


  return true;
}


conf_vector_t PolynomialToConfVector(const PPType& polynomial) {
  // extract all knot points and times
  robot_conf_vector_t trajectory;
  std::vector<double> times_in_seconds;
  const auto break_times = polynomial.get_segment_times();
  for (size_t knot_num = 0; knot_num < break_times.size(); knot_num++) {
    double break_time = break_times[knot_num];
    Eigen::VectorXd conf = polynomial.value(break_time).col(0);
    times_in_seconds.push_back(break_time);
    trajectory.push_back(conf);
  }
  // TODO @dmsj: return times_in_seconds as well
  return trajectory;
}


bool MeanStandardDev(const std::vector<double> v, double& mean, double& stdev) {
    if (!v.size()) {
        log()->error("dru:MeanStandardDev: called on empty vector!");
        return false;
    }
    double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
    mean =  sum / v.size();

    double accum = 0.0;
    std::for_each (std::begin(v), std::end(v), [&](const double d) {
        accum += (d - mean) * (d - mean);
    });

    stdev = sqrt(accum / (v.size()-1));
    log()->debug("dru:MeanStandardDev: mean: {} stdev: {}", mean, stdev);
    return true;
}

}   //  namespace utils
