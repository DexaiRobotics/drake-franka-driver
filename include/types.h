/// @file types.h
#pragma once

#include <Eigen/Dense>  // in: #include "drake/common/eigen_types.h"

#include <vector>

typedef unsigned int uint;
typedef Eigen::VectorXd robot_conf_t;
typedef std::vector<robot_conf_t> robot_conf_vector_t;
typedef robot_conf_vector_t conf_vector_t;  // shorter alias

enum PauseCommandType { CONTINUE = 0, PAUSE = 1, CANCEL_PLAN = 2 };
