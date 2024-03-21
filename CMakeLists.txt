cmake_minimum_required(VERSION 3.20)
project(drake-franka-driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wshadow=local")

set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")
find_package(fmt REQUIRED) # 8.0.1
find_package(spdlog REQUIRED) # 1.9.2
find_package(GTest REQUIRED) # 1.11
find_package(Threads REQUIRED)
find_package(drake REQUIRED)

# Set ROS Paths for find_package(catkin) to work
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/$ENV{ROS_DISTRO}")

# for finding our components in ~/catkin_ws/src, particularly robot_msgs
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/catkin_ws/devel")
set(CATKIN_ENABLE_TESTING OFF)

# for lcmtypes definitions in robot_msgs
find_package(catkin REQUIRED COMPONENTS
  robot_msgs
)
include(GoogleTest) # catkin screws up gtest functions

# find franka
find_package(Franka REQUIRED PATHS "${CMAKE_SOURCE_DIR}/externals/libfranka/build" NO_DEFAULT_PATH)
message(DEBUG "Franka_INCLUDE_DIRS: ${Franka_INCLUDE_DIRS}")
message(DEBUG "Franka_LIBRARIES: ${Franka_LIBRARIES}")

find_package(yaml-cpp REQUIRED COMPONENTS shared)

find_package(lcm REQUIRED)

# copy test data to build directory
configure_file(tests/data/franka_test.urdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
configure_file(tests/data/franka_test.yaml "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)

# add include dir for Franka headers
include_directories(
  ${CMAKE_SOURCE_DIR}/src
  ${YAML_INCLUDES}
  ${Franka_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(send_stop
  src/send_stop.cc
)

target_link_libraries(send_stop
  lcm
)

add_library(franka_driver_lib
  src/driver/franka_plan_runner.cc
  src/driver/communication_interface.cc
  src/driver/constraint_solver.cc
  src/utils/util_conv.cc
  src/utils/util_io.cc
  src/utils/util_math.cc
  src/utils/util_string.cc
  src/utils/robot_parameters.cc
  src/utils/dexai_log.cc
  src/utils/polynomial_encode_decode.cc
)

target_include_directories(franka_driver_lib PUBLIC
  ${CMAKE_SOURCE_DIR}/src
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

target_link_libraries(franka_driver_lib PUBLIC
  ${Franka_LIBRARIES}
  ${LCM_NAMESPACE}lcm
  gflags
  drake::drake
  yaml-cpp
  Threads::Threads
  cnpy
)

# build the lcm driver that uses Franka Robot Interface (FRI)
set(driver_name franka_driver)
add_executable(franka_driver
  src/driver/franka_driver.cc
)

# drake on focal cannot propagate HAVE_SPDLOG
# https://github.com/RobotLocomotion/drake/issues/15004
target_compile_definitions(franka_driver PUBLIC HAVE_SPDLOG)

target_link_libraries(franka_driver
  franka_driver_lib
)

# -------------------
# Unit Testing
# -------------------
enable_testing()

set(GTEST_SRC_FILES
  tests/test_franka_driver.cc
  tests/test_util_conv.cc
  tests/test_util_io.cc
  tests/test_util_math.cc
  tests/test_util_string.cc
  tests/test_robot_parameters.cc
  tests/test_constraint_solver.cc
)

set(TEST_INCLUDES
  include
  ${YAML_INCLUDES}
  ${GTEST_INCLUDE_DIRS}
)

set(TEST_SOURCES
  src/driver/constraint_solver.cc
  src/utils/util_conv.cc
  src/utils/util_io.cc
  src/utils/util_math.cc
  src/utils/util_string.cc
  src/utils/robot_parameters.cc
  src/utils/dexai_log.cc
)

set(TEST_LINK_LIBRARIES
  franka_driver_lib
  GTest::gtest
  GTest::gtest_main
)

foreach(_file ${GTEST_SRC_FILES})
  get_filename_component(_name ${_file} NAME_WE)
  include_directories(${TEST_INCLUDES})
  add_executable(${_name}
    ${_file}
    ${TEST_SOURCES}
  )
  target_compile_definitions(${_name} PUBLIC HAVE_SPDLOG)
  target_link_libraries(${_name}
    ${TEST_LINK_LIBRARIES}
  )
  gtest_add_tests(
    TARGET ${_name}
    TEST_LIST unittests)
endforeach()

set_tests_properties(${unittests} PROPERTIES TIMEOUT 270)
