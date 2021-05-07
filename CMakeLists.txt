cmake_minimum_required(VERSION 3.20)
project(drake-franka-driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wshadow=local")

set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")

# threads
find_package(Threads REQUIRED)

# find drake
set(DRAKE_BUILD_DIR "/opt/drake")
find_package(drake REQUIRED)

# find franka
set(FRANKA_BUILD_DIR "${CMAKE_SOURCE_DIR}/externals/libfranka/build")
list(APPEND CMAKE_PREFIX_PATH ${FRANKA_BUILD_DIR})
find_package(Franka REQUIRED)
message("Franka_INCLUDE_DIRS: ${Franka_INCLUDE_DIRS}")
message("Franka_LIBRARIES: ${Franka_LIBRARIES}")

find_package(yaml-cpp REQUIRED COMPONENTS shared)
find_package(GTest REQUIRED)
find_package(gflags REQUIRED COMPONENTS shared)
find_package(lcm REQUIRED)

# Set ROS Paths for find_package(catkin) to work
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/$ENV{ROS_DISTRO}")
# for finding our components in ~/catkin_ws/src, particularly robot_msgs
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/catkin_ws/devel")
# for lcmtypes definitions in robot_msgs
find_package(catkin REQUIRED COMPONENTS
  robot_msgs
)

if (NOT EXISTS "/usr/include/spdlog/")
  set(SPDLOG_INCLUDE_DIR "/opt/drake/include/spdlog")
  message("SPDLOG not found in system, setting SPDLOG_INCLUDE_DIR: " ${SPDLOG_INCLUDE_DIR})
endif()

# copy test data to build directory
configure_file(tests/data/franka_test.urdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
configure_file(tests/data/franka_test.yaml "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)

# add include dir for Franka headers
include_directories(
  ${CMAKE_SOURCE_DIR}/src
  ${SPDLOG_INCLUDE_DIR}
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

# build the lcm driver that uses Franka Robot Interface (FRI)
set(driver_name franka_driver)
add_executable(franka_driver
  src/driver/franka_driver.cc
  src/driver/franka_plan_runner.cc
  src/driver/communication_interface.cc
  src/driver/constraint_solver.cc
  # src/examples/examples_common.cpp
  src/utils/util_conv.cc
  src/utils/util_io.cc
  src/utils/util_math.cc
  src/utils/util_string.cc
  src/utils/robot_parameters.cc
  src/utils/dexai_log.cc
  src/utils/polynomial_encode_decode.cc
)

# drake on focal cannot propagate HAVE_SPDLOG
# https://github.com/RobotLocomotion/drake/issues/15004
target_compile_definitions(franka_driver PUBLIC HAVE_SPDLOG)

target_link_libraries(franka_driver
  ${Franka_LIBRARIES}
  ${LCM_NAMESPACE}lcm
  gflags
  drake::drake
  yaml-cpp
  Threads::Threads
  cnpy
)

#-------------------
# Unit Testing
#-------------------

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
  GTest::gtest
  GTest::gtest_main
  Threads::Threads
  gflags
  drake::drake
  yaml-cpp
  ${Franka_LIBRARIES}
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
