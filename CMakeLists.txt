cmake_minimum_required(VERSION 3.19)
project(drake-franka-driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wshadow=local")

set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")

# threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# find drake
set(DRAKE_BUILD_DIR "/opt/drake")
find_package(drake REQUIRED)

# find franka
set(FRANKA_BUILD_DIR "${CMAKE_SOURCE_DIR}/externals/libfranka/build")
list(APPEND CMAKE_PREFIX_PATH ${FRANKA_BUILD_DIR})
find_package(Franka REQUIRED)
message(STATUS "Franka_INCLUDE_DIRS: ${Franka_INCLUDE_DIRS}")
message(STATUS "Franka_LIBRARIES: ${Franka_LIBRARIES}")

find_package(yaml-cpp REQUIRED COMPONENTS shared)
find_package(GTest REQUIRED)
find_package(gflags REQUIRED COMPONENTS shared)
find_package(lcm REQUIRED)
find_package(fcl REQUIRED)

find_package(Python3 REQUIRED Interpreter Development NumPy)
message("Python3_VERSION: ${Python3_VERSION}")
message("Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")
message("Python3_LIBRARY_DIRS: ${Python3_LIBRARY_DIRS}")
message("Python3_LIBRARIES: ${Python3_LIBRARIES}")
message("Python3_NumPy_FOUND: ${Python3_NumPy_FOUND}")
message("Python3_NumPy_INCLUDE_DIRS: ${Python3_NumPy_INCLUDE_DIRS}")

if (DEFINED ENV{SPDLOG_INCLUDE_DIR})
  set(SPDLOG_INCLUDE_DIR $ENV{SPDLOG_INCLUDE_DIR})
  message("SPDLOG_INCLUDE_DIR variable taken from environment: " ${SPDLOG_INCLUDE_DIR})
else()
  set(SPDLOG_INCLUDE_DIR /opt/drake/include/spdlog)
  message("SPDLOG_INCLUDE_DIR variable defaulted in CMakeList: " ${SPDLOG_INCLUDE_DIR})
endif()

# copy test data to build directory
configure_file(tests/data/franka_test.urdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
configure_file(tests/data/franka_test.yaml "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)

# add include dir for Franka headers
include_directories(
  ${CMAKE_SOURCE_DIR}
  ${SPDLOG_INCLUDE_DIR}
  ${YAML_INCLUDES}
  ${FILESYSTEM_INCLUDE_DIR}
  ${Franka_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
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
  src/franka_driver.cc
  src/franka_plan_runner.cc
  src/communication_interface.cc
  src/examples_common.cpp
  src/util_conv.cc
  src/util_io.cc
  src/util_math.cc
  src/util_string.cc
  src/robot_parameters.cc
  src/constraint_solver.cc
  src/dexai_log.cc
  src/polynomial_encode_decode.cc
)

target_link_libraries(franka_driver
  ${Franka_LIBRARIES}
  ${LCM_NAMESPACE}lcm
  gflags
  drake::drake
  yaml-cpp
  Threads::Threads
  ${Python3_LIBRARIES}
  fcl
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
  ${SPDLOG_INCLUDE_DIR}
  ${YAML_INCLUDES}
  ${FILESYSTEM_INCLUDE_DIR}
  ${GTEST_INCLUDE_DIRS}
)

set(TEST_SOURCES
  src/util_conv.cc
  src/util_io.cc
  src/util_math.cc
  src/util_string.cc
  src/robot_parameters.cc
  src/constraint_solver.cc
  src/dexai_log.cc
)

set(TEST_LINK_LIBRARIES
  gtest
  gtest_main
  ${GTEST_BOTH_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
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
  target_link_libraries(${_name}

    ${TEST_LINK_LIBRARIES}
  )
  add_test(${_name} ${_name})
  set_tests_properties(${_name} PROPERTIES TIMEOUT 270)
endforeach()
