cmake_minimum_required(VERSION 3.14)
project(drake-franka-driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# TODO(dyt) enable -Werror
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -D_GLIBCXX_USE_CXX11_ABI=1")

set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")

# add macOS support
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_RPATH})

# threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Set Drake's path
set(DRAKE_BUILD_DIR "/opt/drake")
set(FRANKA_BUILD_DIR "${CMAKE_SOURCE_DIR}/externals/libfranka/build")

# copy test data to build directory
configure_file(tests/data/franka_test.urdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
configure_file(tests/data/franka_test.yaml "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)

# Find LCM
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

list(APPEND CMAKE_PREFIX_PATH ${DRAKE_BUILD_DIR})
list(APPEND CMAKE_PREFIX_PATH ${DRAKE_BUILD_DIR}/lib/cmake)
list(APPEND CMAKE_PREFIX_PATH ${DRAKE_BUILD_DIR}/lib/cmake/drake)

list(APPEND CMAKE_PREFIX_PATH ${FRANKA_BUILD_DIR})

find_package(drake REQUIRED)
find_package(Franka REQUIRED)
find_package(yaml-cpp REQUIRED COMPONENTS shared)

message(STATUS "Franka_INCLUDE_DIRS: ${Franka_INCLUDE_DIRS}")
message(STATUS "Franka_LIBRARIES: ${Franka_LIBRARIES}")

find_package(lcm REQUIRED)
find_package(gflags MODULE REQUIRED COMPONENTS shared)
find_package(GTest REQUIRED)
find_package(fcl REQUIRED)

find_package( Python3 3.6 REQUIRED Interpreter Development NumPy)
message(STATUS "Python3_VERSION: ${Python3_VERSION}")
message(STATUS "Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3_LIBRARY_DIRS: ${Python3_LIBRARY_DIRS}")
message(STATUS "Python3_LIBRARIES: ${Python3_LIBRARIES}")
message(STATUS "Python3_NumPy_FOUND: ${Python3_NumPy_FOUND}")
message(STATUS "Python3_NumPy_INCLUDE_DIRS: ${Python3_NumPy_INCLUDE_DIRS}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

# message("Setting SPDLOG_INCLUDE_DIR; environment:" $ENV{SPDLOG_INCLUDE_DIR})
if (DEFINED ENV{SPDLOG_INCLUDE_DIR})
    set(SPDLOG_INCLUDE_DIR $ENV{SPDLOG_INCLUDE_DIR})
    message(STATUS "SPDLOG_INCLUDE_DIR variable taken from environment: " ${SPDLOG_INCLUDE_DIR})
else()
    set(SPDLOG_INCLUDE_DIR /opt/drake/include/spdlog)
    message(STATUS "SPDLOG_INCLUDE_DIR variable defaulted in CMakeList: " ${SPDLOG_INCLUDE_DIR})
endif()
# message(STATUS "SPDLOG_INCLUDE_DIR variable used by cmake: " ${SPDLOG_INCLUDE_DIR})


set(INCLUDES 
    include
    ${SPDLOG_INCLUDE_DIR}
    ${YAML_INCLUDES}
    ${FILESYSTEM_INCLUDE_DIR}
    ${Franka_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
) 

# add include dir for Franka headers
include_directories(
    ${INCLUDES}
)


set(SOURCES 
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

# build the lcm driver that uses Franka Robot Interface (FRI)
# add_definitions(-std=c++14)
set(driver_name franka_driver)
add_executable(${driver_name} ${SOURCES}
)

set(test_name send_stop)
add_executable(${test_name}
    src/send_stop.cc
)

target_link_libraries(${test_name}
    ${LCM_NAMESPACE}lcm
)

set(LINK_LIBRARIES 
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

target_link_libraries(${driver_name} 
    ${LINK_LIBRARIES}
)

# install the lcm driver
# install(TARGETS ${driver_name}
#     RUNTIME DESTINATION bin
#     LIBRARY DESTINATION lib
# ARCHIVE DESTINATION lib)


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
