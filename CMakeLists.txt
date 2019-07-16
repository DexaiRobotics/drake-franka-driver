cmake_minimum_required(VERSION 3.5.1)
project(drake-franka-driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_RPATH})

configure_file(tests/data/franka_test.urdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
configure_file(tests/data/franka_test.yaml "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)

# Find LCM
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

list(APPEND CMAKE_PREFIX_PATH $ENV{DRAKE_BUILD_DIR})
list(APPEND CMAKE_PREFIX_PATH $ENV{DRAKE_BUILD_DIR}/lib/cmake)
list(APPEND CMAKE_PREFIX_PATH $ENV{DRAKE_BUILD_DIR}/lib/cmake/drake)

list(APPEND CMAKE_PREFIX_PATH $ENV{FRANKA_BUILD_DIR})

find_package(drake REQUIRED)
find_package(Franka REQUIRED)
find_package(lcm REQUIRED)
find_package(gflags MODULE REQUIRED COMPONENTS shared)
find_package(GTest REQUIRED)
find_package(fcl REQUIRED)

set(Python2_ROOT_DIR $ENV{LOCAL_PYTHON_PATH})
find_package( Python2 2.7.12 REQUIRED Interpreter Development)
set(PYTHON_LIBRARIES ${Python2_LIBRARIES} )
set(PYTHON_INCLUDE_DIRS "${Python2_INCLUDE_DIRS}"
    "${Python2_LIBRARY_DIRS}/python2.7/dist-packages/numpy/core/include"
    "${Python2_LIBRARY_DIRS}/python2.7/site-packages/numpy/core/include")

message(STATUS "Python2_LIBRARIES: ${Python2_LIBRARIES}")
message(STATUS "Python2_INCLUDE_DIRS: ${Python2_INCLUDE_DIRS}")
message(STATUS "Python2_LIBRARY_DIRS: ${Python2_LIBRARY_DIRS}")
message(STATUS "PYTHON_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

# add include dir for Franka headers
include_directories(
    include
    $ENV{DRACULA_INCLUDE_DIR}
    $ENV{DRACULA_INCLUDE_DIR}/../tests
    $ENV{DRACULA_INCLUDE_DIR}/../../externals/json/include
    ${Franka_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    $ENV{CTPL_INCLUDE_DIR}
    externals/StopWatch
)


# build the lcm driver that uses Franka Robot Interface (FRI)
# add_definitions(-std=c++14)
set(driver_name franka_plan_runner)
add_executable(${driver_name}
    src/franka_plan_runner.cc
    src/examples_common.cpp
)

set(test_name send_stop)
add_executable(${test_name}
    src/send_stop.cc
)

target_link_libraries(${test_name}
    ${LCM_NAMESPACE}lcm
)


target_link_libraries(${driver_name}
    ${Franka_LIBRARIES}
    ${LCM_NAMESPACE}lcm
    gflags
    drake::drake
    Threads::Threads
    $ENV{DRACULA_LIBRARIES}
    ${PYTHON_LIBRARIES}
    fcl
)

add_executable( ArrayToVectorPerformance
    src/test_vector_performance.cc
    externals/StopWatch/StopWatch.cpp
)
target_link_libraries( ArrayToVectorPerformance
    ${Franka_LIBRARIES}
    ${LCM_NAMESPACE}lcm
    gflags
    drake::drake
    Threads::Threads
    $ENV{DRACULA_LIBRARIES}
    ${PYTHON_LIBRARIES}
    fcl
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
    tests/test_test.cc
)

foreach(_file ${GTEST_SRC_FILES})
    get_filename_component(_name ${_file} NAME_WE)
    add_executable(${_name} ${_file})
    target_link_libraries(${_name}
    gtest gtest_main ${CMAKE_THREAD_LIBS_INIT})
    add_test(${_name} ${_name})
    set_tests_properties(${_name} PROPERTIES TIMEOUT 270)
endforeach()
