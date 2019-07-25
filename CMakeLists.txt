cmake_minimum_required(VERSION 3.14)
project(drake-franka-driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_RPATH})


## Set up environment
message(STATUS "setting up environment!")

# Set Drake's path
set(DRAKE_BUILD_DIR "/opt/drake")
message(STATUS "DRAKE_BUILD_DIR: ${DRAKE_BUILD_DIR}")

set(FRANKA_BUILD_DIR "${CMAKE_SOURCE_DIR}/externals/libfranka/build")
message(STATUS "FRANKA_BUILD_DIR: ${FRANKA_BUILD_DIR}")


# Figure out where dracula can be found
if (($ENV{CIRCLECI} MATCHES "true") )
    get_filename_component(DEXAI_SRC_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)
    set(ENV{DEXAI_SRC} ${DEXAI_SRC_DIR})
elseif(NOT DEFINED ENV{DEXAI_SRC})
    message(STATUS "NOTE: The environment variable DEXAI_SRC is not defined")
    message(STATUS "NOTE: setting DEXAI_SRC to /src")
    set(ENV{DEXAI_SRC} /src)
endif()
message(STATUS "DEXAI_SRC: $ENV{DEXAI_SRC}")
set(DRACULA_SOURCE_PATH "$ENV{DEXAI_SRC}/dracula")
message(STATUS "DRACULA_SOURCE_PATH: ${DRACULA_SOURCE_PATH}")
set(DRACULA_BUILD_PATH "${DRACULA_SOURCE_PATH}/build")
message(STATUS "DRACULA_BUILD_PATH: ${DRACULA_BUILD_PATH}")
set(DRACULA_INCLUDE_DIR "${DRACULA_SOURCE_PATH}/dracula/include")
message(STATUS "DRACULA_INCLUDE_DIR: ${DRACULA_INCLUDE_DIR}")

# Set Dracula Libraries
if (UNIX AND NOT APPLE)
    set(DRACULA_LIBRARIES "${DRACULA_BUILD_PATH}/dracula/libdracula.so")
elseif (APPLE)
    message ("MAC OS X")
    set(DRACULA_LIBRARIES "${DRACULA_BUILD_PATH}/dracula/libdracula.dylib")
endif ()

set(CTPL_INCLUDE_DIR "${DRACULA_SOURCE_PATH}/externals/CTPL")

#LOCAL_PYTHON=`which python`
#export LOCAL_PYTHON_PATH=`dirname $LOCAL_PYTHON`

## end of setting environment variables


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

# add include dir for Franka headers
include_directories(
    include
    ${DRACULA_INCLUDE_DIR}
    ${DRACULA_INCLUDE_DIR}/../tests
    ${DRACULA_INCLUDE_DIR}/../../externals/json/include
    ${Franka_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
    ${CTPL_INCLUDE_DIR}
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
    ${DRACULA_LIBRARIES}
    ${Python3_LIBRARIES}
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
