cmake_minimum_required(VERSION 3.5.1)
project(drake-franka-driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_RPATH})

# Find LCM
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

list(APPEND CMAKE_PREFIX_PATH $ENV{DRAKE_BUILD_DIR})
list(APPEND CMAKE_PREFIX_PATH $ENV{DRAKE_BUILD_DIR}/lib/cmake)
list(APPEND CMAKE_PREFIX_PATH $ENV{DRAKE_BUILD_DIR}/lib/cmake/drake)

list(APPEND CMAKE_PREFIX_PATH $ENV{FRANKA_BUILD_DIR})


list(APPEND CMAKE_PREFIX_PATH "/usr/local/share/eigen3/cmake")
find_package(Eigen3 3.3 CONFIG REQUIRED)
message(STATUS "Eigen3 use file: ${EIGEN3_USE_FILE}")
message(STATUS "Eigen3 include dirs: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Eigen3 found: ${EIGEN3_FOUND}")


find_package(drake REQUIRED)
find_package(Franka REQUIRED)
find_package(lcm REQUIRED)
find_package(gflags MODULE REQUIRED COMPONENTS shared)
find_package(GTest REQUIRED)
# find_package(spdlog CONFIG REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

# add include dir for Franka headers
include_directories(
	include
	$ENV{DRACULA_INCLUDE_DIR}
	${Franka_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	)


# build the lcm driver that uses Franka Robot Interface (FRI)
add_definitions(-std=c++14)
set(driver_name franka_plan_runner)
add_executable(${driver_name} 
	src/franka_plan_runner.cc
	src/examples_common.cpp
	)
target_link_libraries(${driver_name}
                      ${Franka_LIBRARIES}
                      ${LCM_NAMESPACE}lcm 
                      gflags
					  drake::drake
                      $ENV{DRACULA_LIBRARIES} 
                      )

# install the lcm driver
install(TARGETS ${driver_name}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib)

