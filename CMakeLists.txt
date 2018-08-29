cmake_minimum_required(VERSION 3.5.1)
project(drake-kuka-driver)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_RPATH})

# Find LCM
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

list(APPEND CMAKE_PREFIX_PATH $ENV{DRAKE_BUILD_DIR})
list(APPEND CMAKE_PREFIX_PATH $ENV{DRAKE_BUILD_DIR}/lib/cmake)
list(APPEND CMAKE_PREFIX_PATH $ENV{DRAKE_BUILD_DIR}/lib/cmake/drake)

find_package(drake REQUIRED)
find_package(lcm REQUIRED)
find_package(GFlags MODULE REQUIRED COMPONENTS shared)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")



# add include dir for FRIClient headers
include_directories(${fri_client_build}/include)


# build the lcm driver that uses FRIClient
add_definitions(-std=c++11)
set(driver_name drake_franka_driver)
add_executable(${driver_name} kuka_driver.cc)
target_link_libraries(${driver_name}
                      ${fri_client_library}
                      ${LCM_NAMESPACE}lcm 
                      gflags_shared
                      drake::drake-lcmtypes-cpp 
                      ${catkin_LIBRARIES})

# install the lcm driver
install(TARGETS ${driver_name}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib)

