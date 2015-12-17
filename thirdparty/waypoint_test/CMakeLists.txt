## Minimum cmake version
cmake_minimum_required(VERSION 2.8.3)

## Project name
project(waypoint_test)

# Add custom cmake
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

## Link to required simulator
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rosbag
  hal_quadrotor
)
# Find Boost
find_package(Boost REQUIRED)

# Find GPS toolkit
find_package(GeographicLib REQUIRED)

## Include CRATES libraries
include_directories(${catkin_INCLUDE_DIRS} ${GEOGRAPHICLIB_INCLUDE_DIR})

## For converting the WGS84 coordinates to LTP
add_executable(wgs84_to_ltp wgs84_to_ltp.cpp)
target_link_libraries(wgs84_to_ltp ${GEOGRAPHICLIB_LIBRARY})

## For converting the WGS84 coordinates to LTP
add_executable(fcs_sync fcs_sync.cpp)
target_link_libraries(fcs_sync ${GEOGRAPHICLIB_LIBRARY})

## For dumping the state estimate and control
add_executable(rosbag_dump rosbag_dump.cpp)
target_link_libraries(rosbag_dump ${catkin_LIBRARIES})

## For converting the WGS84 coordinates to LTP
add_executable(dynamics_test dynamics_test.cpp)
target_link_libraries(dynamics_test ${catkin_LIBRARIES})
