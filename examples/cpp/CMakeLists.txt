## Minimum cmake version
cmake_minimum_required(VERSION 2.8.3)

## Project name
project(example)

# Add custom cmake
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}")

## Link to required simulator
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  sim
)

# find GSL
find_package(GSL)
# find Boost
find_package(Boost) # 1.34 COMPONENTS date_time filesystem iostreams ... )

## Include CRATES libraries
include_directories( 
  ${catkin_INCLUDE_DIRS}
	${GSL_INCLUDE_DIRS}
)

## The example executable
add_executable(
  example main.cpp
)

## The PIROS executable
add_executable(
  PIROS PIROS.cpp pi/global.cpp
)

## The piNodeUAV executable
add_executable(
  PINodeUAV PINodeUAV.cpp pi/global.cpp
)

## The example_vel executable
add_executable(
  example_vel example_vel.cpp pi/PIController.cpp pi/ModelHoldingPattern.cpp 
)

## The example_vel_jstate executable
add_executable(
  example_vel_jstate_mainloop example_vel_jstate_mainloop.cpp pi/PIController.cpp pi/ModelHoldingPattern.cpp
)

## The main_muav executable
add_executable(
  main_muav main_muav.cpp pi/PIController.cpp pi/ModelHoldingPattern.cpp
)

## The init_muav executable
add_executable(
  init_muav init_muav.cpp
)

## The land_muav executable
add_executable(
  land_muav land_muav.cpp
)


## Link against CRATES libraries
target_link_libraries(example 
  ${catkin_LIBRARIES}
)

## Link against CRATES libraries
target_link_libraries(PIROS
  ${catkin_LIBRARIES}
)

## Link against CRATES libraries
target_link_libraries(PINodeUAV
  ${catkin_LIBRARIES}
)

## Link against CRATES libraries
target_link_libraries(example_vel 
  ${catkin_LIBRARIES}
	${GSL_LIBRARIES} 
)

## Link against CRATES libraries
target_link_libraries(example_vel_jstate_mainloop 
  ${catkin_LIBRARIES}
	${GSL_LIBRARIES} 
)

## Link against CRATES libraries
target_link_libraries(main_muav 
  ${catkin_LIBRARIES}
	${GSL_LIBRARIES} 
)

## Link against CRATES libraries
target_link_libraries(init_muav 
  ${catkin_LIBRARIES}
)

## Link against CRATES libraries
target_link_libraries(land_muav 
  ${catkin_LIBRARIES}
)


# Test of a simple ros node from the tutorial
add_executable(talker talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

# Test for recording multi uav state
add_executable(muav muav.cpp)
target_link_libraries(muav ${catkin_LIBRARIES})
