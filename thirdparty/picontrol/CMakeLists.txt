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

# Find Boost
find_package(Boost REQUIRED COMPONENTS
  thread
) 

## Include CRATES libraries
include_directories( 
  ${catkin_INCLUDE_DIRS}
	${GSL_INCLUDE_DIRS}
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
