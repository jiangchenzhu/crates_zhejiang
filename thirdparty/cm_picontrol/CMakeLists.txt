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

include_directories( 
	/usr/include/eigen3/
)


## The main_muav executable
add_executable(
  main_suav main_suav.cpp pi/PIController.cpp pi/ModelHoldingPattern.cpp
)

## The main_muav executable
#add_executable(
#  main_muav_cm main_muav.cpp pi/PIController.cpp pi/ModelCatMouse.cpp
#)

add_executable(
  main_mugv main_mugv.cpp pi/PIController.cpp pi/ModelCatMouse.cpp
)

add_executable(
  main_ougv main_ougv.cpp pi/PIController.cpp pi/ModelCatMouse.cpp
)

## The init_muav executable
add_executable(
  init_suav init_suav.cpp
)

add_executable(
  init_mugv init_mugv.cpp
)

add_executable(
  init_ougv init_ougv.cpp
)


## The land_muav executable
#add_executable(
#  land_muav land_muav.cpp
#)

## Link against CRATES libraries
target_link_libraries(main_suav 
  ${catkin_LIBRARIES}
	${GSL_LIBRARIES} 
)

## Link against CRATES libraries
#target_link_libraries(main_muav_cm
#  ${catkin_LIBRARIES}
#	${GSL_LIBRARIES} 
#)

target_link_libraries(main_mugv
  ${catkin_LIBRARIES}
	${GSL_LIBRARIES} 
)

target_link_libraries(main_ougv
  ${catkin_LIBRARIES}
	${GSL_LIBRARIES} 
)


## Link against CRATES libraries
target_link_libraries(init_suav 
  ${catkin_LIBRARIES}
	${GSL_LIBRARIES} 
)

target_link_libraries(init_mugv 
  ${catkin_LIBRARIES}
	${GSL_LIBRARIES} 
)

target_link_libraries(init_ougv 
  ${catkin_LIBRARIES}
	${GSL_LIBRARIES} 
)

## Link against CRATES libraries
#target_link_libraries(land_muav 
#  ${catkin_LIBRARIES}
#)
