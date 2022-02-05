cmake_minimum_required(VERSION 3.0.2)
project(moveit)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  moveit_commander
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  shape_msgs
  tf
  trajectory_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS 
#   system
#   filesystem
#   date_time
#   thread
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moveit
  CATKIN_DEPENDS 
    actionlib control_msgs geometry_msgs moveit_commander moveit_core moveit_msgs moveit_ros_planning_interface moveit_visual_tools roscpp shape_msgs tf trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pick_and_placer src/pick_and_placer.cpp)
target_link_libraries(pick_and_placer
  ${catkin_LIBRARIES}
)

# add_executable(arm_move src/arm_move.cpp)
# target_link_libraries(arm_move
#   ${catkin_LIBRARIES}
#   # arm_move
# )

add_executable(arm_move_main src/arm_move_main.cpp src/arm_move.cpp)
target_link_libraries(arm_move_main
  ${catkin_LIBRARIES}
)

add_executable(arm_move_main_85 src/arm_move_main_85.cpp src/arm_move_85.cpp)
target_link_libraries(arm_move_main_85
  ${catkin_LIBRARIES}
)