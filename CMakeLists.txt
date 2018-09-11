cmake_minimum_required(VERSION 2.8.3)
project(rll_planning_project)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_srvs
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rll_msgs
  rll_move
  genmsg actionlib_msgs actionlib
)

add_service_files(
  FILES
  Move.srv
  CheckPath.srv
)

add_action_files(DIRECTORY action FILES PlanToGoal.action)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime rll_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(planning_iface src/planning_iface.cpp)
target_link_libraries(planning_iface ${catkin_LIBRARIES})
add_dependencies(planning_iface ${PROJECT_NAME}_generate_messages_cpp)
