cmake_minimum_required(VERSION 2.8.3)
project(rll_planning_project)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  message_generation
  rll_move
  rll_move_client
  rll_msgs
  rostest
  std_msgs
)

add_service_files(
  FILES
  CheckPath.srv
  GetStartGoal.srv
  Move.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime rll_move_client rll_msgs std_msgs
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(planning_iface src/planning_iface.cpp src/planning_iface_node.cpp)
target_link_libraries(planning_iface ${catkin_LIBRARIES})
add_dependencies(planning_iface ${PROJECT_NAME}_generate_messages_cpp)

catkin_install_python(PROGRAMS scripts/path_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS planning_iface
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch meshes urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_rostest(tests/launch/tests_python.test)
endif()
