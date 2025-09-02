cmake_minimum_required(VERSION 3.10)
project(robotiq_2f_gripper_action_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  actionlib_msgs
  control_msgs
  robotiq_2f_gripper_control
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp actionlib actionlib_msgs control_msgs robotiq_2f_gripper_control
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robotiq_2f_gripper_action_server_node
  src/robotiq_2f_gripper_action_server_node.cpp
  src/robotiq_2f_gripper_action_server.cpp
  include/robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server.h
)

add_dependencies(robotiq_2f_gripper_action_server_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(robotiq_2f_gripper_action_server_node
  ${catkin_LIBRARIES}
)

add_executable(robotiq_2f_gripper_action_client_node
  src/robotiq_2f_gripper_action_client_node.cpp
)

add_dependencies(robotiq_2f_gripper_action_client_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(robotiq_2f_gripper_action_client_node
  ${catkin_LIBRARIES}
)

install(TARGETS
  robotiq_2f_gripper_action_server_node
  robotiq_2f_gripper_action_client_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
