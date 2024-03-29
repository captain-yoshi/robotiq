# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(robotiq_2f_gripper_control)

#add_definitions (-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation )

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")
#find_package(Modbus REQUIRED)

set(MODBUS_INCLUDE_DIR /usr/include/modbus)
set(MODBUS_LIBRARIES /usr/lib/x86_64-linux-gnu/libmodbus.so)


#set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



###################################
## catkin message generation     ##
###################################
add_message_files(
  FILES
  Robotiq2FGripper_robot_input.msg
  Robotiq2FGripper_robot_output.msg
)

catkin_python_setup()

generate_messages()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    LIBRARIES
      ${PROJECT_NAME}
    INCLUDE_DIRS
      include
    CATKIN_DEPENDS
      rospy
      message_runtime
      roscpp
    #DEPENDS modbus
)

add_library(${PROJECT_NAME}
  src/conversion.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MODBUS_INCLUDE_DIR}
)

add_executable(robotiq_2f_gripper_serial_node
  src/robotiq_2f_gripper_control/robotiq_2f_gripper_serial_node.cpp
  src/robotiq_2f_gripper_control/robotiq_2f_gripper_serial_client.cpp
)

target_link_libraries(robotiq_2f_gripper_serial_node
   ${catkin_LIBRARIES}
   ${MODBUS_LIBRARIES}  
)

add_dependencies(robotiq_2f_gripper_serial_node robotiq_2f_gripper_control_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS nodes/Robotiq2FGripperSimpleController.py nodes/Robotiq2FGripperStatusListener.py
    nodes/Robotiq2FGripperTcpNode.py nodes/Robotiq2FGripperRtuNode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

