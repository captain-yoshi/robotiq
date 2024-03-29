cmake_minimum_required(VERSION 3.0.2)
project(robotiq_ft_sensor)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
    ft_sensor.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    sensor_accessor.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs roscpp
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

# make the executables
add_executable(rq_sensor nodes/rq_sensor.cpp src/rq_sensor_com.cpp src/rq_sensor_state.cpp src/low_pass_filter.cpp)
add_dependencies(rq_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rq_sensor ${catkin_LIBRARIES})

add_executable(rq_test_sensor nodes/rq_test_sensor.cpp)
add_dependencies(rq_test_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rq_test_sensor ${catkin_LIBRARIES})

add_executable(rq_tare_wrench nodes/rq_tare_wrench.cpp)
add_dependencies(rq_tare_wrench ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rq_tare_wrench ${catkin_LIBRARIES})

install(TARGETS
  rq_sensor
  rq_test_sensor
  rq_tare_wrench
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
