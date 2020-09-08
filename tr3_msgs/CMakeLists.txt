cmake_minimum_required(VERSION 2.8.3)
project(tr3_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  ActuatorState.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ForwardIK.srv
  InverseIK.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
)
