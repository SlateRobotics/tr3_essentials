cmake_minimum_required(VERSION 3.1.3)
project(tr3_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tr3_msgs
  moveit_ros_planning_interface
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    tr3_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
    moveit_ros_planning_interface 
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(forward_ik_server src/forward_ik_server.cpp)
target_link_libraries(forward_ik_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(inverse_ik_server src/inverse_ik_server.cpp)
target_link_libraries(inverse_ik_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS forward_ik_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS inverse_ik_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
