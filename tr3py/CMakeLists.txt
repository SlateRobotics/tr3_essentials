cmake_minimum_required(VERSION 2.8.3)
project(tr3py)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tr3_msgs
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS
  tr3_msgs
)

install(PROGRAMS src/tr3/tr3.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
