cmake_minimum_required(VERSION 2.8.3)
project(tr3py)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS src/tr3/tr3.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
