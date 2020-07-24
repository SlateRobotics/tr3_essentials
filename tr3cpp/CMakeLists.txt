cmake_minimum_required(VERSION 2.8.3)
project(tr3cpp)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    roscpp
  LIBRARIES
		tr3cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(tr3cpp
	include/${PROJECT_NAME}/tr3.h
	include/${PROJECT_NAME}/joint.h
	include/${PROJECT_NAME}/msgs.h
	include/${PROJECT_NAME}/Protocol.h
	include/${PROJECT_NAME}/ProtocolSimple.h
	include/${PROJECT_NAME}/Socket.h
	include/${PROJECT_NAME}/Utility.h
	include/${PROJECT_NAME}/Socket.tpp
	src/${PROJECT_NAME}/tr3.cpp
	src/${PROJECT_NAME}/joint.cpp
	src/Protocol.cpp
	src/ProtocolSimple.cpp
	src/Socket.cpp
)
target_link_libraries(tr3cpp ${catkin_LIBRARIES})
