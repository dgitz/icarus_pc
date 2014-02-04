cmake_minimum_required(VERSION 2.8.3)
project(icarus_pc)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge image_transport)
find_package (OpenCV REQUIRED)

# CONFIGURE OPENNI2
find_library(OPENNI2_LIBRARY
	NAMES OpenNI2
	PATHS "~/OpenNI-Linux-Arm-2.2/Redist" "~/OpenNI-Linux-x64-2.2/Redist"
)
find_path(OPENNI2_INCLUDE_DIR OpenNI.h
	PATHS "~/OpenNI-Linux-Arm-2.2/Include" "~/OpenNI-Linux-x64-2.2/Include"
)

#link_directories(${OPENNI2_LIBRARY})
#include_directories(${OPENNI2_INCLUDE_DIR})

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${OPENNI2_INCLUDE_DIR})

add_executable(xtionsender nodes/xtionsender.cpp)
target_link_libraries(xtionsender ${catkin_LIBRARIES} ${OPENNI2_LIBRARY} ${OpenCV_LIBS})
add_dependencies(xtionsender icarus_pc_generate_messages_cpp)

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)
