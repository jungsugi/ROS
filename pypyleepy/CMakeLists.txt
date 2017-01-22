cmake_minimum_required(VERSION 2.8.3)
project(pypyleepy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(DIRECTORY srv
    FILES 
    srvChangeMode.srv
    srvRC.srv  
    srvPaint.srv
)
 
generate_messages(
   DEPENDENCIES
   std_msgs
   
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES team4
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


