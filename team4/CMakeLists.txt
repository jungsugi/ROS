cmake_minimum_required(VERSION 2.8.3)
project(team4)

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


add_executable(pub_team4_4 src/pub_team4_4.cpp)
target_link_libraries(pub_team4_4 ${catkin_LIBRARIES})
add_dependencies(pub_team4_4 team4_generate_messages_cpp)




add_executable(middle src/middle.cpp)
target_link_libraries(middle ${catkin_LIBRARIES})
add_dependencies(middle team4_generate_messages_cpp)
