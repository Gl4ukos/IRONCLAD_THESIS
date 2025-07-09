cmake_minimum_required(VERSION 3.0.2)
project(brain)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  control
  ackermann_simulation
)

catkin_package(
  CATKIN_DEPENDS
    roscpp rospy std_msgs ackermann_simulation control
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(commander src/commander.cpp )

 target_link_libraries(commander 
  ${catkin_LIBRARIES}
  control_lib
  command_publishers  
) 

