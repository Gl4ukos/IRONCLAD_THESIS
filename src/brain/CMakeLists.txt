cmake_minimum_required(VERSION 3.0.2)
project(brain)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(commander src/commander.cpp)
 target_link_libraries(commander ${catkin_LIBRARIES})

