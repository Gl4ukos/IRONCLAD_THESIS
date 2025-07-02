cmake_minimum_required(VERSION 3.0.2)
project(ackermann_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  gazebo_ros
  gazebo_msgs
  urdf
  xacro
)

catkin_package(
  CATKIN_DEPENDS
    roscpp rospy std_msgs sensor_msgs
    gazebo_ros gazebo_msgs geometry_msgs urdf xacro
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# === Add Executables ===

add_executable( teleop
  src/teleop.cpp
  src/command_publishers.cpp
)

add_executable( target_spawner
  src/target_spawner.cpp
)

# === Link Libraries ===

target_link_libraries(teleop
  ${catkin_LIBRARIES}
)

target_link_libraries(target_spawner
  ${catkin_LIBRARIES}
)

# === Install Targets ===

install(TARGETS
  teleop
  target_spawner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# === Install URDF Files ===

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

