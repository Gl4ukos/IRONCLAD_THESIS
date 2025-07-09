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
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    roscpp rospy std_msgs sensor_msgs
    gazebo_ros gazebo_msgs geometry_msgs urdf xacro
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# === Add Executables ===

add_library( command_publishers
  src/command_publishers.cpp
)

add_executable( teleop
  src/teleop.cpp
  src/command_publishers.cpp
)

add_executable( target_spawner
  src/target_spawner.cpp
)

# === Link Libraries ===

target_include_directories(command_publishers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries( command_publishers
  ${catkin_LIBRARIES}
)

target_link_libraries(teleop
  ${catkin_LIBRARIES}
)

target_link_libraries(target_spawner
  ${catkin_LIBRARIES}
)

# === Install Targets ===
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES command_publishers
  CATKIN_DEPENDS
    roscpp rospy std_msgs sensor_msgs
    gazebo_ros gazebo_msgs geometry_msgs urdf xacro
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


install(TARGETS
  teleop
  target_spawner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# === Install URDF Files ===

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

