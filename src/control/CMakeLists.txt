cmake_minimum_required(VERSION 3.0.2)
project(control)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)

add_library( control_lib
  src/pure_pursuit.cpp
  src/lateral.cpp
  src/mpc.cpp
)

target_include_directories(control_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries( control_lib
  ${catkin_LIBRARIES}
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES control_lib
  CATKIN_DEPENDS roscpp std_msgs 
  DEPENDS EIGEN3
)

install(TARGETS control_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
