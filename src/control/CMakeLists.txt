cmake_minimum_required(VERSION 3.0.2)
project(control)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

add_library( control_lib
  src/pure_pursuit.cpp
  src/lateral.cpp
)

target_include_directories(control_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries( control_lib
  ${catkin_LIBRARIES}
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES control_lib
  CATKIN_DEPENDS roscpp std_msgs 
)

install(TARGETS control_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
