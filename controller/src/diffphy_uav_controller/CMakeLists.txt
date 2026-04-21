cmake_minimum_required(VERSION 3.0.2)
project(diffphy_uav_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs std_msgs tf2 tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(attitude_mapper
  src/attitude_mapper.cpp
)

target_link_libraries(attitude_mapper
  ${catkin_LIBRARIES}
)

add_executable(offboard_controller_node
  src/offboard_controller_node.cpp
)

target_link_libraries(offboard_controller_node
  attitude_mapper
  ${catkin_LIBRARIES}
)

install(TARGETS attitude_mapper offboard_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
