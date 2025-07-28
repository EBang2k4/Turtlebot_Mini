cmake_minimum_required(VERSION 3.0.2)
project(teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_sdk_examples
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs dynamixel_sdk_examples
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_node src/teleop_node.cpp)
target_link_libraries(teleop_node ${catkin_LIBRARIES})
install(TARGETS teleop_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

