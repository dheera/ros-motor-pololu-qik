cmake_minimum_required(VERSION 2.8.3)
project(motor_pololu_qik)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES qik_driver
#  CATKIN_DEPENDS roscpp trajectory_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pololu_qik_node src/pololu_qik_node.cpp src/PololuQikActivity.cpp)

target_link_libraries(pololu_qik_node
  ${catkin_LIBRARIES}
)

install(TARGETS pololu_qik_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

