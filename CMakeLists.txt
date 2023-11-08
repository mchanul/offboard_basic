cmake_minimum_required(VERSION 3.0.2)
project(offboard_basic)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(offboard_basic src/offboard_basic.cpp)

target_link_libraries(offboard_basic ${catkin_LIBRARIES})

