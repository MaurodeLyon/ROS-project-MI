cmake_minimum_required(VERSION 2.8.3)
project(core)
find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(core src/core.cpp)
target_link_libraries(core ${catkin_LIBRARIES})
