cmake_minimum_required(VERSION 2.8.3)
project(joy_handler)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_handler src/joy_handler.cpp)
target_link_libraries(joy_handler  ${catkin_LIBRARIES})
