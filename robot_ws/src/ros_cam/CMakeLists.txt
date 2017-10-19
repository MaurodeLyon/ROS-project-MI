cmake_minimum_required(VERSION 2.8.3)
project(ros_cam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs 
  sensor_msgs 
  cv_bridge 
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(subscriber1 src/subscriber1.cpp)
target_link_libraries(subscriber1 ${catkin_LIBRARIES} ${OpenCV_LIBS})
