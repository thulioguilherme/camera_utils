cmake_minimum_required(VERSION 3.1.2)
project(camera_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  camera_calibration_parsers
)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wpedantic)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES InfoPublisher
  CATKIN_DEPENDS roscpp sensor_msgs camera_calibration_parsers
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(InfoPublisher src/info_publisher.cpp)

add_dependencies(InfoPublisher
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(InfoPublisher
  ${catkin_LIBRARIES}
)
