cmake_minimum_required(VERSION 2.8.3)
project(iiwa_test)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  iiwa_ros
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(test_publisher src/test_publisher.cpp)
add_dependencies(test_publisher iiwa_msgs_generate_messages_cpp)
target_link_libraries(test_publisher ${catkin_LIBRARIES})
