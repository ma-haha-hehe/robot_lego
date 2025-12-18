cmake_minimum_required(VERSION 3.8)
project(panda_pick)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(pick_and_place_node src/pick_and_place.cpp)

ament_target_dependencies(pick_and_place_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

install(TARGETS
  pick_and_place_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()