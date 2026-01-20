cmake_minimum_required(VERSION 3.8)
project(panda_pick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# 2. 定义可执行文件
add_executable(cpp_pick_node src/cpp_pick_node.cpp)

# 3. 链接库和头文件
ament_target_dependencies(cpp_pick_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  tf2_geometry_msgs
  yaml-cpp
)

target_link_libraries(cpp_pick_node yaml-cpp)

# 4. 安装规则 (Install Rules)
# 安装可执行文件
install(TARGETS
  cpp_pick_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Launch 文件 (非常重要，否则 ros2 launch 找不到)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Config 文件 (控制器配置)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()