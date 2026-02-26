cmake_minimum_required(VERSION 3.8)
project(my_robot_vision)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# 安装 Python 脚本到 lib 下
install(PROGRAMS
  my_robot_vision/vision_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件到 share 下
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
# 2. 确保 Python 节点也被安装
install(PROGRAMS
  my_robot_vision/vision_node.py
  my_robot_vision/foundation_wrapper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()