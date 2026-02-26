cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找核心工具包
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# 2. 声明服务文件并添加依赖
# 这一步会自动生成 C++ (.hpp) 和 Python (.py) 接口代码
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetBlockPose.srv"
  DEPENDENCIES geometry_msgs
)

# 3. 导出包信息
ament_package()