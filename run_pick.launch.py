import os
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. 手动定义路径 (根据你之前的日志)
    robot_xacro_filepath = '/home/i6user/Desktop/robot_lego/src/franka_ros2/franka_description/robots/panda_arm.urdf.xacro'
    srdf_filepath = '/home/i6user/Desktop/robot_lego/src/franka_ros2/franka_moveit_config/srdf/panda_arm.srdf.xacro'

    # 2. 解析 URDF (Xacro)
    robot_description_config = xacro.process_file(
        robot_xacro_filepath, 
        mappings={'robot_ip': '172.16.0.2', 'use_fake_hardware': 'false', 'hand': 'true'}
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 3. 解析 SRDF (Xacro)
    srdf_config = xacro.process_file(srdf_filepath, mappings={'arm_id': 'panda'})
    robot_description_semantic = {'robot_description_semantic': srdf_config.toxml()}

    return LaunchDescription([
        Node(
            package='panda_pick',
            executable='cpp_pick_node',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                {'use_sim_time': False}
            ],
        )
    ])