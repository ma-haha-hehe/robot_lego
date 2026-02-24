import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def generate_launch_description():
    # 1. 路径准备
    # 使用你之前提供的真实路径
    franka_xacro_file = '/home/i6user/Desktop/robot_lego/src/franka_ros2/franka_description/robots/panda_arm.urdf.xacro'
    franka_srdf_file = '/home/i6user/Desktop/robot_lego/src/franka_ros2/franka_moveit_config/srdf/panda_arm.srdf.xacro'

    # 2. 解析机器人描述 (URDF & SRDF)
    # 这里的 mappings 非常重要，连接真实机器人必须指定 IP
    robot_description_config = xacro.process_file(
        franka_xacro_file, 
        mappings={'robot_ip': '172.16.0.2', 'use_fake_hardware': 'false', 'hand': 'true'}
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    srdf_config = xacro.process_file(franka_srdf_file, mappings={'arm_id': 'panda'})
    robot_description_semantic = {'robot_description_semantic': srdf_config.toxml()}

    # 3. 加载 Kinematics (逆运动学解算器配置)
    # MoveIt 需要知道如何计算 6D 坐标到 7 个关节角的转换
    kinematics_yaml = os.path.join(get_package_share_directory("franka_moveit_config"), "config", "kinematics.yaml")

    # 4. 启动 MoveGroup 节点 (这是核心大脑)
    # 它负责接收你的 C++ 节点指令，并协调 RViz 和真实机械臂
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"publish_robot_description_semantic": True},
            {"use_sim_time": False},
        ],
    )

    # 5. 启动 RViz (可视化窗口)
    rviz_config_file = os.path.join(get_package_share_directory("franka_moveit_config"), "launch", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    # 6. 你的 C++ 业务节点 (设置 5 秒延迟，等待 MoveGroup 启动完毕)
    pick_node = Node(
        package='panda_pick',
        executable='cpp_pick_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {"use_sim_time": False}
        ],
        # 话题重映射：确保夹爪和关节状态与真机对齐
        remappings=[
            ('/panda_hand/gripper_action', '/panda_gripper/gripper_action'),
            ('/joint_states', '/franka/joint_states')
        ]
    )
    delayed_pick_node = TimerAction(period=5.0, actions=[pick_node])

    # 7. 静态坐标变换 (防止 RViz 中机器人飘走)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"]
    )

    return LaunchDescription([
        static_tf,
        move_group_node,
        rviz_node,
        delayed_pick_node
    ])
