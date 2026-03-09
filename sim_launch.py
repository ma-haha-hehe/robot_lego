import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # 1. 路径定义 (请确保路径准确)
    robot_xacro_filepath = '/home/i6user/Desktop/robot_lego/src/franka_ros2/franka_description/robots/panda_arm.urdf.xacro'
    srdf_filepath = '/home/i6user/Desktop/robot_lego/src/franka_ros2/franka_moveit_config/srdf/panda_arm.srdf.xacro'
    
    # 2. 解析 URDF 和 SRDF
    # 注意：既然是虚拟机械臂，use_fake_hardware 设为 true
    robot_description_config = xacro.process_file(
        robot_xacro_filepath, 
        mappings={'use_fake_hardware': 'true', 'hand': 'true'}
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    srdf_config = xacro.process_file(srdf_filepath, mappings={'arm_id': 'panda'})
    robot_description_semantic = {'robot_description_semantic': srdf_config.toxml()}

    # 3. 加载运动学插件配置 (IK 求解器)
    kinematics_yaml = '/home/i6user/Desktop/robot_lego/src/franka_ros2/franka_moveit_config/config/kinematics.yaml'

    # 4. 启动 MoveGroup (机械臂的大脑)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': False, 'publish_robot_description_semantic': True}
        ],
    )

    # 5. 启动 Robot State Publisher (发布 TF 树)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': False}]
    )

    # 6. 启动 RViz2 (可视化窗口)
    rviz_config_file = os.path.join(get_package_share_directory('moveit_resources_panda_moveit_config'), 'launch', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description, robot_description_semantic],
        output='screen'
    )

    # 7. 你的 C++ 执行节点 (负责接收 YAML 并生成场景物体)
    pick_node = Node(
        package='panda_pick',
        executable='cpp_pick_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'use_sim_time': False}
        ],
        remappings=[
            ('/joint_states', '/joint_states') # 仿真模式下通常直接使用默认话题
        ]
    )

    # 8. 静态变换 (world -> panda_link0)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'panda_link0']
    )

    return LaunchDescription([
        static_tf,
        rsp_node,
        move_group_node,
        rviz_node,
        pick_node
    ])