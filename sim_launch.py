import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    # 1. 获取硬件类型参数
    hw_type = LaunchConfiguration("ros2_control_hardware_type").perform(context)

    # 2. 定义初始位姿 (避开自碰撞的关键姿态)
    initial_positions = {
        "initial_positions": {
            "panda_joint1": 0.0,
            "panda_joint2": -0.785,
            "panda_joint3": 0.0,
            "panda_joint4": -2.356,
            "panda_joint5": 0.0,
            "panda_joint6": 1.571,
            "panda_joint7": 0.785,
        }
    }

    # 3. 使用 MoveItConfigsBuilder 构建配置
    # 注意：这里会尝试加载 SRDF 并进行“手术”以禁用 link5/7 碰撞
    moveit_config_builder = MoveItConfigsBuilder("moveit_resources_panda")
    moveit_config_builder.robot_description(
        file_path="config/panda.urdf.xacro",
        mappings={"ros2_control_hardware_type": hw_type}
    )
    moveit_config_builder.robot_description_semantic(file_path="config/panda.srdf")
    moveit_config_builder.trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
    
    # 强制指定 OMPL 作为主规划器，避开 CHOMP
    moveit_config_builder.planning_pipelines(pipelines=["ompl"])
    
    moveit_config = moveit_config_builder.to_moveit_configs()

    # 🔥 [运行时手术] 修改 SRDF 字符串，强行禁用 link5 和 link7 碰撞检测
    srdf_content = moveit_config.robot_description_semantic["robot_description_semantic"]
    collision_bypass = '<disable_collisions link1="panda_link5" link2="panda_link7" reason="Adjacent"/>'
    if collision_bypass not in srdf_content:
        moveit_config.robot_description_semantic["robot_description_semantic"] = srdf_content.replace(
            '</robot>', f'    {collision_bypass}\n</robot>'
        )

    # 4. 定义节点
    
    # MoveGroup 核心
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
    )

    # 你的 C++ 节点 (增加 5 秒延迟，确保控制器 spawner 完成工作)
    pick_node = Node(
        package="panda_pick",
        executable="cpp_pick_node",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
        prefix='bash -c "sleep 5.0; $0 $@"'
    )

    # RViz (使用完整配置)
    rviz_base = os.path.join(get_package_share_directory("moveit_resources_panda_moveit_config"), "launch")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[moveit_config.to_dict()],
        condition=UnlessCondition(LaunchConfiguration("rviz_tutorial")),
    )

    # Static TF & State Publisher
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control 节点 (注入初始位姿)
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config", "ros2_controllers.yaml"
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path, initial_positions],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    # Spawners (控制器加载器)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    return [
        static_tf_node,
        robot_state_publisher,
        move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        rviz_node,
        pick_node
    ]

def generate_launch_description():
    # 声明参数
    return LaunchDescription([
        DeclareLaunchArgument("rviz_tutorial", default_value="False"),
        DeclareLaunchArgument("ros2_control_hardware_type", default_value="mock_components"),
        OpaqueFunction(function=launch_setup)
    ])