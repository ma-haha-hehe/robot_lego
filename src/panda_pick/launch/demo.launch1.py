import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 声明参数 (虽然下面写死了，但保留这个声明是个好习惯，防止其他地方用到)
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type", default_value="mock_components",
    )

    # 2. 构建 MoveIt 配置
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            # 🔥 [修复] 这里直接用字符串 "mock_components"，避开 LaunchConfiguration 读取时序问题
            mappings={"ros2_control_hardware_type": "mock_components"},
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml") 
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # 3. 显式注入控制器命名空间 (防止 "Returned 0 controllers" 错误)
    moveit_controllers_overrides = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": ["panda_arm_controller", "panda_hand_controller"],
            "panda_arm_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": [ # 🔥 必须加上这个列表！
                    "panda_joint1",
                    "panda_joint2",
                    "panda_joint3",
                    "panda_joint4",
                    "panda_joint5",
                    "panda_joint6",
                    "panda_joint7"
                ]
            },
            "panda_hand_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": [ # 🔥 还有这个！
                    "panda_finger_joint1",
                    "panda_finger_joint2"
                ]
            }
        }
    }

    # 4. 启动 move_group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"robot_description_kinematics.panda_arm.kinematics_solver_timeout": 0.05},
            moveit_controllers_overrides,
            {"moveit_manage_controllers": True},
        ],
    )

    # 5. 获取路径
    panda_pick_dir = get_package_share_directory("panda_pick")
    ros2_controllers_path = os.path.join(panda_pick_dir, "config", "ros2_controllers.yaml")

    # 6. 强制开启 Open Loop Control (防止仿真抖动)
    ros2_control_overrides = {
        "panda_arm_controller": {
            "ros__parameters": {
                "open_loop_control": True,
                "allow_nonzero_velocity_at_trajectory_end": True,
            }
        },
        "panda_hand_controller": {
            "ros__parameters": {
                "open_loop_control": True,
                "allow_partial_joints_goal": True,
            }
        }
    }

    # 7. 启动 ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            ros2_control_overrides 
        ],
        output="screen",
    )

    # 8. Spawners
    spawn_jsb = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    spawn_arm = Node(package="controller_manager", executable="spawner", arguments=["panda_arm_controller"])
    spawn_hand = Node(package="controller_manager", executable="spawner", arguments=["panda_hand_controller"])
    
    # 9. MuJoCo Bridge
    bridge = Node(
        package="mj_bridge", 
        executable="mj_bridge", 
        output="screen"
    )

    # 10. RViz & RSP
    rviz_base = os.path.join(get_package_share_directory("moveit_resources_panda_moveit_config"), "launch")
    rviz_node = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", os.path.join(rviz_base, "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            {"robot_description_kinematics.panda_arm.kinematics_solver_timeout": 0.05},
            moveit_config.robot_description_kinematics,
        ]
    )
    static_tf = Node(package="tf2_ros", executable="static_transform_publisher", arguments=["0","0","0","0","0","0","world","panda_link0"])
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[moveit_config.robot_description])
    
    # 11. 控制节点 (延迟 8 秒)
    pick_node = Node(
        package="panda_pick",
        executable="cpp_pick_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    delayed_pick_node = TimerAction(period=8.0, actions=[pick_node])

    return LaunchDescription([
        static_tf, rsp, ros2_control_node, 
        spawn_jsb, spawn_arm, spawn_hand,
        move_group_node, rviz_node, bridge,
        delayed_pick_node
    ])