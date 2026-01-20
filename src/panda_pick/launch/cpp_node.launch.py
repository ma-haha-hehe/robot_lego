import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 加载 MoveIt 基础配置
    moveit_config = MoveItConfigsBuilder("panda", package_name="moveit_resources_panda_moveit_config").to_moveit_configs()

    # 2. 手动寻找 ros2_controllers.yaml
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml"
    )

    # 3. 控制器配置
    moveit_controllers = {
        "moveit_simple_controller_manager": {
            "controller_names": ["panda_arm_controller", "panda_hand_controller"],
            "panda_arm_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": [
                    "panda_joint1", "panda_joint2", "panda_joint3", 
                    "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
                ]
            },
            "panda_hand_controller": {
                "type": "GripperCommand",
                "action_ns": "gripper_cmd",
                "default": True,
                "joints": ["panda_finger_joint1", "panda_finger_joint2"]
            }
        },
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # 4. 定义 MoveGroup
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # [🔥核心修复] 改为 False，因为我们没有跑 Gazebo，用的是系统时间
            {"use_sim_time": False}, 
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
            moveit_controllers, 
        ],
    )

    # 5. 定义你的 C++ 节点
    run_my_node = Node(
        package="panda_pick",
        executable="cpp_pick_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # [🔥核心修复] 改为 False
            {"use_sim_time": False},
        ],
    )

    # 6. 启动 ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )

    # 7. 孵化器 (Spawners)
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_hand = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        ros2_control_node,
        spawn_jsb,
        spawn_arm,
        spawn_hand,
        run_move_group_node,
        run_my_node
    ])