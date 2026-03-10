import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    hw_type = LaunchConfiguration("ros2_control_hardware_type").perform(context)

    # 1. 定义初始位姿 (Ready姿态)
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

    # 2. 核心：强制修正参数类型 Bug (Double vs String)
    # 我们将这些参数手动写死为 float 类型，防止被误判为 string
    joint_limit_params = {
        "robot_description_planning": {
            "joint_limits": {
                "panda_joint1": {"max_velocity": 2.175, "max_acceleration": 3.75},
                "panda_joint2": {"max_velocity": 2.175, "max_acceleration": 1.875},
                "panda_joint3": {"max_velocity": 2.175, "max_acceleration": 2.5},
                "panda_joint4": {"max_velocity": 2.175, "max_acceleration": 3.125},
                "panda_joint5": {"max_velocity": 2.610, "max_acceleration": 3.75},
                "panda_joint6": {"max_velocity": 2.610, "max_acceleration": 5.0},
                "panda_joint7": {"max_velocity": 2.610, "max_acceleration": 5.0},
            }
        }
    }

    # 3. 构建 MoveIt 配置
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro", mappings={"ros2_control_hardware_type": hw_type})
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 4. 定义节点

    # 机器人状态发布器 (显示机器人的关键)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": False}],
    )

    # MoveGroup 核心
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            joint_limit_params, # 强制覆盖错误参数
            {"use_sim_time": False}
        ],
    )

    # RViz2 (修复模型显示)
    rviz_base = os.path.join(get_package_share_directory("moveit_resources_panda_moveit_config"), "launch")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(rviz_base, "moveit.rviz")],
        parameters=[
            moveit_config.to_dict(),
            joint_limit_params, # 强制覆盖错误参数
            {"use_sim_time": False}
        ],
    )

    # 静态变换 (修复 1970 时间戳 Bug)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        # 使用新式参数格式
        arguments=["--x", "0", "--y", "0", "--z", "0", 
                   "--roll", "0", "--pitch", "0", "--yaw", "0", 
                   "--frame-id", "world", "--child-frame-id", "panda_link0"],
    )

    # ros2_control 节点
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

    # 1. 关节状态发布器 (标准配置)
    js_broadcaster = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    # 2. 机械臂控制器 (之前你已经有了)
    arm_controller = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["panda_arm_controller", "--controller-manager", "/controller_manager"],
    )

    # 3. 🔥 核心修正：单独为夹爪创建一个加载节点
    # 这将启动 panda_hand_controller，解决 Action Server 连不上的报错
    hand_controller = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["panda_hand_controller", "--controller-manager", "/controller_manager"],
    )

    # 4. 用户 C++ 节点 (建议把 joint_limit_params 也传给它)
    pick_node = Node(
        package="panda_pick",
        executable="cpp_pick_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(), 
            joint_limit_params,  # 💡 传入修正参数，防止 IK 报错
            {"use_sim_time": False}
        ],
        prefix='bash -c "sleep 8.0; $0 $@"'
    )

    return [
        static_tf_node,
        robot_state_publisher,
        move_group_node,
        ros2_control_node,
        js_broadcaster,
        arm_controller,
        hand_controller, # 👈 确保这里添加了 hand_controller
        rviz_node,
        pick_node
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ros2_control_hardware_type", default_value="mock_components"),
        OpaqueFunction(function=launch_setup)
    ])