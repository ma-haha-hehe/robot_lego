#!/usr/bin/env python3
import os
import time  # 导入 time 模块用于 sleep
import threading  # 导入 threading 模块用于线程锁

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from control_msgs.action import FollowJointTrajectory  # 导入 Action 类型

import mujoco
import mujoco_viewer

# Panda 的 7 个主关节名字（和 MoveIt 中保持一致）
# 我们将使用这个列表作为“唯一”的关节顺序标准
JOINTS = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]


class MjBridgeNode(Node):
    def __init__(self):
        super().__init__("mj_bridge_node")

        # 1. MuJoCo 模型路径
        xml_path = "/home/aaa/robot/ros2_ws/src/mj_bridge/mj_bridge/scene.xml"
        self.get_logger().info(f"[mj_bridge] Loading MuJoCo model from: {xml_path}")

        # 2. 加载 MuJoCo 模型
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

        # 3. 预计算：每个关节在 qpos 里的索引 (与之前相同)
        self.qpos_id = []
        for name in JOINTS:
            j = self.model.joint(name)
            idx = j.qposadr[0]
            self.qpos_id.append(idx)
            self.get_logger().info(f"[mj_bridge] Joint {name} -> qpos index {idx}")

        # 4. 【新】线程锁：
        # 因为 Action 回调和 Timer 回调会在不同的线程中同时运行
        # 它们会同时访问 self.joint_pos_target，所以需要一个“锁”来保证数据安全
        self._lock = threading.Lock()

        # 5. 【新】目标关节位置
        # 这是 Action Server（控制线程）写入的 "目标"
        # 也是 Timer（仿真线程）读取的 "目标"
        # 它取代了原来从 /joint_states 订阅的功能
        self.joint_pos_target = {name: 0.0 for name in JOINTS}

        # 6. 【新】创建 Action Server
        #    MoveIt (Rviz) 将会把它的 "Execute" 目标 (Goal) 发送到这里
        action_name = "/panda_arm_controller/follow_joint_trajectory"
        self.get_logger().info(f"Starting FollowJointTrajectory Action Server on '{action_name}'")
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self.execute_trajectory_callback,
            goal_callback=self.goal_callback
        )
        
        # 7. 【修改】定时器：现在只负责“仿真”和“渲染”
        # 它的职责是从 self.joint_pos_target 读取目标并应用到 MuJoCo
        self.timer = self.create_timer(0.01, self.update_mujoco)

        self.get_logger().info("[mj_bridge] MjBridgeNode initialized as Action Server.")

    # 【新】Goal 回调：检查是否接受新的轨迹目标
    def goal_callback(self, goal_request):
        self.get_logger().info("Received new trajectory goal request.")
        # 简单起见，我们总是接受
        return GoalResponse.ACCEPT

    # 【新】Execute 回调：这是 MoveIt 点击 "Execute" 时运行的核心
    # (这个函数由 ActionServer 在一个单独的线程中运行)
    def execute_trajectory_callback(self, goal_handle):
        self.get_logger().info("Executing trajectory goal...")
        
        trajectory = goal_handle.request.trajectory
        
         # === 打印 MoveIt 发送来的轨迹结构 ===
        self.get_logger().info(f"Trajectory joint_names: {trajectory.joint_names}")
        self.get_logger().info(f"Trajectory has {len(trajectory.points)} points.")

        for i, point in enumerate(trajectory.points):
            t = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            pos = list(point.positions)
            vel = list(point.velocities) if point.velocities else []
            acc = list(point.accelerations) if point.accelerations else []
            self.get_logger().info(
                f"  Point {i}: t={t:.3f}s\n"
                f"    positions={pos}\n"
                f"    velocities={vel}\n"
                f"    accelerations={acc}"
            )

        # --- 安全检查 (非常重要) ---
        # 检查 MoveIt 发来的轨迹里的关节名是否和我们模型里的匹配
        if trajectory.joint_names != JOINTS:
            self.get_logger().error(
                "Trajectory joint names do not match expected names!\n"
                f"Expected: {JOINTS}\n"
                f"Got: {trajectory.joint_names}"
            )
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        last_point_time = 0.0

        # 循环遍历轨迹中的每一个“路点”
        for point in trajectory.points:
            # 1. 检查是否中途被取消
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled.")
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            # 2. 计算需要“睡”多久才到下一个点
            target_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            sleep_duration = target_time - last_point_time
            
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            
            last_point_time = target_time

            # 3. 【核心】更新目标字典
            #    我们把新路点的角度，写入到那个“共享”的目标字典中
            with self._lock:
                for i, name in enumerate(JOINTS):
                    self.joint_pos_target[name] = point.positions[i]
            
            # (我们不在这里渲染，渲染由 update_mujoco 定时器负责)

        # 4. 轨迹执行完毕
        goal_handle.succeed()
        self.get_logger().info("Trajectory execution finished successfully.")
        
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    # 【修改】定时器回调：现在是“仿真/渲染”循环
    def update_mujoco(self):
        # 1. 把“目标字典”里的角度应用到 MuJoCo 的 qpos
        #    (加锁，确保我们拿到的是一个完整的目标，而不是正在被 Action 线程修改的)
        with self._lock:
            for i, name in enumerate(JOINTS):
                self.data.qpos[self.qpos_id[i]] = self.joint_pos_target[name]

        # 2. 不走动力学积分，只做前向运动学计算
        mujoco.mj_forward(self.model, self.data)

        # 3. 渲染到窗口
        try:
            self.viewer.render()
        except Exception as e:
            self.get_logger().warn(f"Failed to render viewer: {e}")
            # (这里可以添加逻辑，如果窗口关闭了就退出)


def main(args=None):
    rclpy.init(args=args)
    node = MjBridgeNode()
    
    # 使用 MultiThreadedExecutor 来确保 Action 回调和 Timer 可以在不同线程运行
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()