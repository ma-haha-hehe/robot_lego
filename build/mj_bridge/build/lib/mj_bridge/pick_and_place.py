#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node

import mujoco
import numpy as np

from geometry_msgs.msg import PoseStamped

# 如果你有 moveit_commander 的 Python 接口（或者自己的 wrapper），这里导入：
import moveit_commander


class PandaPickNode(Node):
    def __init__(self):
        super().__init__("panda_pick_node")

        # === 1. MoveIt 初始化 ===
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.hand_group = moveit_commander.MoveGroupCommander("hand")

        # 可选：设置速度比例
        self.arm_group.set_max_velocity_scaling_factor(0.2)
        self.arm_group.set_max_acceleration_scaling_factor(0.2)

        # === 2. 从 MuJoCo 读 brick1 位姿并生成 3 个抓取姿态 ===
        self.pre_grasp, self.grasp, self.lift = self.compute_poses_from_mujoco()

        self.get_logger().info(f"pre_grasp: {self.pre_grasp.pose}")
        self.get_logger().info(f"grasp: {self.grasp.pose}")
        self.get_logger().info(f"lift: {self.lift.pose}")

        # === 3. 执行完整的 pick 流程 ===
        self.pick()


    def compute_poses_from_mujoco(self):
        # 读取 scene.xml
        base_dir = os.path.dirname(os.path.abspath(__file__))
        xml_path = os.path.join(base_dir, "scene.xml")

        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        brick1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "brick1")
        brick1_pos = data.xpos[brick1_id].copy()
        self.get_logger().info(f"brick1_pos: {brick1_pos}")

        bx, by, bz = brick1_pos
        brick_top_z = bz + 0.01

        quat = [1.0, 0.0, 0.0, 0.0]  # 末端 z 轴朝下

        def make_pose(x, y, z):
            p = PoseStamped()
            p.header.frame_id = "panda_link0"
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.position.z = float(z)
            p.pose.orientation.x = quat[0]
            p.pose.orientation.y = quat[1]
            p.pose.orientation.z = quat[2]
            p.pose.orientation.w = quat[3]
            return p

        pre_grasp = make_pose(bx, by, brick_top_z + 0.10)
        grasp     = make_pose(bx, by, brick_top_z + 0.01)
        lift      = make_pose(bx, by, brick_top_z + 0.15)

        return pre_grasp, grasp, lift


    # === 辅助函数：执行规划+执行 ===
    def plan_and_execute_pose(self, pose: PoseStamped):
        self.arm_group.set_pose_target(pose)
        plan = self.arm_group.plan()
        if not plan or len(plan.joint_trajectory.points) == 0:
            self.get_logger().error("规划失败，plan 为空")
            return False
        self.arm_group.execute(plan, wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return True


    # === 手爪开合 ===
    def open_gripper(self):
        # 这里的关节名要和你的 Panda xacro/MoveIt 配置一致
        joint_goal = self.hand_group.get_current_joint_values()
        # 假设 hand 有两个对称关节
        joint_goal[0] = 0.04   # 打开到 4cm 左右
        joint_goal[1] = 0.04
        self.hand_group.go(joint_goal, wait=True)
        self.hand_group.stop()

    def close_gripper(self):
        joint_goal = self.hand_group.get_current_joint_values()
        joint_goal[0] = 0.0
        joint_goal[1] = 0.0
        self.hand_group.go(joint_goal, wait=True)
        self.hand_group.stop()


    # === 完整的 Pick 流程 ===
    def pick(self):
        self.get_logger().info("1) 打开手爪")
        self.open_gripper()

        self.get_logger().info("2) 规划到 pre-grasp")
        if not self.plan_and_execute_pose(self.pre_grasp):
            return

        self.get_logger().info("3) 规划到 grasp")
        if not self.plan_and_execute_pose(self.grasp):
            return

        self.get_logger().info("4) 闭合手爪")
        self.close_gripper()

        self.get_logger().info("5) 规划到 lift（把积木抬起来）")
        if not self.plan_and_execute_pose(self.lift):
            return

        self.get_logger().info("抓取流程完成！")
def main(args=None):
    rclpy.init(args=args)
    node = PandaPickNode()
    # 执行完 pick() 就可以退出；如果你想后续扩展，可以 spin 一下
    rclpy.shutdown()

if __name__ == "__main__":
    main()