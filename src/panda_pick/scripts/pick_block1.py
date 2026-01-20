#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import time
import math

# ROS 消息
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint
from moveit_msgs.action import MoveGroup
from control_msgs.action import GripperCommand

# ==========================================
# 🎯 目标坐标配置
# ==========================================
PREDICTED_X = 0.42
PREDICTED_Y = -0.12

# 积木几何中心 (作为参考)
CENTER_Z = 0.465 

# 🔥 关键修改：抓取高度抬高 2.5cm，只抓上半部分 🔥
# 0.465 (中心) + 0.025 = 0.490
GRASP_Z = CENTER_Z + 0.025 

# 手长补偿 (10.34cm)
GRIPPER_LENGTH_OFFSET = 0.1034 
# ==========================================

def clear_screen():
    print("\033c", end="")

class PandaCNNPicker(Node):
    def __init__(self):
        super().__init__('panda_cnn_picker')
        self.move_group = ActionClient(self, MoveGroup, 'move_action')
        self.gripper = ActionClient(self, GripperCommand, 'panda_hand_controller/gripper_cmd')
        
        print("Waiting for MoveIt server...")
        self.move_group.wait_for_server()
        self.gripper.wait_for_server()
        print("✅ Ready!")

    def control_gripper(self, width):
        print(f"   🖐 Gripper moving to: {width:.3f}")
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 500.0 
        self.gripper.send_goal_async(goal)
        time.sleep(1.0)

    # 复位函数 (使用关节角度，最稳)
    def go_to_home(self):
        print("\n🏠 Going to HOME position (Reset)...")
        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_arm"
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5 
        goal.request.max_acceleration_scaling_factor = 0.5
        
        # Panda Ready Pose
        joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
        joint_values = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        constraints = Constraints()
        for name, val in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name; jc.position = val
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal.request.goal_constraints.append(constraints)
        
        future = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if not res or not res.accepted:
            print("❌ Failed to go Home.")
            return False
            
        res_future = res.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        print("✅ Robot is at HOME.")
        return True

    def move_to_target(self, x, y, z_tcp, desc="Moving", strict_mode=False):
        wrist_z = z_tcp + GRIPPER_LENGTH_OFFSET

        print(f"\n📍 {desc} {'[🔥直线]' if strict_mode else '[普通]'}")
        print(f"   Target: [{x:.3f}, {y:.3f}, {z_tcp:.3f}]")

        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_arm"
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        
        # 1. 位置约束
        pc = PositionConstraint()
        pc.header.frame_id = "world"; pc.link_name = "panda_hand"
        bv = BoundingVolume(); box = SolidPrimitive(); box.type = SolidPrimitive.BOX; box.dimensions = [0.005, 0.005, 0.005]
        bv.primitives.append(box)
        pose = Pose(); pose.position.x = x; pose.position.y = y; pose.position.z = wrist_z
        bv.primitive_poses.append(pose)
        pc.constraint_region = bv; pc.weight = 1.0

        # 2. 终点姿态约束
        oc = OrientationConstraint()
        oc.header.frame_id = "world"; oc.link_name = "panda_hand"
        oc.orientation.x = 1.0; oc.orientation.y = 0.0; oc.orientation.z = 0.0; oc.orientation.w = 0.0
        
        # 终点姿态稍微严格一点，保证手是正的
        oc.absolute_x_axis_tolerance = 0.1 
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 3.14
        oc.weight = 1.0
        goal.request.goal_constraints.append(Constraints(position_constraints=[pc], orientation_constraints=[oc]))

        # 3. 路径约束 (Strict Mode)
        if strict_mode:
            path_oc = OrientationConstraint()
            path_oc.header.frame_id = "world"; path_oc.link_name = "panda_hand"
            path_oc.orientation.x = 1.0; path_oc.orientation.y = 0.0; path_oc.orientation.z = 0.0; path_oc.orientation.w = 0.0
            # 🔥 放宽路径约束到 0.2，避免太严苛导致规划失败 🔥
            path_oc.absolute_x_axis_tolerance = 0.2
            path_oc.absolute_y_axis_tolerance = 0.2
            path_oc.absolute_z_axis_tolerance = 3.14
            path_oc.weight = 1.0
            goal.request.path_constraints.orientation_constraints.append(path_oc)

        future = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if not res or not res.accepted:
            print("❌ Planning Rejected.")
            return False
            
        res_future = res.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        result_code = res_future.result().result.error_code.val
        if result_code == 1:
            return True
        else:
            print(f"❌ Move Failed! Error Code: {result_code}")
            return False

    def run_inference_grasp(self):
        clear_screen()
        print("==========================================")
        print("🤖 Simulating CNN-based Grasping Pipeline")
        print("==========================================")
        
        # Step 0: 复位
        self.go_to_home()
        input("👉 Robot at Home. Press Enter to start...")

        # Step 1: 张开
        self.control_gripper(0.2) 

        # Step 2: 悬停
        hover_z = CENTER_Z + 0.15
        if not self.move_to_target(PREDICTED_X, PREDICTED_Y, hover_z, "Hovering"): return
        input("👉 Hover reached. Press Enter to descend...")

        # Step 3: 下降 (🔥 去抓上半部分 🔥)
        # 目标是 GRASP_Z (0.490)，比中心点高 2.5cm
        if not self.move_to_target(PREDICTED_X, PREDICTED_Y, GRASP_Z, "Lowering (Top Grasp)", strict_mode=True): return
        input("👉 Grasp position reached. Press Enter to close gripper...")

        # Step 4: 抓取
        self.control_gripper(0.02)
        input("👉 Gripper closed. Press Enter to lift...")

        # Step 5: 抬起
        self.move_to_target(PREDICTED_X, PREDICTED_Y, hover_z, "Lifting object", strict_mode=True)
        print("\n🎉 Done!")

def main():
    rclpy.init()
    node = PandaCNNPicker()
    try:
        node.run_inference_grasp()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()