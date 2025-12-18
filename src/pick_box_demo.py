#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    CollisionObject,
    PlanningScene,
    AttachedCollisionObject,
)

from moveit.planning import MoveItPy


# ====== 可根据需要改的常量 ======
WORLD_FRAME = "world"          # 或 "panda_link0"，看你配置
EEF_LINK    = "panda_hand"     # 你的末端/手爪 link（如是 panda_hand_tcp 就改名）
ARM_GROUP   = "panda_arm"
HAND_GROUP  = "hand"
FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")

# 方块参数（与 MuJoCo 保持一致）
BOX_ID   = "cube"
BOX_SIZE = (0.05, 0.05, 0.05)  # X Y Z, 单位 m
BOX_POS  = (0.40, 0.00, 0.025) # 立方体中心（z=半高，正好“落地”）
BOX_Q_WXYZ = (1.0, 0.0, 0.0, 0.0)  # 无旋转

# 抓取动作参数
PREGRASP_OFFSET_Z = 0.10   # 抓取前，方块上方 10cm
APPROACH_DOWN_Z   = 0.06   # 抓取时，末端与地面的高度（自己调到能夹住）
LIFT_UP_Z         = 0.20   # 抓完抬起到 20cm 高
OPEN_WIDTH        = 0.04   # 手指张开宽度（两指各 0.04）
CLOSE_WIDTH       = 0.0    # 闭合（根据你的模型可适当设 0.001~0.005）


def make_pose_stamped(xyz, qwxyz, frame=WORLD_FRAME):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = xyz[0]
    p.pose.position.y = xyz[1]
    p.pose.position.z = xyz[2]
    p.pose.orientation.w = qwxyz[0]
    p.pose.orientation.x = qwxyz[1]
    p.pose.orientation.y = qwxyz[2]
    p.pose.orientation.z = qwxyz[3]
    return p


def add_box_to_scene(node: Node, pub_scene, box_id=BOX_ID):
    scene = PlanningScene()
    scene.is_diff = True

    co = CollisionObject()
    co.id = box_id
    co.header = Header(frame_id=WORLD_FRAME)

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(BOX_SIZE)

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = BOX_POS
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = BOX_Q_WXYZ

    co.primitives.append(box)
    co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD

    scene.world.collision_objects.append(co)
    pub_scene.publish(scene)
    node.get_logger().info(f"PlanningScene: added box '{box_id}'.")


def attach_box(node: Node, pub_scene, link_name=EEF_LINK, box_id=BOX_ID):
    aco = AttachedCollisionObject()
    aco.link_name = link_name
    aco.object.id = box_id
    aco.object.header.frame_id = WORLD_FRAME
    aco.object.operation = CollisionObject.ADD
    # 允许与手爪本体碰撞
    aco.touch_links = [link_name]

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.attached_collision_objects.append(aco)

    pub_scene.publish(scene)
    node.get_logger().info(f"PlanningScene: attached '{box_id}' to '{link_name}'.")


def detach_box(node: Node, pub_scene, link_name=EEF_LINK, box_id=BOX_ID):
    aco = AttachedCollisionObject()
    aco.link_name = link_name
    aco.object.id = box_id
    aco.object.operation = CollisionObject.REMOVE

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.attached_collision_objects.append(aco)

    pub_scene.publish(scene)
    node.get_logger().info(f"PlanningScene: detached '{box_id}' from '{link_name}'.")


def set_hand_width(moveit2: MoveItPy, width: float):
    """用关节目标控制两根手指：每根手指的位移就是 width。"""
    hand = moveit2.get_planning_component(HAND_GROUP)

    joint_goal = {FINGER_JOINTS[0]: width, FINGER_JOINTS[1]: width}
    hand.set_start_state_to_current_state()
    hand.set_goal_state(joint_positions=joint_goal)

    plan = hand.plan()
    assert plan, "Hand planning failed"
    moveit2.execute(plan.trajectory)


def plan_to_pose(moveit2: MoveItPy, pose: PoseStamped):
    arm = moveit2.get_planning_component(ARM_GROUP)
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose, end_effector_link_name=EEF_LINK)

    plan = arm.plan()
    if not plan:
        return None
    return plan


def run_demo():
    rclpy.init()
    node = rclpy.create_node("pick_box_demo")
    moveit2 = MoveItPy(node_name="moveit_py")
    pub_scene = node.create_publisher(PlanningScene, "planning_scene", 10)

    # 1) 往场景里加方块
    rclpy.spin_once(node, timeout_sec=0.5)
    add_box_to_scene(node, pub_scene)

    # 2) 规划到 "预抓取"（方块上方）
    pregrasp_pose = make_pose_stamped(
        (BOX_POS[0], BOX_POS[1], BOX_POS[2] + PREGRASP_OFFSET_Z),
        BOX_Q_WXYZ,
    )
    plan = plan_to_pose(moveit2, pregrasp_pose)
    assert plan, "Plan to pre-grasp failed"
    moveit2.execute(plan.trajectory)

    # 3) 打开夹爪
    set_hand_width(moveit2, OPEN_WIDTH)

    # 4) 直线下压到抓取位（为了简洁，这里用第二个姿态规划）
    grasp_pose = make_pose_stamped(
        (BOX_POS[0], BOX_POS[1], APPROACH_DOWN_Z),
        BOX_Q_WXYZ,
    )
    plan = plan_to_pose(moveit2, grasp_pose)
    assert plan, "Plan to grasp pose failed"
    moveit2.execute(plan.trajectory)

    # 5) 闭合夹爪
    set_hand_width(moveit2, CLOSE_WIDTH)

    # 6) 将方块 attach 到手上
    attach_box(node, pub_scene, link_name=EEF_LINK, box_id=BOX_ID)

    # 7) 抬起
    lift_pose = make_pose_stamped(
        (BOX_POS[0], BOX_POS[1], LIFT_UP_Z),
        BOX_Q_WXYZ,
    )
    plan = plan_to_pose(moveit2, lift_pose)
    assert plan, "Plan to lift failed"
    moveit2.execute(plan.trajectory)

    node.get_logger().info("Pick succeeded. (To place: move above target, open gripper, detach.)")

    rclpy.shutdown()


if __name__ == "__main__":
    run_demo()
