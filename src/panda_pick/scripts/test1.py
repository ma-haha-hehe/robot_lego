#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
from typing import Dict, Optional, List, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, RobotState

from control_msgs.action import FollowJointTrajectory, GripperCommand


# =========================
# 你只需要改这里
# =========================
MOVEGROUP_ACTION = "move_action"
MOVEIT_GROUP = "panda_arm"
MOVEIT_EE_LINK = "panda_hand"      # 先用 panda_hand 跑通；以后可换真正 TCP link
WORLD_FRAME = "world"

ARM_EXEC_ACTION = "/panda_arm_controller/follow_joint_trajectory"
GRIPPER_ACTION  = "/panda_hand_controller/gripper_cmd"

# 抓取目标（世界坐标系）
# 你可以先手动填 brick_1 的几何中心坐标（来自 mujoco body xpos / 或你 xml）
PICK_CENTER = np.array([0.42, -0.12, 0.44], dtype=float)

# 放置点（世界坐标）
PLACE_CENTER = np.array([0.55, 0.20, 0.46], dtype=float)

# 高度策略
HOVER_DZ   = 0.16     # 悬停高度（相对目标 z）
APPROACH_DZ = 0.02    # 抓取时距离中心的偏移（避免直接撞）

# 夹爪（注意：你的 bridge 里 hand_target_pos = command.position，单位就是关节位移）
GRIPPER_OPEN  = 0.04
GRIPPER_CLOSE = 0.00
GRIPPER_EFFORT = 50.0  # 你的 bridge 没用 effort 也没关系

# MoveIt 约束（先放宽跑通，再收紧）
POS_BOX = 0.03   # 3cm 位置容差盒
ORI_Q_XYZW = (1.0, 0.0, 0.0, 0.0)  # 先照你之前的“向下”四元数占位
ORI_TOL_X = 1.2
ORI_TOL_Y = 1.2
ORI_TOL_Z = math.pi

PLANNING_TIME = 5.0
VEL_SCALE = 0.15
ACC_SCALE = 0.15

# 关节名（必须与你 bridge 发布的 /joint_states 一致）
ARM_JOINTS = [
    "panda_joint1", "panda_joint2", "panda_joint3",
    "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
]
FINGER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]
# =========================


def _now_s() -> float:
    return time.time()


class PickPlaceRouteA(Node):
    """
    Route A:
    - MoveIt: plan_only -> get joint trajectory
    - Execute: send trajectory to /panda_arm_controller/follow_joint_trajectory (MuJoCo bridge)
    - Gripper: /panda_hand_controller/gripper_cmd (MuJoCo bridge)
    - Start state sync: always build MoveIt request.start_state from latest /joint_states (MuJoCo truth)
    """

    def __init__(self):
        super().__init__("pick_place_routeA")

        # joint_states subscriber (MuJoCo truth)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self._latest_js: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, qos)

        # MoveIt planner
        self._moveit = ActionClient(self, MoveGroup, MOVEGROUP_ACTION)
        # Execution (MuJoCo bridge)
        self._arm_exec = ActionClient(self, FollowJointTrajectory, ARM_EXEC_ACTION)
        self._gripper  = ActionClient(self, GripperCommand, GRIPPER_ACTION)

        self.get_logger().info("Waiting for MoveGroup action server...")
        self._moveit.wait_for_server()
        self.get_logger().info("✅ MoveGroup action server ready.")

        self.get_logger().info("Waiting for MuJoCo arm/gripper action servers...")
        self._arm_exec.wait_for_server()
        self._gripper.wait_for_server()
        self.get_logger().info("✅ MuJoCo bridge action servers ready.")

    # ---------- joint_states ----------
    def _js_cb(self, msg: JointState):
        if not msg.name or not msg.position:
            return
        self._latest_js = msg

    def wait_joint_states(self, timeout_s=5.0) -> JointState:
        t0 = _now_s()
        while _now_s() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_js is None:
                continue
            names = set(self._latest_js.name)
            if all(j in names for j in ARM_JOINTS):
                return self._latest_js
        raise RuntimeError("Timeout waiting for /joint_states with panda_joint1..7 (MuJoCo bridge maybe not running?)")

    def build_start_state(self, js: JointState) -> RobotState:
        name_to_pos = {n: p for n, p in zip(js.name, js.position)}

        start_js = JointState()
        start_js.name = []
        start_js.position = []

        for j in ARM_JOINTS:
            start_js.name.append(j)
            start_js.position.append(float(name_to_pos[j]))

        # 手指可选：加上也没坏处
        for fj in FINGER_JOINTS:
            if fj in name_to_pos:
                start_js.name.append(fj)
                start_js.position.append(float(name_to_pos[fj]))

        rs = RobotState()
        rs.joint_state = start_js
        rs.is_diff = True
        return rs

    # ---------- MoveIt planning ----------
    def plan_to_xyz(self, xyz: np.ndarray) -> "moveit_msgs.msg.RobotTrajectory":
        js = self.wait_joint_states(timeout_s=6.0)
        start_state = self.build_start_state(js)

        # 打印一下，方便你确认“起点一致”
        name_to_pos = {n: p for n, p in zip(js.name, js.position)}
        arm_q = np.array([float(name_to_pos[j]) for j in ARM_JOINTS])
        self.get_logger().info(f"Start (from MuJoCo /joint_states) arm_q = {arm_q}")

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = MOVEIT_GROUP
        req.allowed_planning_time = float(PLANNING_TIME)
        req.num_planning_attempts = 10
        req.max_velocity_scaling_factor = float(VEL_SCALE)
        req.max_acceleration_scaling_factor = float(ACC_SCALE)

        # ✅ 关键：显式指定 start_state，保证起始一致
        req.start_state = start_state

        pc = PositionConstraint()
        pc.header.frame_id = WORLD_FRAME
        pc.link_name = MOVEIT_EE_LINK

        bv = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [POS_BOX, POS_BOX, POS_BOX]
        bv.primitives.append(box)

        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        bv.primitive_poses.append(pose)

        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = WORLD_FRAME
        oc.link_name = MOVEIT_EE_LINK
        oc.orientation.x = float(ORI_Q_XYZW[0])
        oc.orientation.y = float(ORI_Q_XYZW[1])
        oc.orientation.z = float(ORI_Q_XYZW[2])
        oc.orientation.w = float(ORI_Q_XYZW[3])
        oc.absolute_x_axis_tolerance = float(ORI_TOL_X)
        oc.absolute_y_axis_tolerance = float(ORI_TOL_Y)
        oc.absolute_z_axis_tolerance = float(ORI_TOL_Z)
        oc.weight = 1.0

        req.goal_constraints = [Constraints(position_constraints=[pc],
                                            orientation_constraints=[oc])]

        goal.planning_options.plan_only = True

        self.get_logger().info(f"Planning to xyz={xyz} ...")
        fut = self._moveit.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("MoveIt rejected planning goal")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result().result
        if res.error_code.val != 1:
            raise RuntimeError(f"MoveIt planning failed error_code={res.error_code.val}")

        traj = res.planned_trajectory
        jt = traj.joint_trajectory
        self.get_logger().info(f"✅ Plan OK: joints={len(jt.joint_names)}, points={len(jt.points)}")
        return traj

    # ---------- execute trajectory in MuJoCo bridge ----------
    def exec_arm_traj(self, traj) -> None:
        jt = traj.joint_trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt

        self.get_logger().info("Executing arm trajectory in MuJoCo bridge...")
        fut = self._arm_exec.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("Arm controller rejected trajectory goal")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        self.get_logger().info("✅ Arm execution done.")

    def set_gripper(self, width: float) -> None:
        width = float(np.clip(width, 0.0, 0.04))
        goal = GripperCommand.Goal()
        goal.command.position = width
        goal.command.max_effort = float(GRIPPER_EFFORT)

        self.get_logger().info(f"Gripper -> {width:.3f}")
        fut = self._gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("Gripper controller rejected goal")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        self.get_logger().info("✅ Gripper done.")

    # ---------- high-level steps ----------
    def step_move(self, xyz: np.ndarray, label: str):
        self.get_logger().info(f"\n===== STEP: {label} =====")
        traj = self.plan_to_xyz(xyz)
        input("按回车执行该步轨迹（防止一口气乱跑）...")
        self.exec_arm_traj(traj)

    def run(self):
        pick_hover = PICK_CENTER + np.array([0, 0, HOVER_DZ], dtype=float)
        pick_approach = PICK_CENTER + np.array([0, 0, APPROACH_DZ], dtype=float)

        place_hover = PLACE_CENTER + np.array([0, 0, HOVER_DZ], dtype=float)
        place_approach = PLACE_CENTER + np.array([0, 0, APPROACH_DZ], dtype=float)

        # 1) open gripper
        self.get_logger().info("\n===== STEP: open gripper =====")
        self.set_gripper(GRIPPER_OPEN)

        # 2) move to hover above pick
        self.step_move(pick_hover, "move to PICK hover")

        # 3) descend to approach
        self.step_move(pick_approach, "descend to PICK approach")

        # 4) close gripper
        self.get_logger().info("\n===== STEP: close gripper =====")
        self.set_gripper(GRIPPER_CLOSE)

        # 5) lift back to hover
        self.step_move(pick_hover, "lift back to PICK hover")

        # 6) move to place hover
        self.step_move(place_hover, "move to PLACE hover")

        # 7) descend to place approach
        self.step_move(place_approach, "descend to PLACE approach")

        # 8) open gripper (release)
        self.get_logger().info("\n===== STEP: release =====")
        self.set_gripper(GRIPPER_OPEN)

        # 9) lift
        self.step_move(place_hover, "lift back to PLACE hover")

        self.get_logger().info("\n✅ Pick & Place sequence finished.")


def main():
    rclpy.init()
    node = PickPlaceRouteA()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"FAILED: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
from typing import Dict, Optional, List, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, RobotState

from control_msgs.action import FollowJointTrajectory, GripperCommand


# =========================
# 你只需要改这里
# =========================
MOVEGROUP_ACTION = "move_action"
MOVEIT_GROUP = "panda_arm"
MOVEIT_EE_LINK = "panda_hand"      # 先用 panda_hand 跑通；以后可换真正 TCP link
WORLD_FRAME = "world"

ARM_EXEC_ACTION = "/panda_arm_controller/follow_joint_trajectory"
GRIPPER_ACTION  = "/panda_hand_controller/gripper_cmd"

# 抓取目标（世界坐标系）
# 你可以先手动填 brick_1 的几何中心坐标（来自 mujoco body xpos / 或你 xml）
PICK_CENTER = np.array([0.42, -0.12, 0.44], dtype=float)

# 放置点（世界坐标）
PLACE_CENTER = np.array([0.55, 0.20, 0.46], dtype=float)

# 高度策略
HOVER_DZ   = 0.16     # 悬停高度（相对目标 z）
APPROACH_DZ = 0.02    # 抓取时距离中心的偏移（避免直接撞）

# 夹爪（注意：你的 bridge 里 hand_target_pos = command.position，单位就是关节位移）
GRIPPER_OPEN  = 0.04
GRIPPER_CLOSE = 0.00
GRIPPER_EFFORT = 50.0  # 你的 bridge 没用 effort 也没关系

# MoveIt 约束（先放宽跑通，再收紧）
POS_BOX = 0.03   # 3cm 位置容差盒
ORI_Q_XYZW = (1.0, 0.0, 0.0, 0.0)  # 先照你之前的“向下”四元数占位
ORI_TOL_X = 1.2
ORI_TOL_Y = 1.2
ORI_TOL_Z = math.pi

PLANNING_TIME = 5.0
VEL_SCALE = 0.15
ACC_SCALE = 0.15

# 关节名（必须与你 bridge 发布的 /joint_states 一致）
ARM_JOINTS = [
    "panda_joint1", "panda_joint2", "panda_joint3",
    "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
]
FINGER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]
# =========================


def _now_s() -> float:
    return time.time()


class PickPlaceRouteA(Node):
    """
    Route A:
    - MoveIt: plan_only -> get joint trajectory
    - Execute: send trajectory to /panda_arm_controller/follow_joint_trajectory (MuJoCo bridge)
    - Gripper: /panda_hand_controller/gripper_cmd (MuJoCo bridge)
    - Start state sync: always build MoveIt request.start_state from latest /joint_states (MuJoCo truth)
    """

    def __init__(self):
        super().__init__("pick_place_routeA")

        # joint_states subscriber (MuJoCo truth)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self._latest_js: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, qos)

        # MoveIt planner
        self._moveit = ActionClient(self, MoveGroup, MOVEGROUP_ACTION)
        # Execution (MuJoCo bridge)
        self._arm_exec = ActionClient(self, FollowJointTrajectory, ARM_EXEC_ACTION)
        self._gripper  = ActionClient(self, GripperCommand, GRIPPER_ACTION)

        self.get_logger().info("Waiting for MoveGroup action server...")
        self._moveit.wait_for_server()
        self.get_logger().info("✅ MoveGroup action server ready.")

        self.get_logger().info("Waiting for MuJoCo arm/gripper action servers...")
        self._arm_exec.wait_for_server()
        self._gripper.wait_for_server()
        self.get_logger().info("✅ MuJoCo bridge action servers ready.")

    # ---------- joint_states ----------
    def _js_cb(self, msg: JointState):
        if not msg.name or not msg.position:
            return
        self._latest_js = msg

    def wait_joint_states(self, timeout_s=5.0) -> JointState:
        t0 = _now_s()
        while _now_s() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_js is None:
                continue
            names = set(self._latest_js.name)
            if all(j in names for j in ARM_JOINTS):
                return self._latest_js
        raise RuntimeError("Timeout waiting for /joint_states with panda_joint1..7 (MuJoCo bridge maybe not running?)")

    def build_start_state(self, js: JointState) -> RobotState:
        name_to_pos = {n: p for n, p in zip(js.name, js.position)}

        start_js = JointState()
        start_js.name = []
        start_js.position = []

        for j in ARM_JOINTS:
            start_js.name.append(j)
            start_js.position.append(float(name_to_pos[j]))

        # 手指可选：加上也没坏处
        for fj in FINGER_JOINTS:
            if fj in name_to_pos:
                start_js.name.append(fj)
                start_js.position.append(float(name_to_pos[fj]))

        rs = RobotState()
        rs.joint_state = start_js
        rs.is_diff = True
        return rs

    # ---------- MoveIt planning ----------
    def plan_to_xyz(self, xyz: np.ndarray) -> "moveit_msgs.msg.RobotTrajectory":
        js = self.wait_joint_states(timeout_s=6.0)
        start_state = self.build_start_state(js)

        # 打印一下，方便你确认“起点一致”
        name_to_pos = {n: p for n, p in zip(js.name, js.position)}
        arm_q = np.array([float(name_to_pos[j]) for j in ARM_JOINTS])
        self.get_logger().info(f"Start (from MuJoCo /joint_states) arm_q = {arm_q}")

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = MOVEIT_GROUP
        req.allowed_planning_time = float(PLANNING_TIME)
        req.num_planning_attempts = 10
        req.max_velocity_scaling_factor = float(VEL_SCALE)
        req.max_acceleration_scaling_factor = float(ACC_SCALE)

        # ✅ 关键：显式指定 start_state，保证起始一致
        req.start_state = start_state

        pc = PositionConstraint()
        pc.header.frame_id = WORLD_FRAME
        pc.link_name = MOVEIT_EE_LINK

        bv = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [POS_BOX, POS_BOX, POS_BOX]
        bv.primitives.append(box)

        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        bv.primitive_poses.append(pose)

        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = WORLD_FRAME
        oc.link_name = MOVEIT_EE_LINK
        oc.orientation.x = float(ORI_Q_XYZW[0])
        oc.orientation.y = float(ORI_Q_XYZW[1])
        oc.orientation.z = float(ORI_Q_XYZW[2])
        oc.orientation.w = float(ORI_Q_XYZW[3])
        oc.absolute_x_axis_tolerance = float(ORI_TOL_X)
        oc.absolute_y_axis_tolerance = float(ORI_TOL_Y)
        oc.absolute_z_axis_tolerance = float(ORI_TOL_Z)
        oc.weight = 1.0

        req.goal_constraints = [Constraints(position_constraints=[pc],
                                            orientation_constraints=[oc])]

        goal.planning_options.plan_only = True

        self.get_logger().info(f"Planning to xyz={xyz} ...")
        fut = self._moveit.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("MoveIt rejected planning goal")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result().result
        if res.error_code.val != 1:
            raise RuntimeError(f"MoveIt planning failed error_code={res.error_code.val}")

        traj = res.planned_trajectory
        jt = traj.joint_trajectory
        self.get_logger().info(f"✅ Plan OK: joints={len(jt.joint_names)}, points={len(jt.points)}")
        return traj

    # ---------- execute trajectory in MuJoCo bridge ----------
    def exec_arm_traj(self, traj) -> None:
        jt = traj.joint_trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt

        self.get_logger().info("Executing arm trajectory in MuJoCo bridge...")
        fut = self._arm_exec.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("Arm controller rejected trajectory goal")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        self.get_logger().info("✅ Arm execution done.")

    def set_gripper(self, width: float) -> None:
        width = float(np.clip(width, 0.0, 0.04))
        goal = GripperCommand.Goal()
        goal.command.position = width
        goal.command.max_effort = float(GRIPPER_EFFORT)

        self.get_logger().info(f"Gripper -> {width:.3f}")
        fut = self._gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("Gripper controller rejected goal")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        self.get_logger().info("✅ Gripper done.")

    # ---------- high-level steps ----------
    def step_move(self, xyz: np.ndarray, label: str):
        self.get_logger().info(f"\n===== STEP: {label} =====")
        traj = self.plan_to_xyz(xyz)
        input("按回车执行该步轨迹（防止一口气乱跑）...")
        self.exec_arm_traj(traj)

    def run(self):
        pick_hover = PICK_CENTER + np.array([0, 0, HOVER_DZ], dtype=float)
        pick_approach = PICK_CENTER + np.array([0, 0, APPROACH_DZ], dtype=float)

        place_hover = PLACE_CENTER + np.array([0, 0, HOVER_DZ], dtype=float)
        place_approach = PLACE_CENTER + np.array([0, 0, APPROACH_DZ], dtype=float)

        # 1) open gripper
        self.get_logger().info("\n===== STEP: open gripper =====")
        self.set_gripper(GRIPPER_OPEN)

        # 2) move to hover above pick
        self.step_move(pick_hover, "move to PICK hover")

        # 3) descend to approach
        self.step_move(pick_approach, "descend to PICK approach")

        # 4) close gripper
        self.get_logger().info("\n===== STEP: close gripper =====")
        self.set_gripper(GRIPPER_CLOSE)

        # 5) lift back to hover
        self.step_move(pick_hover, "lift back to PICK hover")

        # 6) move to place hover
        self.step_move(place_hover, "move to PLACE hover")

        # 7) descend to place approach
        self.step_move(place_approach, "descend to PLACE approach")

        # 8) open gripper (release)
        self.get_logger().info("\n===== STEP: release =====")
        self.set_gripper(GRIPPER_OPEN)

        # 9) lift
        self.step_move(place_hover, "lift back to PLACE hover")

        self.get_logger().info("\n✅ Pick & Place sequence finished.")


def main():
    rclpy.init()
    node = PickPlaceRouteA()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"FAILED: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()