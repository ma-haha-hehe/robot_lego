#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import yaml
import numpy as np
import mujoco
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from control_msgs.action import FollowJointTrajectory, GripperCommand


# ================= 配置区域 =================
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_XML_PATH = os.path.join(CURRENT_DIR, "scene.xml")
YAML_CONFIG_PATH = os.path.join(CURRENT_DIR, "initial_positions.yaml")

ARM_ACTION_NAME = "/panda_arm_controller/follow_joint_trajectory"
HAND_ACTION_NAME = "/panda_hand_controller/gripper_cmd"

# 你的 joint 名
ARM_JOINTS = [
    "panda_joint1", "panda_joint2", "panda_joint3",
    "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
]
FINGER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]

# 对应 Mujoco 的 actuator 名（position actuators）
ARM_ACTS = [f"{j}_pos" for j in ARM_JOINTS]
GRIPPER_ACTS = ["finger_joint1", "finger_joint2"]

CAM_GLOBAL = "realsense"
CAM_HAND = "hand_camera"

RENDER_W, RENDER_H = 320, 240
# ===========================================


def _sec(msg_time):
    return float(msg_time.sec) + float(msg_time.nanosec) * 1e-9


class MuJoCoActionServer(Node):
    def __init__(self):
        super().__init__("mj_action_server_node")
        self.get_logger().info("启动 MuJoCo Bridge (Position Actuators)...")

        # --- load model ---
        try:
            self.model = mujoco.MjModel.from_xml_path(MODEL_XML_PATH)
            self.data = mujoco.MjData(self.model)
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {e}")
            raise

        # viewer/renderer
        self.renderer = mujoco.Renderer(self.model, RENDER_W, RENDER_H)
        self.bridge = CvBridge()

        # pubs
        self.global_cam_pub = self.create_publisher(Image, "/camera/global/image_raw", 10)
        self.hand_cam_pub = self.create_publisher(Image, "/camera/hand/image_raw", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.all_joint_names = ARM_JOINTS + FINGER_JOINTS

        # --- map actuator ids (critical) ---
        self.arm_act_ids = self._require_actuators(ARM_ACTS)
        self.gripper_act_ids = self._require_actuators(GRIPPER_ACTS)

        # action states
        self.arm_goal_handle = None
        self.arm_traj = None
        self.arm_start_time = 0.0
        self.arm_is_executing = False

        self.hand_goal_handle = None
        self.hand_target = 0.04
        self.hand_is_executing = False

        # internal targets (joint pos targets)
        self.target_arm = {j: 0.0 for j in ARM_JOINTS}

        self.frame_count = 0
        self.mj_lock = threading.Lock()

        # load init pose
        self.load_initial_positions()

        # action servers
        self._arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            ARM_ACTION_NAME,
            execute_callback=self.execute_arm_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_arm_accepted
        )

        self._hand_server = ActionServer(
            self,
            GripperCommand,
            HAND_ACTION_NAME,
            execute_callback=self.execute_hand_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_hand_accepted
        )

        self.get_logger().info("✅ Action Servers Ready.")

    # ---------- required ids ----------
    def _require_actuators(self, names):
        ids = {}
        for n in names:
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
            if aid == -1:
                raise RuntimeError(
                    f"[FATAL] MuJoCo actuator not found: '{n}'.\n"
                    f"请检查 panda.xml/scene.xml 中 actuator name 是否存在并拼写一致。"
                )
            ids[n] = int(aid)
        return ids

    def _joint_qpos(self, joint_name):
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid == -1:
            return None
        qadr = self.model.jnt_qposadr[jid]
        return qadr

    def _joint_qvel(self, joint_name):
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid == -1:
            return None
        dadr = self.model.jnt_dofadr[jid]
        return dadr

    # ---------- init pose ----------
    def load_initial_positions(self):
        # 1) set qpos from yaml
        try:
            with open(YAML_CONFIG_PATH, "r") as f:
                config = yaml.safe_load(f) or {}
            positions = (config.get("initial_positions", {}) or {})

            for j_name, j_val in positions.items():
                qadr = self._joint_qpos(j_name)
                if qadr is not None:
                    self.data.qpos[qadr] = float(j_val)

            # 2) zero velocities
            self.data.qvel[:] = 0.0
            self.data.qacc[:] = 0.0

            # 3) also initialize targets from current qpos
            for j in ARM_JOINTS:
                qadr = self._joint_qpos(j)
                if qadr is not None:
                    self.target_arm[j] = float(self.data.qpos[qadr])

            # 4) apply ctrl = target positions (position actuators)
            self._apply_arm_ctrl()
            self._apply_gripper_ctrl(self.hand_target)

            mujoco.mj_forward(self.model, self.data)
            self.get_logger().info("✅ Loaded initial_positions.yaml and synced ctrl/qpos.")
        except Exception as e:
            self.get_logger().warn(f"load_initial_positions failed (ignored): {e}")

    # ---------- pub ----------
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.all_joint_names
        pos = []
        vel = []
        for name in self.all_joint_names:
            qadr = self._joint_qpos(name)
            dadr = self._joint_qvel(name)
            if qadr is None or dadr is None:
                pos.append(0.0)
                vel.append(0.0)
            else:
                pos.append(float(self.data.qpos[qadr]))
                vel.append(float(self.data.qvel[dadr]))
        msg.position = pos
        msg.velocity = vel
        self.joint_pub.publish(msg)

    def publish_camera(self):
        # global cam
        try:
            self.renderer.update_scene(self.data, camera=CAM_GLOBAL)
            rgb = self.renderer.render()
            msg = self.bridge.cv2_to_imgmsg(rgb[:, :, ::-1], encoding="bgr8")
            self.global_cam_pub.publish(msg)
        except Exception:
            pass

        # hand cam
        try:
            self.renderer.update_scene(self.data, camera=CAM_HAND)
            rgb = self.renderer.render()
            msg = self.bridge.cv2_to_imgmsg(rgb[:, :, ::-1], encoding="bgr8")
            self.hand_cam_pub.publish(msg)
        except Exception:
            pass

    # ---------- action callbacks ----------
    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def handle_arm_accepted(self, goal_handle):
        # abort previous
        if self.arm_goal_handle and self.arm_goal_handle.is_active:
            self.arm_goal_handle.abort()

        self.arm_goal_handle = goal_handle
        self.arm_traj = goal_handle.request.trajectory
        self.arm_start_time = float(self.data.time)
        self.arm_is_executing = True

        # IMPORTANT: execute after we stored trajectory
        goal_handle.execute()

    def execute_arm_callback(self, goal_handle):
        # just wait until arm_is_executing becomes False
        while self.arm_is_executing and rclpy.ok():
            time.sleep(0.005)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return FollowJointTrajectory.Result()

        goal_handle.succeed()
        return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)

    def handle_hand_accepted(self, goal_handle):
        if self.hand_goal_handle and self.hand_goal_handle.is_active:
            self.hand_goal_handle.abort()
        self.hand_goal_handle = goal_handle
        self.hand_target = float(goal_handle.request.command.position)
        self.hand_is_executing = True
        goal_handle.execute()

    def execute_hand_callback(self, goal_handle):
        # apply immediately
        with self.mj_lock:
            self._apply_gripper_ctrl(self.hand_target)
        time.sleep(0.1)
        self.hand_is_executing = False
        goal_handle.succeed()
        return GripperCommand.Result(position=self.hand_target, reached_goal=True)

    # ---------- control application ----------
    def _apply_arm_ctrl(self):
        # position actuators: ctrl = desired joint position
        for j in ARM_JOINTS:
            act = f"{j}_pos"
            aid = self.arm_act_ids[act]
            self.data.ctrl[aid] = float(self.target_arm[j])

    def _apply_gripper_ctrl(self, pos):
        pos = float(np.clip(pos, 0.0, 0.04))
        self.data.ctrl[self.gripper_act_ids["finger_joint1"]] = pos
        self.data.ctrl[self.gripper_act_ids["finger_joint2"]] = pos

    # ---------- trajectory interpolation ----------
    def update_arm_logic(self):
        if (not self.arm_is_executing) or (self.arm_traj is None):
            return

        points = self.arm_traj.points
        if not points:
            self.arm_is_executing = False
            return

        t_rel = float(self.data.time) - self.arm_start_time
        last_t = _sec(points[-1].time_from_start)

        # finish
        if t_rel >= last_t:
            # set final
            for i, name in enumerate(self.arm_traj.joint_names):
                if name in self.target_arm and i < len(points[-1].positions):
                    self.target_arm[name] = float(points[-1].positions[i])
            self.arm_is_executing = False
            return

        # find segment
        idx = 0
        for i in range(len(points) - 1):
            if t_rel < _sec(points[i + 1].time_from_start):
                idx = i
                break

        p0, p1 = points[idx], points[idx + 1]
        t0 = _sec(p0.time_from_start)
        t1 = _sec(p1.time_from_start)
        alpha = (t_rel - t0) / (t1 - t0) if (t1 - t0) > 1e-9 else 0.0
        alpha = float(np.clip(alpha, 0.0, 1.0))

        for i, name in enumerate(self.arm_traj.joint_names):
            if name not in self.target_arm:
                continue
            if i >= len(p0.positions) or i >= len(p1.positions):
                continue
            q = float(p0.positions[i] + alpha * (p1.positions[i] - p0.positions[i]))
            self.target_arm[name] = q

    # ---------- sim loop ----------
    def step_sim(self, substeps=5):
        # 1) update desired targets from action trajectories
        self.update_arm_logic()

        # 2) write ctrl (position actuators)
        self._apply_arm_ctrl()
        self._apply_gripper_ctrl(self.hand_target)

        # 3) step physics
        for _ in range(substeps):
            mujoco.mj_step(self.model, self.data)


def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    # 首帧就发布 joint_states：让 MoveIt/RViz 立刻拿到“真实初始位姿”
    with node.mj_lock:
        node.publish_joint_states()

    # main loop ~60Hz
    while rclpy.ok():
        t0 = time.time()
        with node.mj_lock:
            node.step_sim(substeps=5)
        node.publish_joint_states()

        node.frame_count += 1
        if node.frame_count % 10 == 0:
            with node.mj_lock:
                node.publish_camera()

        dt = time.time() - t0
        if dt < 0.016:
            time.sleep(0.016 - dt)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()