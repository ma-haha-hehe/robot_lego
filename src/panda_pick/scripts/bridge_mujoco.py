#!/usr/bin/env python3
import time
import yaml
import numpy as np
import mujoco
import os
import threading
import sys

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

KP = 600.0
KD = 20.0
# ===========================================

class MuJoCoActionServer(Node):
    def __init__(self):
        super().__init__('mj_action_server_node')
        self.get_logger().info("启动 MuJoCo Bridge (Wrist Camera Fixed)...")

        try:
            self.model = mujoco.MjModel.from_xml_path(MODEL_XML_PATH)
            self.data = mujoco.MjData(self.model)
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {e}")
            raise e
        
        # 渲染器 (使用 320x240 避免缓冲区溢出)
        self.renderer = mujoco.Renderer(self.model, 320, 240) 
        self.bridge = CvBridge()
        
        # 定义两个相机的发布者
        self.global_cam_pub = self.create_publisher(Image, "/camera/global/image_raw", 10)
        self.hand_cam_pub = self.create_publisher(Image, "/camera/hand/image_raw", 10)

        # 相机名称对应 XML
        self.cam_global_name = "realsense"
        self.cam_hand_name = "hand_camera"

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.all_joint_names = [
            "panda_joint1", "panda_joint2", "panda_joint3", 
            "panda_joint4", "panda_joint5", "panda_joint6", 
            "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"
        ]

        self.arm_goal_handle = None
        self.arm_trajectory = None
        self.arm_start_time = 0.0
        self.arm_is_executing = False
        
        self.hand_goal_handle = None
        self.hand_target_pos = 0.0
        self.hand_is_executing = False

        self.target_qpos = np.zeros(self.model.nq)
        self.target_qvel = np.zeros(self.model.nv)
        self.frame_count = 0 
        
        self.load_initial_positions()

        # [修复] 必须使用关键字参数 (execute_callback=...)
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

        self.get_logger().info("Action Servers Ready.")
        self.mj_lock = threading.Lock()

    def load_initial_positions(self):
        try:
            with open(YAML_CONFIG_PATH, 'r') as f:
                config = yaml.safe_load(f)
            positions = config.get('initial_positions', {})
            for j_name, j_val in positions.items():
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j_name)
                if jid != -1:
                    qadr = self.model.jnt_qposadr[jid]
                    self.data.qpos[qadr] = j_val
                    self.target_qpos[qadr] = j_val
            mujoco.mj_forward(self.model, self.data)
        except Exception: pass

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.all_joint_names
        pos, vel = [], []
        for name in self.all_joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1:
                pos.append(self.data.qpos[self.model.jnt_qposadr[jid]])
                vel.append(self.data.qvel[self.model.jnt_dofadr[jid]])
            else:
                pos.append(0.0); vel.append(0.0)
        msg.position = pos; msg.velocity = vel
        self.joint_pub.publish(msg)

    def publish_camera(self):
        # 全局相机
        try:
            self.renderer.update_scene(self.data, camera=self.cam_global_name)
            rgb = self.renderer.render()
            msg = self.bridge.cv2_to_imgmsg(rgb[:, :, ::-1], encoding="bgr8")
            self.global_cam_pub.publish(msg)
        except Exception: pass

        # 手眼相机
        try:
            self.renderer.update_scene(self.data, camera=self.cam_hand_name)
            rgb = self.renderer.render()
            msg = self.bridge.cv2_to_imgmsg(rgb[:, :, ::-1], encoding="bgr8")
            self.hand_cam_pub.publish(msg)
        except Exception: pass

    def goal_callback(self, goal_request): return GoalResponse.ACCEPT
    def cancel_callback(self, goal_handle): return CancelResponse.ACCEPT

    def handle_arm_accepted(self, goal_handle):
        if self.arm_goal_handle and self.arm_goal_handle.is_active: self.arm_goal_handle.abort()
        self.arm_goal_handle = goal_handle
        goal_handle.execute()
        self.arm_trajectory = goal_handle.request.trajectory
        self.arm_start_time = self.data.time 
        self.arm_is_executing = True
        self.get_logger().info(f"[Arm] Start")

    def execute_arm_callback(self, goal_handle):
        while self.arm_is_executing and rclpy.ok(): time.sleep(0.01)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return FollowJointTrajectory.Result()
        goal_handle.succeed()
        return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)

    def update_arm_logic(self):
        if not self.arm_is_executing or not self.arm_trajectory: return
        t_rel = self.data.time - self.arm_start_time
        points = self.arm_trajectory.points
        last_t = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
        
        if self.arm_goal_handle and self.arm_goal_handle.is_active:
            fb = FollowJointTrajectory.Feedback()
            fb.header.stamp = self.get_clock().now().to_msg()
            fb.joint_names = self.arm_trajectory.joint_names
            self.arm_goal_handle.publish_feedback(fb)

        if t_rel >= last_t:
            self.arm_is_executing = False
            self.set_target_from_point(points[-1], self.arm_trajectory.joint_names)
            return
        
        idx = 0
        for i in range(len(points)-1):
            if t_rel < (points[i+1].time_from_start.sec + points[i+1].time_from_start.nanosec*1e-9):
                idx = i; break
        p0, p1 = points[idx], points[idx+1]
        t0 = p0.time_from_start.sec + p0.time_from_start.nanosec * 1e-9
        t1 = p1.time_from_start.sec + p1.time_from_start.nanosec * 1e-9
        alpha = (t_rel - t0) / (t1 - t0) if (t1-t0) > 1e-6 else 0
        
        for i, name in enumerate(self.arm_trajectory.joint_names):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1:
                self.target_qpos[self.model.jnt_qposadr[jid]] = p0.positions[i] + alpha * (p1.positions[i] - p0.positions[i])

    def set_target_from_point(self, point, joint_names):
        for i, name in enumerate(joint_names):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1: self.target_qpos[self.model.jnt_qposadr[jid]] = point.positions[i]

    def handle_hand_accepted(self, goal_handle):
        if self.hand_goal_handle and self.hand_goal_handle.is_active: self.hand_goal_handle.abort()
        self.hand_goal_handle = goal_handle
        goal_handle.execute()
        self.hand_target_pos = goal_handle.request.command.position
        self.hand_is_executing = True
    
    def execute_hand_callback(self, goal_handle):
        time.sleep(0.5)
        self.hand_is_executing = False
        with self.mj_lock: self.update_hand_logic()
        goal_handle.succeed()
        return GripperCommand.Result(position=self.hand_target_pos, reached_goal=True)

    def update_hand_logic(self):
        for name in ["panda_finger_joint1", "panda_finger_joint2"]:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1:
                self.data.qpos[self.model.jnt_qposadr[jid]] = self.hand_target_pos
                self.data.qvel[self.model.jnt_dofadr[jid]] = 0.0
                self.target_qpos[self.model.jnt_qposadr[jid]] = self.hand_target_pos

    def step_pid(self):
        for i in range(self.model.nu):
            jid = self.model.actuator_trnid[i, 0]
            jname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if jname and "finger" in jname: continue
            self.data.ctrl[i] = self.data.qfrc_bias[self.model.jnt_dofadr[jid]]
        for i in range(self.model.nu):
            jid = self.model.actuator_trnid[i, 0]
            jname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if jname and "finger" in jname: continue
            dof = self.model.jnt_dofadr[jid]
            qadr = self.model.jnt_qposadr[jid]
            self.data.ctrl[i] += KP * (self.target_qpos[qadr] - self.data.qpos[qadr]) + KD * (self.target_qvel[dof] - self.data.qvel[dof])
        mujoco.mj_step(self.model, self.data)

def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    while rclpy.ok():
        start_time = time.time()
        with node.mj_lock:
            node.update_arm_logic()
            node.update_hand_logic()
            for _ in range(50): node.step_pid()
        node.publish_joint_states() 
        node.frame_count += 1
        if node.frame_count % 10 == 0: node.publish_camera()
        elapsed = time.time() - start_time
        if elapsed < 0.016: time.sleep(0.016 - elapsed)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()