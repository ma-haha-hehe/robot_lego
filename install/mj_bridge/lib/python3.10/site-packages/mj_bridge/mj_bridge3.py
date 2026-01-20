import time
import yaml
import numpy as np
import mujoco
import mujoco.viewer
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 关键：引入 Action 定义
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# ================= 配置区域 =================
SOURCE_DIR = "/home/aaa/robot/ros2_ws/src/mj_bridge/mj_bridge" 
MODEL_XML_PATH = os.path.join(SOURCE_DIR, "scene.xml")
YAML_CONFIG_PATH = os.path.join(SOURCE_DIR, "initial_positions.yaml")

# PID 参数 (针对 Panda 机械臂优化)
KP = 2000.0
KD = 40.0

class MuJoCoActionServer(Node):
    def __init__(self):
        super().__init__('mj_action_server_node')
        self.get_logger().info("正在启动 MuJoCo Action Server (加固版)...")

        # 1. 加载 MuJoCo 模型
        try:
            self.model = mujoco.MjModel.from_xml_path(MODEL_XML_PATH)
            self.data = mujoco.MjData(self.model)
            self.mj_lock = threading.Lock() # 用于保护物理数据安全
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {e}")
            raise e
        
        # 2. 视觉渲染配置
        self.renderer = mujoco.Renderer(self.model, 480, 640)
        self.cam_name = "realsense"
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.timer = self.create_timer(1.0/30.0, self.publish_camera)

        # 3. 初始化控制状态变量
        self.current_goal_handle = None 
        self.trajectory = None          
        self.traj_start_time = 0.0      
        self.is_executing = False       
        self.target_qpos = np.zeros(self.model.nq)
        self.target_qvel = np.zeros(self.model.nv)
        self.active_joint_names = []

        # 4. 加载初始位置
        self.load_initial_positions()

        # 5. 创建两个独立的 Action Server (Arm 和 Hand)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/panda_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback
        )
        
        self._hand_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/panda_hand_controller/follow_joint_trajectory',
            execute_callback=self.execute_hand_callback # 绑定专用回调
        )
        self.get_logger().info("手臂与夹爪 Action Server 均已就绪")

    def load_initial_positions(self):
        """同步初始位置到目标向量"""
        try:
            if os.path.exists(YAML_CONFIG_PATH):
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
        except Exception as e:
            self.get_logger().warn(f"初始位置加载跳过: {e}")

    def execute_hand_callback(self, goal_handle):
        self.get_logger().info('收到夹爪请求...')
        traj = goal_handle.request.trajectory
        if not traj.points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        target_val = traj.points[-1].positions[0]

        with self.mj_lock:
            # 🔥 统一修改 target_qpos，而不是直接写 ctrl
            j1_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'panda_finger_joint1')
            j2_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'panda_finger_joint2')
            if j1_id != -1: self.target_qpos[self.model.jnt_qposadr[j1_id]] = target_val
            if j2_id != -1: self.target_qpos[self.model.jnt_qposadr[j2_id]] = target_val

        # 模拟执行时间，确保物理上合拢了再返回成功
        time.sleep(1.0) 
        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    # --- 手臂 Action Server 回调 ---
    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.is_executing = False
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        if self.current_goal_handle is not None and self.current_goal_handle.is_active:
            self.current_goal_handle.abort()
        
        self.current_goal_handle = goal_handle
        goal_handle.execute()
        
        self.trajectory = goal_handle.request.trajectory
        self.active_joint_names = self.trajectory.joint_names
        self.traj_start_time = self.data.time 
        self.is_executing = True

    def execute_callback(self, goal_handle):
        """手臂 Action 执行循环"""
        while self.is_executing and rclpy.ok():
            time.sleep(0.05) # 降低 CPU 占用
        
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        return FollowJointTrajectory.Result()

    # --- 物理循环逻辑 ---
    def update_action_state(self):
        """主循环每帧调用：处理手臂轨迹插值"""
        if not self.is_executing or not self.trajectory or not self.current_goal_handle:
            return

        t_rel = self.data.time - self.traj_start_time
        points = self.trajectory.points
        last_point = points[-1]
        t_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9

        # 判断是否到达终点
        if t_rel >= t_duration:
            self.is_executing = False
            self.set_target_direct(last_point)
            if self.current_goal_handle.is_active:
                self.current_goal_handle.succeed()
            return

        # 线性插值寻找当前目标点
        idx = 0
        for i in range(len(points) - 1):
            t_next = points[i+1].time_from_start.sec + points[i+1].time_from_start.nanosec * 1e-9
            if t_rel < t_next:
                idx = i
                break
        
        p0, p1 = points[idx], points[idx+1]
        t0 = p0.time_from_start.sec + p0.time_from_start.nanosec * 1e-9
        t1 = p1.time_from_start.sec + p1.time_from_start.nanosec * 1e-9
        alpha = (t_rel - t0) / (t1 - t0) if t1 - t0 > 1e-6 else 0.0

        for i, name in enumerate(self.active_joint_names):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1:
                qadr = self.model.jnt_qposadr[jid]
                pos = p0.positions[i] + alpha * (p1.positions[i] - p0.positions[i])
                self.target_qpos[qadr] = pos

    def set_target_direct(self, point):
        for i, name in enumerate(self.active_joint_names):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1:
                self.target_qpos[self.model.jnt_qposadr[jid]] = point.positions[i]

    def step_pid(self):
        with self.mj_lock:
            # 1. 基础重力补偿
            self.data.ctrl[:] = self.data.qfrc_bias[:self.model.nu]

            # 2. 全关节 PID 控制 (涵盖手臂 7 关节 + 夹爪 2 关节)
            for i in range(self.model.nu): 
                jid = self.model.actuator_trnid[i, 0]
                qadr = self.model.jnt_qposadr[jid]
                vadr = self.model.jnt_dofadr[jid]
                
                # 计算 PID 差值
                error_p = self.target_qpos[qadr] - self.data.qpos[qadr]
                error_v = 0.0 - self.data.qvel[vadr]
                
                self.data.ctrl[i] += KP * error_p + KD * error_v

            mujoco.mj_step(self.model, self.data)

    def publish_camera(self):
        with self.mj_lock:
            self.renderer.update_scene(self.data, camera=self.cam_name)
            rgb = self.renderer.render()
        msg = self.bridge.cv2_to_imgmsg(rgb[:, :, ::-1], encoding="bgr8")
        self.image_pub.publish(msg)

def main():
    rclpy.init()
    node = MuJoCoActionServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # MuJoCo 主循环
    with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
        while viewer.is_running() and rclpy.ok():
            loop_start = time.time()
            node.update_action_state()
            for _ in range(5): # 子步进提高物理精度
                node.step_pid()
            viewer.sync()
            
            # 维持约 60FPS
            elapsed = time.time() - loop_start
            if elapsed < 0.016:
                time.sleep(0.016 - elapsed)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()