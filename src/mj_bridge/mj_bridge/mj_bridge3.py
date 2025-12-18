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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# 关键：引入 Action 定义
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# ================= 配置区域 =================
# 务必修改为你的真实绝对路径
SOURCE_DIR = "/home/aaa/robot/ros2_ws/src/mj_bridge/mj_bridge" 
MODEL_XML_PATH = os.path.join(SOURCE_DIR, "scene.xml")
YAML_CONFIG_PATH = os.path.join(SOURCE_DIR, "initial_positions.yaml")

# Action Server 名称
# MoveIt 的 controllers.yaml 里配置的名字通常叫这个
ACTION_NAME = "/panda_arm_controller/follow_joint_trajectory"

# PID 参数
KP = 600.0
KD = 20.0

class MuJoCoActionServer(Node):
    def __init__(self):
        super().__init__('mj_action_server_node')
        self.get_logger().info("正在启动 MuJoCo Action Server...")

        # 1. 加载 MuJoCo 模型
        self.get_logger().info(f"加载模型: {MODEL_XML_PATH}")
        try:
            self.model = mujoco.MjModel.from_xml_path(MODEL_XML_PATH)
            self.data = mujoco.MjData(self.model)
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {e}")
            raise e
        
        #2. create renderer
        self.renderer = mujoco.Renderer(self.model, 480, 640)
        self.cam_name = "realsense"

        # 3. ros2 image publisher
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)

        # 4.clock
        self.timer = self.create_timer(1.0/30.0, self.publish_camera)

        

        # 3. 初始化状态变量
        self.current_goal_handle = None # 当前正在处理的目标句柄
        self.trajectory = None          # 当前轨迹数据
        self.traj_start_time = 0.0      # 轨迹开始的物理时间
        self.is_executing = False       # 是否正在执行
        
        # 目标容器
        self.target_qpos = np.zeros(self.model.nq)
        self.target_qvel = np.zeros(self.model.nv)
        
        # 记录关节名称列表 (用于反馈)
        # 假设前7个关节是我们要控制的，具体根据你的 XML 决定
        # 这里我们动态获取 MoveIt 发来的关节名
        self.active_joint_names = []

        # 2. 加载初始位置
        self.load_initial_positions()

        # 4. 创建 Action Server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            ACTION_NAME,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback
        )
        self.get_logger().info(f"Action Server 已就绪: {ACTION_NAME}")

        self.mj_lock = threading.Lock()
    
    def publish_camera(self):
        # 这里只“读”当前状态，不推进仿真
        with self.mj_lock:   # 第 2 步里会添加这个锁
            self.renderer.update_scene(self.data, camera=self.cam_name)
            rgb = self.renderer.render()

        bgr = rgb[:, :, ::-1]
        msg = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
        self.image_pub.publish(msg)

    def load_initial_positions(self):
        """读取 YAML 设置初始姿态"""
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
        except Exception as e:
            self.get_logger().error(f"YAML 加载错误: {e}")

    # --- Action Server 回调函数 ---

    def goal_callback(self, goal_request):
        """收到新目标请求时触发"""
        self.get_logger().info('收到新的运动规划请求...')
        # 这里可以添加逻辑检查 goal 是否合法
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """收到取消请求时触发 (比如在 Rviz 点击 Stop)"""
        self.get_logger().info('收到取消请求')
        self.is_executing = False
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """请求被接受后触发"""
        
        # 如果有旧目标在运行，先标记为被抢占 (Aborted)
        if self.current_goal_handle is not None and self.current_goal_handle.is_active:
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Goal Preempted"
            self.current_goal_handle.abort(result)

        self.current_goal_handle = goal_handle
        
        # ==========================================
        # 告诉 ROS 状态机：我们将状态从 ACCEPTED 切换为 EXECUTING
        goal_handle.execute()
        # ==========================================

        goal = goal_handle.request
        
        # 提取轨迹信息
        self.trajectory = goal.trajectory
        self.active_joint_names = self.trajectory.joint_names
        self.traj_start_time = self.data.time 
        self.is_executing = True
        self.get_logger().info(f"开始执行轨迹: {len(self.trajectory.points)} 个路点")

    def execute_callback(self, goal_handle):
        """
        标准 Action Server 要求必须有这个回调。
        但在我们的架构里，物理计算在 Main Loop，所以这里我们只需要
        '等待' 任务结束。
        """
        # 这里的 wait 是为了保持 Action 处于 Active 状态
        # 实际的 succeed/abort 信号由 update_action_state() 发出
        while self.is_executing and rclpy.ok():
            time.sleep(0.1) 
        
        # 当 is_executing 变 False 时，说明主循环处理完了
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return FollowJointTrajectory.Result()
        
        # 返回最终结果
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    # --- 核心逻辑：主循环调用的函数 ---

    def update_action_state(self):
        """
        这个函数需要在主循环里每帧调用。
        它负责：
        1. 检查时间，插值计算目标
        2. 发送 Feedback 给 MoveIt
        3. 检查是否到达终点，如果到达则结束 Action
        """
        if not self.is_executing or not self.trajectory or not self.current_goal_handle:
            return

        # 1. 计算时间
        time_now = self.data.time
        t_rel = time_now - self.traj_start_time
        points = self.trajectory.points
        last_point = points[-1]
        t_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9

        # 2. 发送 Feedback (MoveIt 需要这个来更新进度条)
        # 为了节省带宽，可以不每帧都发，比如每 0.1秒发一次
        # 这里简单起见，每次 update 都计算一下
        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.header.stamp = self.get_clock().now().to_msg()
        feedback_msg.joint_names = self.active_joint_names
        
        # 获取当前实际位置
        actual_pos = []
        for name in self.active_joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1:
                actual_pos.append(self.data.qpos[self.model.jnt_qposadr[jid]])
            else:
                actual_pos.append(0.0)
        feedback_msg.actual.positions = actual_pos
        self.current_goal_handle.publish_feedback(feedback_msg)

        # 3. 判断是否结束
        if t_rel >= t_duration:
            self.get_logger().info("轨迹执行完成！")
            self.is_executing = False
            # 设置目标为最后一个点，防止漂移
            self.set_target_direct(last_point)
            
            # 告诉 MoveIt 任务成功！
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            self.current_goal_handle.succeed()
            return

        # 4. 插值计算当前时刻目标 (跟之前一样)
        idx = 0
        for i in range(len(points) - 1):
            p_next = points[i+1]
            t_next = p_next.time_from_start.sec + p_next.time_from_start.nanosec * 1e-9
            if t_rel < t_next:
                idx = i
                break
        
        p0 = points[idx]
        p1 = points[idx+1]
        t0 = p0.time_from_start.sec + p0.time_from_start.nanosec * 1e-9
        t1 = p1.time_from_start.sec + p1.time_from_start.nanosec * 1e-9
        
        alpha = 0.0
        if t1 - t0 > 1e-6:
            alpha = (t_rel - t0) / (t1 - t0)

        for i, name in enumerate(self.active_joint_names):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1:
                qadr = self.model.jnt_qposadr[jid]
                vadr = self.model.jnt_dofadr[jid]
                pos = p0.positions[i] + alpha * (p1.positions[i] - p0.positions[i])
                self.target_qpos[qadr] = pos
                self.target_qvel[vadr] = 0.0

    def set_target_direct(self, point):
        for i, name in enumerate(self.active_joint_names):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1:
                self.target_qpos[self.model.jnt_qposadr[jid]] = point.positions[i]
                self.target_qvel[self.model.jnt_dofadr[jid]] = 0.0

    def step_pid(self):
        """PID 控制 + 重力补偿"""
        # 重力补偿
        for i in range(self.model.nu):
            jid = self.model.actuator_trnid[i, 0]
            dof_adr = self.model.jnt_dofadr[jid]
            self.data.ctrl[i] = self.data.qfrc_bias[dof_adr]

        # PID
        for i in range(self.model.nu):
            jid = self.model.actuator_trnid[i, 0]
            dof_adr = self.model.jnt_dofadr[jid]
            qpos_adr = self.model.jnt_qposadr[jid]
            
            curr_p = self.data.qpos[qpos_adr]
            curr_v = self.data.qvel[dof_adr]
            des_p = self.target_qpos[qpos_adr]
            des_v = self.target_qvel[dof_adr]
            
            self.data.ctrl[i] += KP * (des_p - curr_p) + KD * (des_v - curr_v)

        mujoco.mj_step(self.model, self.data)

def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoActionServer()
    
    # 使用 MultiThreadedExecutor 允许 Action 回调并行处理
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # 开启一个线程来跑 ROS 消息处理 (这样 execute_callback 的等待不会卡死主线程)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # 主线程专门负责 MuJoCo 渲染和物理循环
    with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
        while viewer.is_running() and rclpy.ok():
            start_time = time.time()
            
            # 1. 更新 Action 状态 (插值、反馈、检查完成)
            node.update_action_state()
            
            # 2. 物理步进 (超采样)
            for _ in range(5):
                node.step_pid()
            
            # 3. 渲染
            viewer.sync()

            # 4. 简单的帧率控制
            elapsed = time.time() - start_time
            if elapsed < 0.016:
                time.sleep(0.016 - elapsed)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()