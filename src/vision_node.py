import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import GetBlockPose
import numpy as np
import pyrealsense2 as rs
import yaml
import time
import cv2
import torch
import os
from PIL import Image
from scipy.spatial.transform import Rotation as R

# --- 导入视觉算法链 ---
from transformers import pipeline
# 假设你的 FoundationPose 和 SAM 封装在 estimater 模块中
try:
    from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
    # 如果使用 SAM，通常需要导入相应的 Predictor
    # from segment_anything import sam_model_registry, SamPredictor 
except ImportError:
    print("❌ 错误: 未能在路径中找到 FoundationPose 或相关依赖")

class LegoVisionService(Node):
    def __init__(self):
        super().__init__('lego_vision_service')
        
        # 1. 加载配置路径
        self.declare_parameter('task_yaml', '/path/to/tasks.yaml')
        self.declare_parameter('extr_yaml', '/path/to/camera_to_base.yaml')
        self.declare_parameter('mesh_dir', '/path/to/meshes/')

        # 2. 加载外参 T_base_cam (固定)
        with open(self.get_parameter('extr_yaml').value, "r") as f:
            extr = yaml.safe_load(f)
            self.T_base_cam = np.array(extr["T_base_cam"], dtype=np.float32)

        # 3. 加载任务模板 (预设 6D Pose)
        with open(self.get_parameter('task_yaml').value, "r") as f:
            self.task_templates = yaml.safe_load(f)["tasks"]

        # 4. 初始化算法链 (DINO + FoundationPose)
        # GroundingDINO 用于 2D 检测
        self.detector = pipeline(model="IDEA-Research/grounding-dino-tiny", task="zero-shot-object-detection", device="cuda")
        
        # FoundationPose 组件
        self.scorer = ScorePredictor()
        self.refiner = PoseRefinePredictor()
        # 注意：FoundationPose 实例通常在 handle 时根据目标动态加载对应的 Mesh

        # 5. RealSense 初始化与对齐
        self.pipeline_rs = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline_rs.start(config)
        
        # 获取相机内参 K
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float32)
        
        # 核心：对齐深度图到彩色图
        self.align = rs.align(rs.stream.color)

        # 6. 开启 Service
        self.srv = self.create_service(GetBlockPose, 'get_block_pose', self.handle_get_pose)
        self.get_logger().info("✅ 视觉识别 & 6D 偏移修正服务已启动")

    def handle_get_pose(self, request, response):
        target = request.block_name
        self.get_logger().info(f"🚀 收到任务: 识别 [{target}] 并修正抓取点...")

        # A. 获取该物体在 YAML 里的预设 6D 抓取偏移 (相对于物体中心)
        template = next((t for t in self.task_templates if t["name"] == target), None)
        if not template:
            response.success = False
            return response
        T_pick_in_obj = self.make_matrix(template["pick"]["pos"], template["pick"]["orientation"])

        # B. 7 秒稳定识别优化循环
        start_t = time.time()
        best_pose_cam = None

        while (time.time() - start_t) < 7.0:
            frames = self.pipeline_rs.wait_for_frames()
            frames = self.align.process(frames)
            color_f = frames.get_color_frame()
            depth_f = frames.get_depth_frame()
            if not color_f or not depth_f: continue

            rgb = np.asanyarray(color_f.get_data())
            depth = np.asanyarray(depth_f.get_data()).astype(np.float32) / 1000.0

            # 1. GroundingDINO 检测
            res = self.detector(Image.fromarray(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)), 
                                candidate_labels=[target], threshold=0.3)
            
            if res:
                # 2. 获取 Mask (这里简化为由 DINO Box 生成，如果有 SAM 则在此处生成像素级 Mask)
                box = res[0]["box"]
                mask = np.zeros(depth.shape, dtype=bool)
                mask[int(box["ymin"]):int(box["ymax"]), int(box["xmin"]):int(box["xmax"])] = True
                
                # 3. FoundationPose 解算物体中心相对于相机位姿
                try:
                    # 假定你已根据 target 加载了对应的 mesh 采样点
                    # curr_pose = self.estimator.register(K=self.K, rgb=rgb, depth=depth, ob_mask=mask)
                    # if curr_pose is not None: best_pose_cam = curr_pose
                    pass # 实际运行时取消注释并对接具体接口
                except Exception as e:
                    self.get_logger().warn(f"FP 解算异常: {e}")

            # 可视化进度
            cv2.imshow("Vision Processing", rgb)
            cv2.waitKey(1)

        # C. 坐标转换与响应
        if best_pose_cam is not None:
            # 1. 计算物体真实中心在 Base 系下的位置
            T_base_obj = self.T_base_cam @ best_pose_cam
            
            # 2. 将 YAML 的 6D 抓取点应用到真实物体上
            T_real_pick = T_base_obj @ T_pick_in_obj
            
            # 3. 姿态约束：强制 Roll=180, Pitch=0, 提取 Yaw
            yaw = np.arctan2(T_real_pick[1, 0], T_real_pick[0, 0])
            q_final = R.from_euler('xyz', [np.pi, 0, yaw]).as_quat()

            # 填充 Response
            response.real_pose.position.x = float(T_real_pick[0, 3])
            response.real_pose.position.y = float(T_real_pick[1, 3])
            response.real_pose.position.z = float(T_real_pick[2, 3])
            response.real_pose.orientation.x, response.real_pose.orientation.y, \
            response.real_pose.orientation.z, response.real_pose.orientation.w = q_final
            
            response.success = True
            self.get_logger().info(f"✅ 修正完成: Yaw={np.degrees(yaw):.2f}°")
        else:
            response.success = False
            self.get_logger().error("❌ 7秒超时，未识别到物体")

        return response

    def make_matrix(self, pos, quat):
        mat = np.eye(4)
        mat[:3, 3] = pos
        mat[:3, :3] = R.from_quat(quat).as_matrix()
        return mat

def main():
    rclpy.init()
    node = LegoVisionService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline_rs.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
