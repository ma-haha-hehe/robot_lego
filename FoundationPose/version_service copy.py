import cv2
import torch
import numpy as np
import pyrealsense2 as rs
import os
import trimesh
import yaml
import time
from PIL import Image
from dataclasses import dataclass
from typing import Optional
from transformers import pipeline

# --- 尝试导入 FoundationPose 核心组件 ---
try:
    from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
    FOUNDATION_POSE_READY = True
except ImportError:
    print("❌ 警告: 未找到 estimater.py，请确保该文件在同一目录下。")
    FOUNDATION_POSE_READY = False

# ================= 数据结构 =================
@dataclass
class BoundingBox:
    xmin: int; ymin: int; xmax: int; ymax: int
    @property
    def xyxy(self): return [self.xmin, self.ymin, self.xmax, self.ymax]

@dataclass
class DetectionResult:
    score: float; label: str; box: BoundingBox
    pose: Optional[np.ndarray] = None

# ================= 工具函数 =================
def draw_pose_visuals(image, pose, K, label_name, length=0.06):
    """ 绘制 3D 坐标轴和标签 """
    if pose is None: return image
    points_3d = np.float32([[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, length]])
    R, t = pose[:3, :3], pose[:3, 3]
    points_cam = (R @ points_3d.T).T + t
    
    points_2d = []
    for p in points_cam:
        if p[2] <= 0: return image
        u = int(K[0, 0] * p[0] / p[2] + K[0, 2])
        v = int(K[1, 1] * p[1] / p[2] + K[1, 2])
        points_2d.append((u, v))
    
    origin = points_2d[0]
    cv2.line(image, origin, points_2d[1], (0, 0, 255), 3) # X-Red
    cv2.line(image, origin, points_2d[2], (0, 255, 0), 3) # Y-Green
    cv2.line(image, origin, points_2d[3], (255, 0, 0), 3) # Z-Blue
    
    cv2.putText(image, f"Target: {label_name}", (origin[0] + 10, origin[1] - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return image

# ================= 核心估计类 =================
class LegoPoseEstimator:
    def __init__(self, mesh_path):
        if not FOUNDATION_POSE_READY: return
        print(f">>> 正在加载 FoundationPose 模型...")
        self.scorer = ScorePredictor()
        self.refiner = PoseRefinePredictor()
        
        print(f" -> 正在加载并校准网格中心: {mesh_path}")
        self.mesh = trimesh.load(mesh_path)
        
        # 自动缩放修复
        scale_check = np.linalg.norm(self.mesh.extents)
        if scale_check > 0.1:
            print(f"⚠️ 检测到模型尺寸过大，正在缩放至米...")
            self.mesh.apply_scale(0.001)
            
        center_offset = self.mesh.bounds.mean(axis=0)
        self.mesh.vertices -= center_offset
            
        # 🔥 关键修复：增加采样点到 2048 以提高精度
        model_pts, _ = trimesh.sample.sample_surface(self.mesh, 2048)
        self.model_pts = torch.from_numpy(model_pts.astype(np.float32)).cuda()
        
        self.estimator = FoundationPose(
            model_pts=self.model_pts, model_normals=None, mesh=self.mesh,
            scorer=self.scorer, refiner=self.refiner
        )
        
        self.estimator.mesh = self.mesh
        self.estimator.model_pts = self.model_pts
        self.estimator.center = torch.mean(self.model_pts, dim=0)
        self.estimator.diameter = float(np.linalg.norm(self.mesh.extents))

    def run_once(self, rgb, depth, K, detections):
        depth = depth.astype(np.float32)
        K = K.astype(np.float32)
        H, W = depth.shape
        for det in detections:
            try:
                mask = np.zeros((H, W), dtype=bool)
                y1, y2 = max(0, det.box.ymin), min(H, det.box.ymax)
                x1, x2 = max(0, det.box.xmin), min(W, det.box.xmax)
                mask[y1:y2, x1:x2] = True

                # 🔥 增加迭代次数到 15 以应对白色积木
                pose = self.estimator.register(
                    K=K, rgb=rgb, depth=depth, ob_mask=mask, iteration=15
                )
                return pose
            except Exception as e:
                print(f"❌ 估计出错: {e}")
        return None

# ================= 主程序逻辑 =================
if __name__ == "__main__":
    MESH_PATH = "meshes/LEGO_Duplo_brick_4x2.stl"
    TARGET_FILE = "current_target.txt"
    OUTPUT_FILE = "vision_output.yaml"

    # 1. 硬件初始化
    pipeline_rs = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        profile = pipeline_rs.start(config)
        # 🔥 硬件优化：设置高精度预设以减少深度空洞
        sensor = profile.get_device().first_depth_sensor()
        if sensor.supports(rs.option.visual_preset):
            sensor.set_option(rs.option.visual_preset, 3) # 3: High Accuracy
    except Exception as e:
        print(f"!!! 无法启动 RealSense: {e}")
        exit()

    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K_MATRIX = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
    align = rs.align(rs.stream.color)

    # 2. AI 初始化
    print(">>> 正在加载 DINO 检测器...")
    detector = pipeline(model="IDEA-Research/grounding-dino-tiny", task="zero-shot-object-detection", device="cuda")
    estimator = LegoPoseEstimator(MESH_PATH)

    print(">>> 视觉服务就绪，等待 Executor 指令...")

    try:
        while True:
            # --- 步骤 A: 检查点名指令 ---
            if not os.path.exists(TARGET_FILE):
                time.sleep(0.2)
                continue
            
            with open(TARGET_FILE, 'r') as f:
                active_target = f.read().strip()
            
            if not active_target:
                continue

            # --- 步骤 B: 图像采集 ---
            frames = pipeline_rs.wait_for_frames()
            frames = align.process(frames)
            img = np.asanyarray(frames.get_color_frame().get_data())
            depth_m = np.asanyarray(frames.get_depth_frame().get_data()).astype(np.float32) / 1000.0

            # --- 步骤 C: 针对性检测 ---
            img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            # 仅寻找被点名的标签
            results = detector(img_pil, candidate_labels=[active_target], threshold=0.3)
            
            img_display = img.copy()
            if results:
                res = results[0]
                det = DetectionResult(res['score'], res['label'], BoundingBox(**res['box']))
                
                # --- 步骤 D: 位姿估计 ---
                captured_pose = estimator.run_once(img, depth_m, K_MATRIX, [det])
                
                if captured_pose is not None:
                    # 写入 YAML 供 Executor 读取
                    with open(OUTPUT_FILE, 'w') as f:
                        yaml.dump({active_target: captured_pose.tolist()}, f)
                    
                    # 渲染可视化
                    img_display = draw_pose_visuals(img_display, captured_pose, K_MATRIX, active_target)
                    # 💡 建议增加：任务完成确认，防止视觉一直卡在同一个目标上死循环
                    print(f"✅ {active_target} 位姿已发送，等待 Executor 处理...")
                    # 只有当 Executor 删除了 target 文件，视觉才进入下一个循环
                    while os.path.exists(TARGET_FILE):
                        # 继续显示画面保持实时预览，但不做计算
                        cv2.imshow("Vision Service [Active Mode]", img_display)
                        cv2.waitKey(1)
                        if not os.path.exists(TARGET_FILE): break
                else:
                    print(f"⚠️ 无法解算 {active_target} 的位姿")

            # 界面渲染
            cv2.putText(img_display, f"Targeting: {active_target}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Vision Service [Active Mode]", img_display)
            
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    finally:
        pipeline_rs.stop()
        cv2.destroyAllWindows()