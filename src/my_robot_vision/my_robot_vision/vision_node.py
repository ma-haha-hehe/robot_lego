#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import numpy as np
import pyrealsense2 as rs
import os
import yaml
import trimesh
import time
import sys
from PIL import Image
from dataclasses import dataclass
from typing import Optional
from transformers import pipeline
from scipy.spatial.transform import Rotation as R

# ================= 1. 路径与路径注入 =================
FP_REPO = "/FoundationPose"
RESULT_FILE = "/shared_data/active_task.yaml"
TASKS_YAML = "/vision_code/tasks.yaml"
CAMERA_PARAMS_YAML = "/vision_code/camera_params.yaml"
MESH_DIR = "/FoundationPose/meshes"
ASSEMBLY_CENTER_BASE = np.array([0.45, -0.20, 0.02])

if FP_REPO not in sys.path:
    sys.path.append(FP_REPO)
    sys.path.append(os.path.join(FP_REPO, "root"))

# --- 尝试导入 FoundationPose 核心组件 ---
try:
    from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
    FOUNDATION_POSE_READY = True
except ImportError as e:
    print(f"❌ 错误: 无法加载 FoundationPose 模块: {e}")
    FOUNDATION_POSE_READY = False

# ================= 2. 数据结构 =================
@dataclass
class BoundingBox:
    xmin: int; ymin: int; xmax: int; ymax: int
    @property
    def xyxy(self): return [self.xmin, self.ymin, self.xmax, self.ymax]

@dataclass
class DetectionResult:
    score: float
    label: str
    box: BoundingBox
    pose: Optional[np.ndarray] = None

# ================= 3. 核心估计类 (基于你提供的代码) =================
class LegoPoseEstimator:
    def __init__(self, scorer, refiner):
        self.scorer = scorer
        self.refiner = refiner

    def update_mesh(self, mesh_path):
        """ 动态更新当前识别的积木模型 """
        print(f"📦 正在加载模型: {os.path.basename(mesh_path)}")
        self.mesh = trimesh.load(mesh_path)
        
        # 自动缩放修复
        scale_check = np.linalg.norm(self.mesh.extents)
        if scale_check > 0.1: self.mesh.apply_scale(0.001)
            
        # 💡 核心修复：几何中心校准
        center_offset = self.mesh.bounds.mean(axis=0)
        self.mesh.vertices -= center_offset
            
        # 采样点云
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
                return self.estimator.register(K=K, rgb=rgb, depth=depth, ob_mask=mask, iteration=10)
            except Exception as e:
                print(f"❌ 估计出错: {e}")
        return None

# ================= 4. 自动化流水线类 =================
class RobotVisionNode:
    def __init__(self):
        # 加载相机外参
        with open(CAMERA_PARAMS_YAML, 'r') as f:
            params = yaml.safe_load(f)
        self.T_base_camera = np.array(params['extrinsic_matrix']).reshape(4, 4)
        
        # 加载任务列表
        with open(TASKS_YAML, 'r') as f:
            self.task_list = yaml.safe_load(f)['tasks']

        # 初始化硬件
        self.init_realsense()
        
        # 初始化 AI
        print(">>> 正在初始化 AI 组件...")
        self.detector = pipeline(model="IDEA-Research/grounding-dino-tiny", task="zero-shot-object-detection", device="cuda")
        scorer = ScorePredictor()
        refiner = PoseRefinePredictor()
        self.estimator = LegoPoseEstimator(scorer, refiner)

    def init_realsense(self):
        self.pipeline_rs = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline_rs.start(config)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K_MATRIX = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        self.align = rs.align(rs.stream.color)

    def get_mesh_path(self, task_name):
        """ 根据名字里的 2x4 或 2x2 自动匹配模型 """
        keyword = "4x2" if ("2x4" in task_name or "4x2" in task_name) else "2x2"
        for f in os.listdir(MESH_DIR):
            if keyword in f and f.endswith(".stl"):
                return os.path.join(MESH_DIR, f)
        return None

    def run(self):
        for task in self.task_list:
            name = task['name']
            print(f"\n▶️ 开始任务: {name}")
            
            # 更新当前积木的 Mesh
            mesh_path = self.get_mesh_path(name)
            self.estimator.update_mesh(mesh_path)

            # 稳定识别 6 秒
            start_time = time.time()
            T_cam_obj = None
            
            while (time.time() - start_time) < 6.0:
                frames = self.pipeline_rs.wait_for_frames()
                frames = self.align.process(frames)
                img = np.asanyarray(frames.get_color_frame().get_data())
                depth_m = np.asanyarray(frames.get_depth_frame().get_data()).astype(np.float32) / 1000.0
                
                # 实时检测与反馈
                img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                results = self.detector(img_pil, candidate_labels=[name, "block."], threshold=0.3)
                dets = [DetectionResult(0, r['label'], BoundingBox(**r['box'])) for r in results]
                
                T_cam_obj = self.estimator.run_once(img, depth_m, self.K_MATRIX, dets)
                
                if T_cam_obj is not None:
                    # 绘制 3D 坐标轴（使用你代码中的逻辑）
                    self.draw_pose(img, T_cam_obj)
                
                cv2.imshow("Robot Vision Node", img)
                if cv2.waitKey(1) & 0xFF == ord('q'): break

            if T_cam_obj is None:
                print(f"❌ 识别失败，跳过 {name}"); continue

            # --- 核心：坐标变换 (Camera -> Base) ---
            # 1. 获取任务偏移量 T_offset
            T_offset = np.eye(4)
            T_offset[:3, :3] = R.from_quat(task['pick']['orientation']).as_matrix()
            T_offset[:3, 3] = task['pick']['pos']
            
            # 2. 全局变换: T_base_pick = T_base_camera @ T_cam_obj @ T_offset
            T_base_pick = self.T_base_camera @ T_cam_obj @ T_offset
            pick_q = R.from_matrix(T_base_pick[:3, :3]).as_quat()
            
            # 3. 计算放置位置
            final_place_pos = ASSEMBLY_CENTER_BASE + np.array(task['place']['pos'])
            
            # 4. 输出到 shared_data/active_task.yaml
            res = {
                'id': task.get('id', 0),
                'name': name,
                'pick': {
                    'pos': T_base_pick[:3, 3].tolist(),
                    'orientation': pick_q.tolist()
                },
                'place': {
                    'pos': final_place_pos.tolist(),
                    'orientation': task['place']['orientation']
                }
            }
            
            with open(RESULT_FILE, 'w') as f:
                yaml.dump(res, f)
            print(f"✅ {name} 识别完成。坐标已写入 {RESULT_FILE}")

            # 5. 等待机械臂执行并删除文件
            print("⏳ 等待机械臂执行动作...")
            while os.path.exists(RESULT_FILE):
                time.sleep(0.5)

    def draw_pose(self, image, pose, length=0.06):
        """ 绘制坐标轴预览 """
        points_3d = np.float32([[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, length]])
        R_m, t = pose[:3, :3], pose[:3, 3]
        pts_cam = (R_m @ points_3d.T).T + t
        pts_2d = []
        for p in pts_cam:
            u = int(self.K_MATRIX[0, 0] * p[0] / p[2] + self.K_MATRIX[0, 2])
            v = int(self.K_MATRIX[1, 1] * p[1] / p[2] + self.K_MATRIX[1, 2])
            pts_2d.append((u, v))
        cv2.line(image, pts_2d[0], pts_2d[1], (0, 0, 255), 2)
        cv2.line(image, pts_2d[0], pts_2d[2], (0, 255, 0), 2)
        cv2.line(image, pts_2d[0], pts_2d[3], (255, 0, 0), 2)

if __name__ == "__main__":
    node = RobotVisionNode()
    try:
        node.run()
    finally:
        cv2.destroyAllWindows()