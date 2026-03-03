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

# ================= 1. 路径与环境配置 =================
FP_REPO = "/FoundationPose"
RESULT_FILE = "/shared_data/active_task.yaml"  # 握手信号文件
TASKS_YAML = "/vision_code/tasksh.yaml"         # 任务清单
CAMERA_PARAMS_YAML = "/vision_code/camera_params.yaml" 
MESH_DIR = "/FoundationPose/meshes"            
ASSEMBLY_CENTER_BASE = np.array([0.25, 0, 0.0]) 

# 机械臂初始 Ready 位姿的角度偏移 (用于补偿)
ROBOT_READY_YAW_OFFSET = -45.0 

if FP_REPO not in sys.path:
    sys.path.append(FP_REPO)
    sys.path.append(os.path.join(FP_REPO, "root"))

from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor

try:
    from segment_anything import sam_model_registry, SamPredictor
    SAM_AVAILABLE = True
except ImportError:
    SAM_AVAILABLE = False

@dataclass
class BoundingBox:
    xmin: int; ymin: int; xmax: int; ymax: int
    @property
    def xyxy(self): return [self.xmin, self.ymin, self.xmax, self.ymax]

@dataclass
class DetectionResult:
    score: float; label: str; box: BoundingBox

# ================= 2. 核心估计类 =================
class LegoPoseEstimator:
    def __init__(self, scorer, refiner):
        self.scorer = scorer
        self.refiner = refiner

    def update_mesh(self, mesh_path):
        self.mesh = trimesh.load(mesh_path)
        if np.linalg.norm(self.mesh.extents) > 0.1: 
            self.mesh.apply_scale(0.001)
        self.mesh.vertices -= self.mesh.bounds.mean(axis=0)
        model_pts, _ = trimesh.sample.sample_surface(self.mesh, 2048)
        self.model_pts = torch.from_numpy(model_pts.astype(np.float32)).cuda()
        self.estimator = FoundationPose(
            model_pts=self.model_pts, model_normals=None, mesh=self.mesh,
            scorer=self.scorer, refiner=self.refiner
        )

# ================= 3. 自动化视觉节点类 =================
class RobotVisionNode:
    def __init__(self):
        with open(CAMERA_PARAMS_YAML, 'r') as f:
            params = yaml.safe_load(f)
        self.T_base_camera = np.array(params['extrinsic_matrix']).reshape(4, 4)
        
        with open(TASKS_YAML, 'r') as f:
            data = yaml.safe_load(f)
            self.task_list = data.get('tasksh', data.get('tasks', []))

        self.init_realsense()
        self.detector = pipeline(model="IDEA-Research/grounding-dino-tiny", task="zero-shot-object-detection", device="cuda")
        
        if SAM_AVAILABLE:
            sam = sam_model_registry["vit_h"](checkpoint="/FoundationPose/weights/sam_vit_h_4b8939.pth").to("cuda")
            self.sam_predictor = SamPredictor(sam)

        self.pose_est = LegoPoseEstimator(ScorePredictor(), PoseRefinePredictor())

    def init_realsense(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K_MATRIX = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        self.align = rs.align(rs.stream.color)

    def clear_buffer(self):
        """ 关键：丢弃缓冲区中的旧帧，确保看到的是机械臂离开后的实时场景 """
        print("🧹 正在清理相机缓存，确保获取实时画面...")
        for _ in range(30):
            self.pipeline.wait_for_frames()

    def run(self):
        """ 严格顺序：识别 -> 执行 -> 删除 -> 识别下一个 """
        for task in self.task_list:
            name = task['name']
            
            # 1. 检查上一个任务是否真的彻底消失
            while os.path.exists(RESULT_FILE):
                print("⏳ 等待 C++ 节点删除旧的任务信号文件...")
                time.sleep(1.0)

            # 2. 清理缓存，准备识别新物体
            self.clear_buffer()
            
            print(f"\n🎯 顺序开始下一个任务识别: {name}")
            mesh_path = self.get_mesh_path(name)
            self.pose_est.update_mesh(mesh_path)

            # --- 阶段 1: 目标检测 (8s) ---
            obs_start = time.time()
            best_det = None; max_score = -1; best_img_bgr = None
            while (time.time() - obs_start) < 8.0:
                frames = self.pipeline.wait_for_frames()
                img = np.asanyarray(self.align.process(frames).get_color_frame().get_data())
                img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                results = self.detector(img_pil, candidate_labels=[name, "lego block."], threshold=0.3)
                for r in results:
                    if r['score'] > max_score:
                        max_score = r['score']
                        best_det = DetectionResult(r['score'], r['label'], BoundingBox(**r['box']))
                        best_img_bgr = img.copy()
                cv2.imshow("Robot Assembly Vision", img); cv2.waitKey(1)

            if not best_det:
                print(f"❌ 未发现目标 {name}，跳过。"); continue

            # --- 阶段 2: 分割 ---
            if SAM_AVAILABLE:
                self.sam_predictor.set_image(cv2.cvtColor(best_img_bgr, cv2.COLOR_BGR2RGB))
                masks, _, _ = self.sam_predictor.predict(box=np.array(best_det.box.xyxy), multimask_output=False)
                refined_mask = masks[0]
            else:
                refined_mask = np.zeros(best_img_bgr.shape[:2], dtype=bool)
                refined_mask[best_det.box.ymin:best_det.box.ymax, best_det.box.xmin:best_det.box.xmax] = True

            # --- 阶段 3: 6D 姿态精炼 (4s) ---
            pose_samples = []; refine_start = time.time()
            while (time.time() - refine_start) < 4.0:
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                img = np.asanyarray(aligned.get_color_frame().get_data())
                depth_m = np.asanyarray(aligned.get_depth_frame().get_data()).astype(np.float32) / 1000.0
                T_curr = self.pose_est.estimator.register(K=self.K_MATRIX, rgb=img, depth=depth_m, ob_mask=refined_mask, iteration=30)
                if T_curr is not None:
                    pose_samples.append(T_curr)
                    self.visualize_result(img, T_curr)
                cv2.imshow("Robot Assembly Vision", img); cv2.waitKey(1)

            # --- 阶段 4: 下发指令并再次进入阻塞等待 ---
            if pose_samples:
                self.send_to_robot(name, pose_samples[-1], task)

    def visualize_result(self, image, T_cam_obj):
        length = 0.05
        axis_pts_3d = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]])
        pts_cam = (T_cam_obj[:3, :3] @ axis_pts_3d.T).T + T_cam_obj[:3, 3]
        pts_2d = []
        for p in pts_cam:
            u = int(self.K_MATRIX[0,0] * p[0]/p[2] + self.K_MATRIX[0,2])
            v = int(self.K_MATRIX[1,1] * p[1]/p[2] + self.K_MATRIX[1,2])
            pts_2d.append((u, v))
        cv2.line(image, pts_2d[0], pts_2d[1], (0,0,255), 3) # X轴-红 (对齐基准)
        cv2.line(image, pts_2d[0], pts_2d[2], (0,255,0), 2)
        cv2.line(image, pts_2d[0], pts_2d[3], (255,0,0), 2)

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        """ 连续角度下发 + 45度补偿 """
        T_base_obj = self.T_base_camera @ T_cam_obj
        R_base = T_base_obj[:3, :3]
        
        # 1. 提取连续 Yaw (红轴角度)
        raw_yaw_deg = np.degrees(np.arctan2(R_base[1, 0], R_base[0, 0]))
        # 2. 180度归一化
        stable_yaw_deg = ((raw_yaw_deg + 90) % 180) - 90
        # 3. 45度初始偏移补偿
        final_robot_yaw_deg = stable_yaw_deg - ROBOT_READY_YAW_OFFSET
        final_robot_yaw_deg = (final_robot_yaw_deg + 180) % 360 - 180
        
        pick_q = R.from_euler('xyz', [np.pi, 0, np.radians(final_robot_yaw_deg)]).as_quat().tolist()
        
        place_yaw_abs = task_cfg['place']['orientation'][2] if isinstance(task_cfg['place']['orientation'], list) else 0
        place_q = R.from_euler('xyz', [np.pi, 0, np.radians(place_yaw_abs - ROBOT_READY_YAW_OFFSET)]).as_quat().tolist()

        data = {
            'name': name,
            'pick': {'pos': T_base_obj[:3, 3].tolist(), 'orientation': pick_q},
            'place': {'pos': (ASSEMBLY_CENTER_BASE + np.array(task_cfg['place']['pos'])).tolist(), 'orientation': place_q}
        }
        
        with open(RESULT_FILE, 'w') as f:
            yaml.dump(data, f)
            
        print(f"✅ 指令已写入信号文件: {name} (检测角: {stable_yaw_deg:.1f}°, 下发角: {final_robot_yaw_deg:.1f}°)")
        print(f"🛑 视觉节点进入阻塞，等待 C++ 完成并删除文件...")
        
        # 严格阻塞，直到 C++ 删除文件
        while os.path.exists(RESULT_FILE):
            time.sleep(0.5)
        print("🔓 检测到信号文件已删除，开始准备下一个任务...")

    def get_mesh_path(self, task_name):
        keyword = "4x2" if ("2x4" in task_name or "4x2" in task_name) else "2x2"
        for f in os.listdir(MESH_DIR):
            if keyword in f and f.endswith(".stl"): return os.path.join(MESH_DIR, f)
        return ""

if __name__ == "__main__":
    node = RobotVisionNode()
    node.run()