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
RESULT_FILE = "/shared_data/active_task.yaml"  
TASKS_YAML = "/vision_code/task_test.yaml"         
CAMERA_PARAMS_YAML = "/vision_code/camera_params.yaml" 
MESH_DIR = "/FoundationPose/meshes"            
ASSEMBLY_CENTER_BASE = np.array([0.35, 0.2, 0.025])

# 机械臂参数补偿
ROBOT_READY_YAW_OFFSET = -45.0 
FIXED_BLOCK_Z = 0.015  # 积木中心的标准物理高度 (m)

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
        self.estimator = None

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
        for _ in range(30):
            self.pipeline.wait_for_frames()

    def run(self):
        for task in self.task_list:
            name = task['name']
            while os.path.exists(RESULT_FILE):
                time.sleep(0.5)

            self.clear_buffer()
            print(f"\n🎯 开始识别任务: {name}")
            mesh_path = self.get_mesh_path(name)
            self.pose_est.update_mesh(mesh_path)

            # --- 阶段 1: 目标检测 ---
            obs_start = time.time()
            best_det = None; max_score = -1; best_img_bgr = None
            while (time.time() - obs_start) < 5.0:
                frames = self.pipeline.wait_for_frames()
                img = np.asanyarray(self.align.process(frames).get_color_frame().get_data())
                img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                results = self.detector(img_pil, candidate_labels=[name, "lego block"], threshold=0.3)
                for r in results:
                    if r['score'] > max_score:
                        max_score = r['score']
                        best_det = DetectionResult(r['score'], r['label'], BoundingBox(**r['box']))
                        best_img_bgr = img.copy()
            
            if not best_det: continue

            # --- 阶段 2: 分割与遮罩优化 ---
            if SAM_AVAILABLE:
                self.sam_predictor.set_image(cv2.cvtColor(best_img_bgr, cv2.COLOR_BGR2RGB))
                masks, _, _ = self.sam_predictor.predict(box=np.array(best_det.box.xyxy), multimask_output=False)
                kernel = np.ones((5,5), np.uint8)
                refined_mask = cv2.erode(masks[0].astype(np.uint8), kernel, iterations=1).astype(bool)
            else:
                refined_mask = np.zeros(best_img_bgr.shape[:2], dtype=bool)
                refined_mask[best_det.box.ymin:best_det.box.ymax, best_det.box.xmin:best_det.box.xmax] = True

            # --- 阶段 3: 6D 姿态追踪精炼 (开启 Prior 功能) ---
            print("🔄 正在利用时序追踪锁定积木位姿...")
            T_last_known = None
            refine_start = time.time()
            
            while (time.time() - refine_start) < 4.0:
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                img = np.asanyarray(aligned.get_color_frame().get_data())
                depth_m = np.asanyarray(aligned.get_depth_frame().get_data()).astype(np.float32) / 1000.0
                
                # prior=T_last_known 实现了追踪功能，即使没有角度吸附，也会基于上一帧做平滑收敛
                T_curr = self.pose_est.estimator.register(
                    K=self.K_MATRIX, rgb=img, depth=depth_m, 
                    ob_mask=refined_mask, iteration=10,
                    prior=T_last_known 
                )
                
                if T_curr is not None:
                    T_last_known = T_curr
                    self.visualize_result(img, T_curr)
                cv2.imshow("Robot Assembly Vision", img); cv2.waitKey(1)

            if T_last_known is not None:
                self.send_to_robot(name, T_last_known, task)

    def visualize_result(self, image, T_cam_obj):
        length = 0.05
        axis_pts_3d = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]])
        pts_cam = (T_cam_obj[:3, :3] @ axis_pts_3d.T).T + T_cam_obj[:3, 3]
        pts_2d = []
        for p in pts_cam:
            u = int(self.K_MATRIX[0,0] * p[0]/p[2] + self.K_MATRIX[0,2])
            v = int(self.K_MATRIX[1,1] * p[1]/p[2] + self.1[1,2])
            pts_2d.append((u, v))
        cv2.line(image, pts_2d[0], pts_2d[1], (0,0,255), 3) # X-Red
        cv2.line(image, pts_2d[0], pts_2d[2], (0,255,0), 2) # Y-Green

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        # 1. 坐标转换：相机 -> 基座
        T_base_obj = self.T_base_camera @ T_cam_obj
        R_base = T_base_obj[:3, :3]
        
        # 2. 角度计算：【已移除圆整吸附逻辑】
        # 直接提取视觉解算出的偏航角，支持 360 度随机摆放
        raw_yaw_deg = np.degrees(np.arctan2(R_base[1, 0], R_base[0, 0])) - 90.0 
        
        strategy_spin = float(task_cfg.get('grasp_spin', 0)) 
        blueprint_yaw = float(task_cfg.get('blueprint_yaw', 0)) 
        
        # Pick 角度：完全遵循视觉实测
        final_pick_yaw = raw_yaw_deg + strategy_spin + ROBOT_READY_YAW_OFFSET
        pick_q = R.from_euler('xyz', [180, 0, final_pick_yaw], degrees=True).as_quat().tolist()
        
        # Place 角度：遵循蓝图预设
        final_place_yaw = blueprint_yaw + strategy_spin + ROBOT_READY_YAW_OFFSET
        place_q = R.from_euler('xyz', [180, 0, final_place_yaw], degrees=True).as_quat().tolist()

        # 3. 坐标处理：保留 Z 轴硬约束 (因为桌面是水平的，这个物理事实不会变)
        pick_pos = T_base_obj[:3, 3].tolist()
        pick_pos[2] = FIXED_BLOCK_Z 

        data = {
            'name': name,
            'pick': {'pos': pick_pos, 'orientation': pick_q},
            'place': {
                'pos': (ASSEMBLY_CENTER_BASE + np.array(task_cfg['place']['pos'])).tolist(), 
                'orientation': place_q
            }
        }
        with open(RESULT_FILE, 'w') as f:
            yaml.dump(data, f)

    def get_mesh_path(self, task_name):
        keyword = "4x2" if ("2x4" in task_name or "4x2" in task_name) else "2x2"
        for f in os.listdir(MESH_DIR):
            if keyword in f and f.endswith(".stl"): return os.path.join(MESH_DIR, f)
        return ""

if __name__ == "__main__":
    node = RobotVisionNode()
    node.run()