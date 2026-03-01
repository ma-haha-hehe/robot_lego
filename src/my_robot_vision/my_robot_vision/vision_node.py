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
TASKS_YAML = "/vision_code/tasks.yaml"        
CAMERA_PARAMS_YAML = "/vision_code/camera_params.yaml" 
MESH_DIR = "/FoundationPose/meshes"           
# 组装区基准：用于 Place 坐标计算
ASSEMBLY_CENTER_BASE = np.array([0.25, 0, 0.0]) 

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
            # 读取任务列表，注意键名需与你的 plan 生成的文件一致
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

    def run(self):
        for task in self.task_list:
            name = task['name']
            print(f"\n" + "="*60 + f"\n🎯 正在识别: {name}")
            
            mesh_path = self.get_mesh_path(name)
            self.pose_est.update_mesh(mesh_path)

            # --- 阶段 1: 观察 ---
            obs_start = time.time()
            best_det = None
            max_score = -1
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

            if not best_det: continue

            # --- 阶段 2: 分割 ---
            if SAM_AVAILABLE:
                self.sam_predictor.set_image(cv2.cvtColor(best_img_bgr, cv2.COLOR_BGR2RGB))
                masks, _, _ = self.sam_predictor.predict(box=np.array(best_det.box.xyxy), multimask_output=False)
                refined_mask = masks[0]
            else:
                refined_mask = np.zeros(best_img_bgr.shape[:2], dtype=bool)
                refined_mask[best_det.box.ymin:best_det.box.ymax, best_det.box.xmin:best_det.box.xmax] = True

            # --- 阶段 3: 精炼 ---
            pose_samples = []
            refine_start = time.time()
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
        cv2.line(image, pts_2d[0], pts_2d[1], (0,0,255), 2)
        cv2.line(image, pts_2d[0], pts_2d[2], (0,255,0), 2)
        cv2.line(image, pts_2d[0], pts_2d[3], (255,0,0), 2)

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        """ 
        核心逻辑：
        1. Pick Pos: 视觉实时中心 (Vision Center)
        2. Pick Orn: 完全听从视觉检测到的 Yaw (锁定垂直向下)
        3. Place Pos: 原始逻辑 (ASSEMBLY_CENTER_BASE + 任务偏移)
        4. Place Orn: 与抓取时一致 (完全听从视觉检测到的 Yaw)
        """
        # A. 提取视觉实时位姿并转换到机器人基座坐标系
        T_base_vision = self.T_base_camera @ T_cam_obj
        r_vision = R.from_matrix(T_base_vision[:3, :3])
        
        # 提取视觉检测到的实时偏航角 (Yaw) 
        # 忽略任务表里的任何角度建议，只看现场看到的
        detected_yaw = r_vision.as_euler('zyx', degrees=False)[0]

        # B. 【核心修改】构造姿态：强制锁定垂直，Yaw 完全取自视觉
        # Roll = pi (180度), Pitch = 0, Yaw = detected_yaw
        # 这样能保证夹爪永远垂直，且完全对齐视觉看到的积木角度
        r_final = R.from_euler('xyz', [np.pi, 0, detected_yaw], degrees=False)
        final_quat = r_final.as_quat().tolist()

        # C. 提取位置信息
        vision_pick_pos = T_base_vision[:3, 3].tolist() # 视觉中心点
        
        # Place 坐标保持原始逻辑：基准 + YAML 中的相对偏移量
        original_place_pos = (ASSEMBLY_CENTER_BASE + np.array(task_cfg['place']['pos'])).tolist()

        # D. 构造最终数据
        data = {
            'name': name,
            'pick': {
                'pos': vision_pick_pos,      # 听视觉的
                'orientation': final_quat     # 听视觉的 (锁定垂直)
            },
            'place': {
                'pos': original_place_pos,    # 听原始逻辑的
                'orientation': final_quat     # 听视觉的 (保证放下的角度和抓的时候一样)
            }
        }
        
        with open(RESULT_FILE, 'w') as f:
            yaml.dump(data, f)
            
        print(f"✅ 视觉全驱动模式：已应用检测角度 {np.degrees(detected_yaw):.1f}°")
        print(f"   - 抓取位置: {vision_pick_pos}")
        
        # 等待 C++ 节点读取文件并执行
        while os.path.exists(RESULT_FILE):
            time.sleep(0.5)

    def get_mesh_path(self, task_name):
        keyword = "4x2" if "2x4" in task_name or "4x2" in task_name else "2x2"
        for f in os.listdir(MESH_DIR):
            if keyword in f and f.endswith(".stl"): return os.path.join(MESH_DIR, f)
        return ""

if __name__ == "__main__":
    node = RobotVisionNode()
    node.run()