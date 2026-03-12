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
from transformers import pipeline
from scipy.spatial.transform import Rotation as R

# ================= 1. 路径与环境配置 =================
FP_REPO = "/FoundationPose"
RESULT_FILE = "/shared_data/active_task.yaml"
TASKS_YAML = "/vision_code/task_test_door.yaml"
CAMERA_PARAMS_YAML = "/vision_code/camera_params.yaml"
MESH_DIR = "/FoundationPose/meshes"
ASSEMBLY_CENTER_BASE = np.array([0.35, 0.2, 0.025])

if FP_REPO not in sys.path:
    sys.path.append(FP_REPO)
    sys.path.append(os.path.join(FP_REPO, "root"))

from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor

try:
    from segment_anything import sam_model_registry, SamPredictor
    SAM_AVAILABLE = True
except ImportError:
    SAM_AVAILABLE = False

# ================= 2. 颜色识别配置 (HSV 空间) =================
# 格式: "颜色名": ([H_min, S_min, V_min], [H_max, S_max, V_max])
COLOR_RANGES = {
    "red":    [([0, 120, 70], [10, 255, 255]), ([170, 120, 70], [180, 255, 255])], # 红色跨越0度
    "blue":   [([100, 120, 50], [130, 255, 255])],
    "green":  [([40, 100, 50], [80, 255, 255])],
    "yellow": [([20, 120, 100], [35, 255, 255])],
}

def check_color_match(img_bgr, mask, target_color_name):
    """
    检查 SAM 掩码区域内的平均颜色是否匹配目标颜色
    """
    if target_color_name not in COLOR_RANGES:
        return True, 1.0 # 如果任务没设颜色，默认匹配
    
    hsv_img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    # 提取掩码内的像素
    pixels = hsv_img[mask > 0]
    if len(pixels) == 0: return False, 0.0
    
    avg_hsv = np.mean(pixels, axis=0)
    
    matched = False
    for (lower, upper) in COLOR_RANGES[target_color_name]:
        lower = np.array(lower)
        upper = np.array(upper)
        if np.all(avg_hsv >= lower) and np.all(avg_hsv <= upper):
            matched = True
            break
            
    return matched, avg_hsv

# ================= 3. 核心估计与节点类 =================
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

class RobotVisionNode:
    def __init__(self):
        with open(CAMERA_PARAMS_YAML, 'r') as f:
            params = yaml.safe_load(f)
        self.T_base_camera = np.array(params['extrinsic_matrix']).reshape(4, 4)
        with open(TASKS_YAML, 'r') as f:
            data = yaml.safe_load(f)
            self.task_list = data.get('tasks', data.get('tasksh', []))

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
        RATIO_TOLERANCE = 0.3 
        
        for task in self.task_list:
            name = task['name'].lower()
            
            # --- 自动识别任务中的颜色关键词 ---
            target_color = None
            for c_name in COLOR_RANGES.keys():
                if c_name in name:
                    target_color = c_name
                    break

            while os.path.exists(RESULT_FILE):
                time.sleep(0.5)

            print(f"\n🎯 [视觉启动] 任务: {name} | 目标颜色: {target_color if target_color else '未指定'}")
            mesh_path = self.get_mesh_path(name)
            self.pose_est.update_mesh(mesh_path)
            target_ratio = 2.0 if ("4x2" in name or "2x4" in name) else 1.0

            # 图像采集
            self.pipeline.wait_for_frames() # 预热
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            img_bgr = np.asanyarray(aligned.get_color_frame().get_data())
            img_pil = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
            
            results = self.detector(img_pil, candidate_labels=["lego block"], threshold=0.15)
            
            valid_candidates = []
            viz_frame = img_bgr.copy()

            for r in results:
                box = [r['box']['xmin'], r['box']['ymin'], r['box']['xmax'], r['box']['ymax']]
                self.sam_predictor.set_image(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
                masks, _, _ = self.sam_predictor.predict(box=np.array(box), multimask_output=False)
                mask = masks[0]
                
                # A. 几何校验 (比例)
                contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not contours: continue
                rect = cv2.minAreaRect(max(contours, key=cv2.contourArea))
                w, h = rect[1]
                actual_ratio = max(w, h) / (min(w, h) + 1e-6)
                
                if abs(actual_ratio - target_ratio) > RATIO_TOLERANCE:
                    continue 

                # B. 颜色校验 (新增核心)
                color_matched, avg_hsv = check_color_match(img_bgr, mask, target_color)
                if not color_matched:
                    # 颜色不符，画蓝色框表示“虽然是积木但颜色不对”
                    cv2.rectangle(viz_frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 1)
                    continue
                
                # C. 通过校验
                match_score = r['score'] * (1.0 / (abs(actual_ratio - target_ratio) + 0.1))
                valid_candidates.append({'mask': mask, 'score': match_score, 'box': box})
                cv2.rectangle(viz_frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 255), 2)

            if not valid_candidates:
                print(f" 未找到匹配目标。")
                cv2.imshow("Detection", viz_frame); cv2.waitKey(500); continue

            winner = max(valid_candidates, key=lambda x: x['score'])
            
            # --- 6D 位姿追踪 ---
            start_time = time.time()
            pose_samples = []
            while (time.time() - start_time) < 3.0:
                f = self.pipeline.wait_for_frames()
                a = self.align.process(f)
                rgb = np.asanyarray(a.get_color_frame().get_data())
                dep = np.asanyarray(a.get_depth_frame().get_data()).astype(np.float32) / 1000.0
                
                T_curr = self.pose_est.estimator.register(K=self.K_MATRIX, rgb=rgb, depth=dep, ob_mask=winner['mask'])
                if T_curr is not None:
                    pose_samples.append(T_curr)
                    self.draw_axis(rgb, T_curr)
                cv2.imshow("Tracking", rgb)
                cv2.waitKey(1)

            if pose_samples:
                self.send_to_robot(name, pose_samples[-1], task)

    def draw_axis(self, img, T):
        l = 0.05
        pts = np.float32([[0,0,0], [l,0,0], [0,l,0], [0,0,l]])
        pts_c = (T[:3,:3] @ pts.T).T + T[:3,3]
        p2d = []
        for p in pts_c:
            u = int(self.K_MATRIX[0,0]*p[0]/p[2] + self.K_MATRIX[0,2])
            v = int(self.K_MATRIX[1,1]*p[1]/p[2] + self.K_MATRIX[1,2])
            p2d.append((u,v))
        cv2.line(img, p2d[0], p2d[1], (0,0,255), 3)
        cv2.line(img, p2d[0], p2d[2], (0,255,0), 3)
        cv2.line(img, p2d[0], p2d[3], (255,0,0), 3)

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        T_base_obj = self.T_base_camera @ T_cam_obj
        pos = T_base_obj[:3, 3]
        R_base = T_base_obj[:3, :3]
        raw_yaw = np.degrees(np.arctan2(R_base[1, 0], R_base[0, 0]))
        refined_yaw = ((raw_yaw + 90) % 180) - 90
        
        spin = float(task_cfg.get('grasp_spin', 0))
        blueprint_yaw = float(task_cfg.get('blueprint_yaw', 0))
        
        brick_q = R.from_euler('xyz', [0, 0, refined_yaw], degrees=True).as_quat().tolist()
        pick_q = R.from_euler('xyz', [180, 0, refined_yaw + spin - 135], degrees=True).as_quat().tolist()
        place_q = R.from_euler('xyz', [180, 0, blueprint_yaw + spin - 45], degrees=True).as_quat().tolist()

        data = {
            'name': name,
            'brick_info': {'pos': pos.tolist(), 'orientation': brick_q},
            'robot_pick': {'orientation': pick_q},
            'place': {
                'pos': (ASSEMBLY_CENTER_BASE + np.array(task_cfg['place']['pos'])).tolist(),
                'orientation': place_q
            }
        }
        with open(RESULT_FILE, 'w') as f:
            yaml.dump(data, f)
        print(f"✅ 结果已保存至 {RESULT_FILE}")

    def get_mesh_path(self, task_name):
        keyword = "4x2" if ("4x2" in task_name or "2x4" in task_name) else "2x2"
        for f in os.listdir(MESH_DIR):
            if keyword in f and f.endswith(".stl"): return os.path.join(MESH_DIR, f)
        return ""

if __name__ == "__main__":
    RobotVisionNode().run()