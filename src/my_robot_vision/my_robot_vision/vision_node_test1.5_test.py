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
TASKS_YAML = "/vision_code/task_test_flower.yaml"
CAMERA_PARAMS_YAML = "/vision_code/camera_params.yaml"
MESH_DIR = "/FoundationPose/meshes"
ASSEMBLY_CENTER_BASE = np.array([0.35, 0.2, 0.025])

ROBOT_READY_YAW_OFFSET = -45

if FP_REPO not in sys.path:
    sys.path.append(FP_REPO)
    sys.path.append(os.path.join(FP_REPO, "root"))

from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor

try:
    from segment_anything import sam_model_registry, SamPredictor
    SAM_AVAILABLE = True
except ImportError:
    SAM_AVAILABLE = False

# 定义颜色的理想中心坐标 (Hue, Saturation, Value)
COLOR_CENTERS = {
    "red":    (0, 200, 150),
    "red_alt": (175, 200, 150),
    "blue":   (115, 200, 150),
    "green":  (60, 200, 120),
    "yellow": (30, 200, 200),
    "white":  (0, 30, 230),  # 调高一点 S，给环境光留余地
    "black":  (0, 0, 40)
}
def clear_buffer(self):
        for _ in range(30): self.pipeline.wait_for_frames()

def get_color_distance(c1, c2):
    """计算两个 HSV 颜色之间的欧几里得距离，并考虑 Hue 的循环性"""
    h1, s1, v1 = c1
    h2, s2, v2 = c2
    
    # Hue 是圆形的 (0 和 180 是相连的)
    dh = min(abs(h1 - h2), 180 - abs(h1 - h2))
    ds = s1 - s2
    dv = v1 - v2
    return np.sqrt(dh**2 + ds**2 + dv**2)

def classify_color(avg_hsv):
    """判定当前 HSV 颜色距离哪个预设中心最近"""
    min_dist = float('inf')
    best_color = "unknown"
    
    for color_name, center in COLOR_CENTERS.items():
        dist = get_color_distance(avg_hsv, center)
        if dist < min_dist:
            min_dist = dist
            best_color = color_name
            
    # 如果最接近红色的两个中心之一，都归类为红色
    if best_color == "red_alt": best_color = "red"
    return best_color

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
        for _ in range(30):
            self.pipeline.wait_for_frames()

    def run(self):
        """
        全流程视觉控制逻辑：集成比例/颜色硬过滤、黑名单记忆机制以及自动重试。
        """
        # 比例容差：允许 0.4 的偏差，防止斜放导致误判
        RATIO_TOLERANCE = 0.4 
        
        for task in self.task_list:
            name = task['name']
            
            # --- 解析任务要求的颜色 ---
            target_color_name = None
            for color_key in ["red", "blue", "green", "yellow", "white", "black"]:
                if color_key in name.lower():
                    target_color_name = color_key
                    break

            task_success = False
            # 【核心】每个新任务开始前，初始化该任务的专属黑名单（坐标记忆）
            blacklist = [] # 存储格式: [(cx1, cy1), ...]

            while not task_success:
                # 0. 任务同步：等待机器人清空 active_task.yaml
                while os.path.exists(RESULT_FILE):
                    print(f"⏳ [{name}] 等待机器人拿走上一块积木...")
                    time.sleep(0.5)

                print(f"\n🎯 [新一轮扫描] 寻找: {name} | 黑名单记录: {len(blacklist)} 个")
                
                # 更新 3D 模型
                mesh_path = self.get_mesh_path(name)
                self.pose_est.update_mesh(mesh_path)
                target_ratio = 2.0 if ("4x2" in name or "2x4" in name) else 1.0

                # --- 1. 图像采集与全图检测 ---
                self.clear_buffer()
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                img_bgr = np.asanyarray(aligned_frames.get_color_frame().get_data())
                img_pil = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
                
                # 运行 GroundingDINO
                results = self.detector(img_pil, candidate_labels=["lego block"], threshold=0.15)
                
                valid_candidates = []
                viz_frame = img_bgr.copy()

                # --- 2. 几何与颜色校验 ---
                for r in results:
                    box = [r['box']['xmin'], r['box']['ymin'], r['box']['xmax'], r['box']['ymax']]
                    cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2

                    # --- A. 检查坐标黑名单 ---
                    is_blacklisted = False
                    for (bx, by) in blacklist:
                        if np.sqrt((cx - bx)**2 + (cy - by)**2) < 20: # 20像素容差
                            is_blacklisted = True
                            break
                    
                    if is_blacklisted:
                        # 黑名单目标画灰色框并跳过，不再重复计算 SAM
                        cv2.rectangle(viz_frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (100, 100, 100), 1)
                        continue

                    # B. SAM 精确掩码
                    self.sam_predictor.set_image(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
                    masks, _, _ = self.sam_predictor.predict(box=np.array(box), multimask_output=False)
                    mask = masks[0]
                    
                    # C. 几何比例校验
                    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if not contours: continue
                    rect = cv2.minAreaRect(max(contours, key=cv2.contourArea))
                    w, h = rect[1]
                    actual_ratio = max(w, h) / (min(w, h) + 1e-6)
                    
                    if abs(actual_ratio - target_ratio) > RATIO_TOLERANCE:
                        print(f"      - [{name}] 比例错误: {actual_ratio:.1f}，加入黑名单")
                        blacklist.append((cx, cy))
                        cv2.rectangle(viz_frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 0, 255), 2)
                        continue 

                    # D. 颜色分类判定 (最近邻分类逻辑)
                    hsv_img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
                    kernel = np.ones((5, 5), np.uint8)
                    eroded_mask = cv2.erode(mask.astype(np.uint8), kernel, iterations=1)
                    mask_pixels = hsv_img[eroded_mask > 0]
                    
                    if len(mask_pixels) > 0:
                        avg_hsv = np.mean(mask_pixels, axis=0)
                        detected_color = classify_color(avg_hsv) # 调用之前的距离算法
                        
                        if target_color_name and detected_color != target_color_name:
                            print(f"      - [{name}] 颜色不对 (检测到: {detected_color})，加入黑名单")
                            blacklist.append((cx, cy))
                            # 颜色不对画蓝框
                            cv2.rectangle(viz_frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 1)
                            cv2.putText(viz_frame, f"IS {detected_color}", (int(box[0]), int(box[1])-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                            continue
                    
                    # E. 校验通过
                    match_score = r['score'] * (1.0 / (abs(actual_ratio - target_ratio) + 0.1))
                    valid_candidates.append({'mask': mask, 'score': match_score, 'box': box})
                    cv2.rectangle(viz_frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 255), 2)

                # --- 3. 重试判定 ---
                if not valid_candidates:
                    print(f" ❌ [未发现匹配目标] 可能是目标被遮挡或在黑名单中。刷新中...")
                    cv2.imshow("Detection Logic", viz_frame)
                    cv2.waitKey(1500)
                    continue # 跳回 while True 开头重新拍照检测

                # 选出最高分 Winner
                winner = max(valid_candidates, key=lambda x: x['score'])
                cv2.rectangle(viz_frame, (int(winner['box'][0]), int(winner['box'][1])), 
                              (int(winner['box'][2]), int(winner['box'][3])), (0, 255, 0), 4)
                cv2.imshow("Detection Logic", viz_frame)
                cv2.waitKey(1000) 

                # --- 4. 6D 姿态追踪 ---
                print("📐 正在精炼 6D 位姿...")
                pose_samples = []
                start_track = time.time()
                while (time.time() - start_track) < 4.0:
                    f = self.pipeline.wait_for_frames(); a = self.align.process(f)
                    rgb = np.asanyarray(a.get_color_frame().get_data())
                    dep = np.asanyarray(a.get_depth_frame().get_data()).astype(np.float32) / 1000.0
                    
                    T_curr = self.pose_est.estimator.register(K=self.K_MATRIX, rgb=rgb, depth=dep, ob_mask=winner['mask'], iteration=20)
                    if T_curr is not None:
                        pose_samples.append(T_curr)
                        self.visualize_result(rgb, T_curr)
                    cv2.imshow("6D Pose Tracking", rgb)
                    if cv2.waitKey(1) & 0xFF == ord('q'): break

                # --- 5. 发送结果 ---
                if pose_samples:
                    print(f"✅ 任务 {name} 位姿解算成功！")
                    self.send_to_robot(name, pose_samples[-1], task)
                    task_success = True # 成功解算，跳出 while True 循环进入下一个积木任务

    def visualize_result(self, image, T_cam_obj):
        l = 0.05
        pts_3d = np.float32([[0,0,0], [l,0,0], [0,l,0], [0,0,l]])
        pts_cam = (T_cam_obj[:3,:3] @ pts_3d.T).T + T_cam_obj[:3,3]
        p2d = []
        for p in pts_cam:
            u = int(self.K_MATRIX[0,0]*p[0]/p[2] + self.K_MATRIX[0,2])
            v = int(self.K_MATRIX[1,1]*p[1]/p[2] + self.K_MATRIX[1,2])
            p2d.append((u,v))
        cv2.line(image, p2d[0], p2d[1], (0,0,255), 3)
        cv2.line(image, p2d[0], p2d[2], (0,255,0), 2)
        cv2.line(image, p2d[0], p2d[3], (255,0,0), 2)

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        T_base_obj = self.T_base_camera @ T_cam_obj
        raw_yaw = np.degrees(np.arctan2(T_base_obj[1,0], T_base_obj[0,0]))
        refined_yaw = ((raw_yaw + 90) % 180) - 90
        spin = float(task_cfg.get('grasp_spin', 0))
        blueprint_yaw = float(task_cfg.get('blueprint_yaw', 0))
        
        brick_q = R.from_euler('xyz', [0,0,refined_yaw], degrees=True).as_quat().tolist()
        pick_q = R.from_euler('xyz', [180,0,refined_yaw + spin - 135], degrees=True).as_quat().tolist()
        place_q = R.from_euler('xyz', [180,0,blueprint_yaw + spin - 45], degrees=True).as_quat().tolist()

        data = {
            'name': name,
            'brick_info': {'pos': T_base_obj[:3,3].tolist(), 'orientation': brick_q},
            'robot_pick': {'orientation': pick_q},
            'place': {'pos': (ASSEMBLY_CENTER_BASE + np.array(task_cfg['place']['pos'])).tolist(), 'orientation': place_q}
        }
        with open(RESULT_FILE, 'w') as f:
            yaml.dump(data, f)

    def get_mesh_path(self, task_name):
        kw = "4x2" if ("2x4" in task_name or "4x2" in task_name) else "2x2"
        for f in os.listdir(MESH_DIR):
            if kw in f and f.endswith(".stl"): return os.path.join(MESH_DIR, f)
        return ""

if __name__ == "__main__":
    node = RobotVisionNode()
    node.run()