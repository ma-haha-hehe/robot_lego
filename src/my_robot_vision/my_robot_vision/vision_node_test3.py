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
TASKS_YAML = "/vision_code/tasksh_true.yaml"#t and bigt
CAMERA_PARAMS_YAML = "/vision_code/camera_params.yaml"
MESH_DIR = "/FoundationPose/meshes"
ASSEMBLY_CENTER_BASE = np.array([0.35, 0.2, 0.025])

# 🔥 仿真修正：删除 -45度补偿，设为 0.0
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
        print("get new view 扫清相机缓存，获取新鲜实时画面...")
        for _ in range(30):
            self.pipeline.wait_for_frames()

    def run(self):
        for task in self.task_list:
            name = task['name']
            while os.path.exists(RESULT_FILE):
                print("⏳ 等待旧任务完成信号清除...")
                time.sleep(1.0)

            self.clear_buffer()
            print(f"\n🎯 [新任务] 目标: {name}")
            mesh_path = self.get_mesh_path(name)
            self.pose_est.update_mesh(mesh_path)

            # --- 阶段 1: 扫描所有可能的候选者 (GroundingDINO) ---
            print("👁️ 阶段 1: 正在扫描桌面所有目标...")
            frames = self.pipeline.wait_for_frames()
            img_bgr = np.asanyarray(self.align.process(frames).get_color_frame().get_data())
            img_pil = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
            
            # 获取所有候选结果
            results = self.detector(img_pil, candidate_labels=[name, "lego block"], threshold=0.2)
            
            candidates = []
            viz_all = img_bgr.copy() # 用于展示所有选手的初始得分

            for r in results:
                box = BoundingBox(**r['box'])
                # 在初始画面画出所有检测到的物体 (蓝色)
                cv2.rectangle(viz_all, (box.xmin, box.ymin), (box.xmax, box.ymax), (255, 0, 0), 2)
                cv2.putText(viz_all, f"Det: {r['score']:.2f}", (box.xmin, box.ymin-5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                # --- 阶段 2: 对每个候选者运行 SAM 并计算真实几何比例 ---
                if SAM_AVAILABLE:
                    self.sam_predictor.set_image(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
                    masks, _, _ = self.sam_predictor.predict(box=np.array(box.xyxy), multimask_output=False)
                    mask = masks[0]
                    
                    # 使用最小外接矩形 (OBB) 计算物理比例，无视旋转
                    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if not contours: continue
                    cnt = max(contours, key=cv2.contourArea)
                    rect = cv2.minAreaRect(cnt)
                    (w, h) = rect[1]
                    
                    actual_w = max(w, h)
                    actual_h = min(w, h)
                    if actual_h == 0: continue
                    aspect_ratio = actual_w / actual_h 
                    
                    # 评分逻辑：2x4 目标比例 2.0，2x2 目标比例 1.0
                    target_ratio = 2.0 if ("4x2" in name or "2x4" in name) else 1.0
                    geo_score = 1.0 / (abs(aspect_ratio - target_ratio) + 0.1)
                    
                    # 综合得分 = 语义分 * 几何分
                    total_score = geo_score * r['score']
                    candidates.append({'det': r, 'mask': mask, 'total_score': total_score})

            if not candidates:
                print("❌ 未发现符合条件的积木。")
                continue

            # --- 决策可视化: 选出“冠军”并展示 ---
            best_candidate = max(candidates, key=lambda x: x['total_score'])
            box_win = BoundingBox(**best_candidate['det']['box'])
            
            # 高亮胜出者 (黄色)
            cv2.rectangle(viz_all, (box_win.xmin, box_win.ymin), (box_win.xmax, box_win.ymax), (0, 255, 255), 4)
            cv2.putText(viz_all, f"WINNER: {name} ({best_candidate['total_score']:.2f})", 
                        (box_win.xmin, box_win.ymin-25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # 在胜出者上叠加绿色掩码
            viz_all[best_candidate['mask']] = viz_all[best_candidate['mask']] * 0.6 + np.array([0, 255, 0], dtype=np.uint8) * 0.4
            
            cv2.imshow("Vision - Ranking & Selection", viz_all)
            print(f"✅ 胜出目标: {name}，展示决策中...")
            cv2.waitKey(2000) # 🔥 停留 2 秒展示“选妃”结果

            # --- 阶段 3: 6D 姿态精炼 (FoundationPose) ---
            print("📏 阶段 3: 正在精炼 6D 姿态...")
            pose_samples = []; refine_start = time.time()
            final_viz = None

            while (time.time() - refine_start) < 4.0:
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                img = np.asanyarray(aligned.get_color_frame().get_data())
                depth_m = np.asanyarray(aligned.get_depth_frame().get_data()).astype(np.float32) / 1000.0
                
                # 🔥 关键修复：使用 best_candidate 的 mask
                T_curr = self.pose_est.estimator.register(
                    K=self.K_MATRIX, rgb=img, depth=depth_m, 
                    ob_mask=best_candidate['mask'], iteration=30
                )
                
                if T_curr is not None:
                    pose_samples.append(T_curr)
                    self.visualize_result(img, T_curr)
                    final_viz = img.copy()
                
                cv2.imshow("Vision - 6D Pose", img)
                cv2.waitKey(1)

            if pose_samples:
                if final_viz is not None:
                    cv2.putText(final_viz, "READY TO GRASP", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.imshow("Vision - 6D Pose", final_viz)
                
                print(f"🚀 流程结束，发送数据。")
                cv2.waitKey(2000) # 🔥 最终姿态停留 2 秒
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
        # 1. 基础坐标转换
        T_base_obj = self.T_base_camera @ T_cam_obj
        pos = T_base_obj[:3, 3]
        R_base = T_base_obj[:3, :3]
        # 2. 提取并固定 Yaw 角 (强制平放)
        raw_yaw_deg = np.degrees(np.arctan2(R_base[1, 0], R_base[0, 0]))
        # 归一化到 [-90, 90]，利用长方形对称性
        refined_yaw = ((raw_yaw_deg + 90) % 180) - 90
        # 3. 读取策略与蓝图
        strategy_spin = float(task_cfg.get('grasp_spin', 0)) # 0或90
        blueprint_yaw = float(task_cfg.get('blueprint_yaw', 0)) # 0或90
        # --- [A] 积木生成的姿态 ---
        brick_q = R.from_euler('xyz', [0, 0, refined_yaw], degrees=True).as_quat().tolist()

        # --- [B] 抓取姿态 ---
        final_pick_yaw = refined_yaw + strategy_spin - 135
        pick_q = R.from_euler('xyz', [180, 0, final_pick_yaw], degrees=True).as_quat().tolist()

        # --- [C] 放置姿态 ---
        final_place_yaw = blueprint_yaw + strategy_spin - 45.0
        place_q = R.from_euler('xyz', [180, 0, final_place_yaw], degrees=True).as_quat().tolist()

        # 4. 整合输出
        data = {
            'name': name,
            'brick_info': {'pos': T_base_obj[:3,3].tolist(), 'orientation': brick_q},
            'robot_pick': {'orientation': pick_q},
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