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
TASKS_YAML = "/vision_code/tasks.yaml"        # 任务列表
CAMERA_PARAMS_YAML = "/vision_code/camera_params.yaml" # 相机外参矩阵
MESH_DIR = "/FoundationPose/meshes"           # STL模型存放处
ASSEMBLY_CENTER_BASE = np.array([0.25,0, 0.0]) # 组装区基准中心

if FP_REPO not in sys.path:
    sys.path.append(FP_REPO)
    sys.path.append(os.path.join(FP_REPO, "root"))

from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor

# 尝试加载 SAM 组件
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
        """ 加载并校准 3D 模型，确保坐标原点在几何中心 """
        self.mesh = trimesh.load(mesh_path)
        if np.linalg.norm(self.mesh.extents) > 0.1: 
            self.mesh.apply_scale(0.001)
        
        # 居中处理，消除模型自带的偏移
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
        # 加载相机外参 (Camera to Base)
        with open(CAMERA_PARAMS_YAML, 'r') as f:
            params = yaml.safe_load(f)
        self.T_base_camera = np.array(params['extrinsic_matrix']).reshape(4, 4)
        
        with open(TASKS_YAML, 'r') as f:
            self.task_list = yaml.safe_load(f)['tasks']

        self.init_realsense()
        
        # 初始化 GroundingDINO
        self.detector = pipeline(model="IDEA-Research/grounding-dino-tiny", task="zero-shot-object-detection", device="cuda")
        
        # 初始化 SAM (Segment Anything)
        if SAM_AVAILABLE:
            sam = sam_model_registry["vit_h"](checkpoint="/FoundationPose/weights/sam_vit_h_4b8939.pth").to("cuda")
            self.sam_predictor = SamPredictor(sam)

        # 初始化 FoundationPose
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
            print(f"\n" + "="*60)
            print(f"🎯 当前识别目标: {name}")
            
            mesh_path = self.get_mesh_path(name)
            self.pose_est.update_mesh(mesh_path)

            # --- 阶段 1: GroundingDINO 可视化 ---
            print(f"👀 阶段 1: 正在观察场景 (8.0s)...")
            best_det = None
            max_score = -1
            best_img_bgr = None
            obs_start = time.time()
            
            while (time.time() - obs_start) < 8.0:
                frames = self.pipeline.wait_for_frames()
                img = np.asanyarray(self.align.process(frames).get_color_frame().get_data())
                img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                
                results = self.detector(img_pil, candidate_labels=[name, "lego block."], threshold=0.3)
                
                # 创建检测可视化画布
                dino_vis = img.copy()
                for r in results:
                    if r['score'] > max_score:
                        max_score = r['score']
                        best_det = DetectionResult(r['score'], r['label'], BoundingBox(**r['box']))
                        best_img_bgr = img.copy()
                    
                    # 绘制所有候选框
                    b = r['box']
                    cv2.rectangle(dino_vis, (b['xmin'], b['ymin']), (b['xmax'], b['ymax']), (0, 255, 0), 2)
                    cv2.putText(dino_vis, f"{r['label']}: {r['score']:.2f}", (b['xmin'], b['ymin']-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                cv2.putText(dino_vis, f"Scanning... Time Left: {8-(time.time()-obs_start):.1f}s", (20, 35), 0, 0.7, (0, 255, 255), 2)
                cv2.imshow("Step 1: GroundingDINO Detection", dino_vis)
                cv2.waitKey(1)

            if not best_det:
                print("❌ 无法识别物体，跳过当前任务。"); continue

            # --- 阶段 2: SAM 像素级分割可视化 ---
            print(f"✂️ 阶段 2: SAM 精确扣图...")
            if SAM_AVAILABLE:
                self.sam_predictor.set_image(cv2.cvtColor(best_img_bgr, cv2.COLOR_BGR2RGB))
                masks, _, _ = self.sam_predictor.predict(box=np.array(best_det.box.xyxy), multimask_output=False)
                refined_mask = masks[0]
            else:
                refined_mask = np.zeros(best_img_bgr.shape[:2], dtype=bool)
                b = best_det.box
                refined_mask[b.ymin:b.ymax, b.xmin:b.xmax] = True

            # 创建 SAM 遮罩可视化
            sam_vis = best_img_bgr.copy()
            # 将遮罩区域染色（蓝色叠加层）
            sam_vis[refined_mask] = sam_vis[refined_mask] * 0.5 + np.array([255, 0, 0]) * 0.5
            cv2.imshow("Step 2: SAM Mask Visualization", sam_vis.astype(np.uint8))
            cv2.waitKey(500) # 暂停半秒展示分割结果

            # --- 阶段 3: FoundationPose 位姿可视化 ---
            print(f"💎 阶段 3: 位姿解算与精炼 (4.0s)...")
            pose_samples = []
            refine_start = time.time()
            
            while (time.time() - refine_start) < 4.0:
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                img = np.asanyarray(aligned.get_color_frame().get_data())
                depth_m = np.asanyarray(aligned.get_depth_frame().get_data()).astype(np.float32) / 1000.0
                
                T_curr = self.pose_est.estimator.register(
                    K=self.K_MATRIX, rgb=img, depth=depth_m, 
                    ob_mask=refined_mask, iteration=30
                )
                
                if T_curr is not None:
                    pose_samples.append(T_curr)
                    # visualize_result 内部已包含绘制 Base 坐标和坐标轴的逻辑
                    pose_vis = self.visualize_result(img.copy(), T_curr)
                    cv2.imshow("Step 3: FoundationPose Refinement", pose_vis)
                
                cv2.waitKey(1)

            if len(pose_samples) > 0:
                self.send_to_robot(name, pose_samples[-1], task)
            else:
                print("❌ 未捕获到有效的位姿样本。")

    def visualize_result(self, image, T_cam_obj):
        """ 综合可视化：绘制姿态轴和 Base 空间坐标 """
        # 1. 绘制 3D 坐标轴
        length = 0.05
        axis_pts_3d = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]])
        R_m, t_vec = T_cam_obj[:3, :3], T_cam_obj[:3, 3]
        pts_cam = (R_m @ axis_pts_3d.T).T + t_vec
        
        pts_2d = []
        for p in pts_cam:
            u = int(self.K_MATRIX[0,0] * p[0]/p[2] + self.K_MATRIX[0,2])
            v = int(self.K_MATRIX[1,1] * p[1]/p[2] + self.K_MATRIX[1,2])
            pts_2d.append((u, v))
        
        cv2.line(image, pts_2d[0], pts_2d[1], (0,0,255), 2) # X-Red
        cv2.line(image, pts_2d[0], pts_2d[2], (0,255,0), 2) # Y-Green
        cv2.line(image, pts_2d[0], pts_2d[3], (255,0,0), 2) # Z-Blue

        # 2. 计算并绘制物体的 Base 坐标
        # P_base = T_base_camera @ P_camera
        P_base = self.T_base_camera @ np.append(t_vec, 1.0)
        bx, by, bz = P_base[0], P_base[1], P_base[2]

        # 在中心位置画个圆点
        cv2.circle(image, pts_2d[0], 5, (255, 255, 0), -1)
        
        # 在圆点旁边标注 Base 坐标信息
        coord_txt = f"Base: [{bx:.3f}, {by:.3f}, {bz:.3f}]"
        cv2.putText(image, coord_txt, (pts_2d[0][0] + 10, pts_2d[0][1] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return image

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        """ 坐标最终变换并发送信号给 C++ """
        T_base_pick = self.T_base_camera @ T_cam_obj

        # 提取欧拉角 euler[0] yam
        r_pick = R.from_matrix(T_base_pick[:3, :3])
        euler_pick = r_pick.as_euler('zyx', degrees=False)
        detected_yaw = euler_pick[0]

        # 3. 构造放置姿态：Roll=pi, Pitch=0, Yaw=识别到的值
        # 这里的 pi (180度) 确保夹爪垂直向下
        r_place = R.from_euler('xyz', [np.pi, 0, detected_yaw], degrees=False)
        pick_q = r_place.as_quat()
        
        # 计算放置位姿
        place_pos = ASSEMBLY_CENTER_BASE + np.array(task_cfg['place']['pos'])
        
        data = {
        'name': name,
        'pick': {
            'pos': T_base_pick[:3, 3].tolist(), 
            'orientation': r_pick.as_quat().tolist() # 抓取时完全匹配识别姿态
        },
        'place': {
            'pos': place_pos.tolist(), 
            'orientation': place_q.tolist() # 放置时垂直向下但保留旋转
        }
        }
        
        with open(RESULT_FILE, 'w') as f:
            yaml.dump(data, f)
        print(f"✅ 已写入信号文件。等待机械臂执行并清理...")
        
        while os.path.exists(RESULT_FILE):
            time.sleep(0.5)

    def get_mesh_path(self, task_name):
        keyword = "4x2" if ("2x4" in task_name or "4x2" in task_name) else "2x2"
        for f in os.listdir(MESH_DIR):
            if keyword in f and f.endswith(".stl"):
                return os.path.join(MESH_DIR, f)
        return ""

if __name__ == "__main__":
    node = RobotVisionNode()
    try:
        node.run()
    finally:
        cv2.destroyAllWindows()