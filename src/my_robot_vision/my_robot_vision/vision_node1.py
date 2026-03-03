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
ROBOT_READY_YAW_OFFSET = 45.0 
# ================= 1. 路径与环境配置 =================
FP_REPO = "/FoundationPose"
RESULT_FILE = "/shared_data/active_task.yaml"  # 发送给 C++ 节点的信号文件
TASKS_YAML = "/vision_code/tasksh.yaml"        # 存储由 planner 生成的任务列表
CAMERA_PARAMS_YAML = "/vision_code/camera_params.yaml" 
MESH_DIR = "/FoundationPose/meshes"            # 存放积木 STL 模型的目录
# 组装区基准坐标 (Base系)：用于 Place 阶段的全局坐标计算
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
        """ 动态加载不同积木的 3D 模型 """
        self.mesh = trimesh.load(mesh_path)
        # 统一单位为米 (m)
        if np.linalg.norm(self.mesh.extents) > 0.1: 
            self.mesh.apply_scale(0.001)
        # 将模型中心移至原点
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
        # 加载相机外参 (Camera 到 Robot Base 的变换矩阵)
        with open(CAMERA_PARAMS_YAML, 'r') as f:
            params = yaml.safe_load(f)
        self.T_base_camera = np.array(params['extrinsic_matrix']).reshape(4, 4)
        
        # 加载待执行的任务清单
        with open(TASKS_YAML, 'r') as f:
            data = yaml.safe_load(f)
            self.task_list = data.get('tasksh', data.get('tasks', []))

        self.init_realsense()
        # 初始化 GroundingDINO 用于零样本目标检测
        self.detector = pipeline(model="IDEA-Research/grounding-dino-tiny", task="zero-shot-object-detection", device="cuda")
        
        if SAM_AVAILABLE:
            sam = sam_model_registry["vit_h"](checkpoint="/FoundationPose/weights/sam_vit_h_4b8939.pth").to("cuda")
            self.sam_predictor = SamPredictor(sam)

        # 初始化 FoundationPose 核心估计器
        self.pose_est = LegoPoseEstimator(ScorePredictor(), PoseRefinePredictor())

    def init_realsense(self):
        """ 初始化 RealSense 深度相机并获取内参 """
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
            print(f"\n" + "="*60 + f"\n🎯 准备识别下一个目标: {name}")

            # --- 【关键修改：刷新相机缓冲区】 ---
            # 在开始新任务前，先连续读取并丢弃 30-50 帧，确保画面是当下的
            print("🧹 正在清理相机缓存...")
            for _ in range(30):
                self.pipeline.wait_for_frames() 
            
            mesh_path = self.get_mesh_path(name)
            self.pose_est.update_mesh(mesh_path)

            # --- 阶段 1: 目标检测 (此时读取到的就是刷新后的实时画面) ---
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

            if not best_det: 
                print(f"❌ 未能检测到物体: {name}")
                continue

            # --- 阶段 2: 语义分割 (Segmentation) ---
            if SAM_AVAILABLE:
                self.sam_predictor.set_image(cv2.cvtColor(best_img_bgr, cv2.COLOR_BGR2RGB))
                masks, _, _ = self.sam_predictor.predict(box=np.array(best_det.box.xyxy), multimask_output=False)
                refined_mask = masks[0]
            else:
                refined_mask = np.zeros(best_img_bgr.shape[:2], dtype=bool)
                refined_mask[best_det.box.ymin:best_det.box.ymax, best_det.box.xmin:best_det.box.xmax] = True

            # --- 阶段 3: 6D 姿态精炼 (Pose Refinement) ---
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
        """ 综合可视化：绘制姿态轴并显示修正后的实时数值 """
        length = 0.05
        # 定义 3D 坐标轴端点 (X-红, Y-绿, Z-蓝)
        axis_pts_3d = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]])
        
        # 1. 投影 3D 点到图像平面
        pts_cam = (T_cam_obj[:3, :3] @ axis_pts_3d.T).T + T_cam_obj[:3, 3]
        pts_2d = []
        for p in pts_cam:
            u = int(self.K_MATRIX[0,0] * p[0]/p[2] + self.K_MATRIX[0,2])
            v = int(self.K_MATRIX[1,1] * p[1]/p[2] + self.K_MATRIX[1,2])
            pts_2d.append((u, v))
        
        # 绘制轴线
        cv2.line(image, pts_2d[0], pts_2d[1], (0,0,255), 2) # X轴
        cv2.line(image, pts_2d[0], pts_2d[2], (0,255,0), 2) # Y轴
        cv2.line(image, pts_2d[0], pts_2d[3], (255,0,0), 2) # Z轴

        # 2. 核心：计算修正后的 Yaw 角度用于显示
        T_base_vision = self.T_base_camera @ T_cam_obj
        bx, by, bz = T_base_vision[:3, 3]
        r_vision = R.from_matrix(T_base_vision[:3, :3])
        raw_yaw = r_vision.as_euler('zyx', degrees=True)[0]
        
        corrected_yaw = ((raw_yaw + 90 + 90) % 180) - 90

        info_txt = f"X:{bx:.3f} Y:{by:.3f} Z:{bz:.3f} Yaw:{corrected_yaw:.1f}deg"
        cv2.putText(image, info_txt, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.circle(image, pts_2d[0], 4, (255, 255, 255), -1)

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        """ 连续角度输出 + 45度初始偏移补偿 """
        T_base_vision = self.T_base_camera @ T_cam_obj
        R_base_obj = T_base_vision[:3, :3]

        # 1. 提取检测到的连续偏航角
        raw_yaw_rad = np.arctan2(R_base_obj[1, 0], R_base_obj[0, 0])
        raw_yaw_deg = np.degrees(raw_yaw_rad)

        # 2. 180度对称归一化 (锁定在 [-90, 90])
        stable_yaw_deg = ((raw_yaw_deg + 90) % 180) - 90

        # 3. 核心：补偿 45 度初始偏移
        final_robot_yaw_deg = stable_yaw_deg - ROBOT_READY_YAW_OFFSET
        final_robot_yaw_deg = (final_robot_yaw_deg + 180) % 360 - 180
        
        # 4. 生成四元数 (Rx=180 翻转)
        pick_quat = R.from_euler('xyz', [np.pi, 0, np.radians(final_robot_yaw_deg)]).as_quat().tolist()

        data = {
            'name': name,
            'pick': {'pos': T_base_vision[:3, 3].tolist(), 'orientation': pick_quat},
            'place': {
                'pos': (ASSEMBLY_CENTER_BASE + np.array(task_cfg['place']['pos'])).tolist(),
                'orientation': R.from_euler('xyz', [np.pi, 0, np.radians(0 - ROBOT_READY_YAW_OFFSET)]).as_quat().tolist()
            }
        }
        
        with open(RESULT_FILE, 'w') as f:
            yaml.dump(data, f)
        print(f"✅ 指令生成！视觉 Yaw: {stable_yaw_deg:.1f}°, 补偿后发送: {final_robot_yaw_deg:.1f}°")
        
        # 握手机制：等待 C++ 节点消费文件
        while os.path.exists(RESULT_FILE):
            time.sleep(0.5)

    def get_mesh_path(self, task_name):
        keyword = "4x2" if "4x2" in task_name else "2x2"
        for f in os.listdir(MESH_DIR):
            if keyword in f and f.endswith(".stl"): return os.path.join(MESH_DIR, f)
        return ""
if __name__ == "__main__":
    node = RobotVisionNode()
    node.run()