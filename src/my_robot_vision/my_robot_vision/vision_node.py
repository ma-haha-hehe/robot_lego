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
from scipy.spatial.transform import Rotation as R

# ================= 1. 路径与环境配置 =================
FP_REPO = "/FoundationPose"
RESULT_FILE = "/shared_data/active_task.yaml"  # 与 C++ 节点通信的 YAML
TASKS_YAML = "/vision_code/tasks.yaml"         # 任务列表
CAMERA_PARAMS_YAML = "/vision_code/camera_params.yaml" 
MESH_DIR = "/FoundationPose/meshes"            # STL 模型路径
ASSEMBLY_CENTER_BASE = np.array([0.25, 0, 0.0]) # 放置区的物理中心

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

# ================= 2. 姿态估计核心类 =================
class LegoPoseEstimator:
    def __init__(self, scorer, refiner):
        self.scorer = scorer
        self.refiner = refiner

    def update_mesh(self, mesh_path):
        """ 加载积木模型并初始化 FoundationPose """
        self.mesh = trimesh.load(mesh_path)
        if np.linalg.norm(self.mesh.extents) > 0.1: 
            self.mesh.apply_scale(0.001) # 毫米转米
        self.mesh.vertices -= self.mesh.bounds.mean(axis=0) # 居中
        model_pts, _ = trimesh.sample.sample_surface(self.mesh, 2048)
        self.model_pts = torch.from_numpy(model_pts.astype(np.float32)).cuda()
        self.estimator = FoundationPose(
            model_pts=self.model_pts, model_normals=None, mesh=self.mesh,
            scorer=self.scorer, refiner=self.refiner
        )

# ================= 3. 视觉处理主节点 =================
class RobotVisionNode:
    def __init__(self):
        # 加载相机外参 (Camera -> Base)
        with open(CAMERA_PARAMS_YAML, 'r') as f:
            params = yaml.safe_load(f)
        self.T_base_camera = np.array(params['extrinsic_matrix']).reshape(4, 4)
        
        # 加载任务清单
        with open(TASKS_YAML, 'r') as f:
            data = yaml.safe_load(f)
            self.task_list = data.get('tasksh', data.get('tasks', []))

        self.init_realsense()
        from transformers import pipeline
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
            print(f"\n🎯 识别目标: {name}")
            self.pose_est.update_mesh(self.get_mesh_path(name))

            # 阶段 1: 持续检测，锁定最佳目标
            obs_start = time.time()
            best_det = None
            max_score = -1
            while (time.time() - obs_start) < 5.0:
                frames = self.pipeline.wait_for_frames()
                img = np.asanyarray(self.align.process(frames).get_color_frame().get_data())
                img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                results = self.detector(img_pil, candidate_labels=[name, "lego"], threshold=0.3)
                for r in results:
                    if r['score'] > max_score:
                        max_score = r['score']
                        best_det = DetectionResult(r['score'], r['label'], BoundingBox(**r['box']))
                        best_img_bgr = img.copy()
                cv2.imshow("Vision", img); cv2.waitKey(1)

            if not best_det: continue

            # 阶段 2: 掩码生成
            self.sam_predictor.set_image(cv2.cvtColor(best_img_bgr, cv2.COLOR_BGR2RGB))
            masks, _, _ = self.sam_predictor.predict(box=np.array(best_det.box.xyxy), multimask_output=False)
            refined_mask = masks[0]

            # 阶段 3: 姿态追踪与发送
            refine_start = time.time()
            while (time.time() - refine_start) < 4.0:
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                img = np.asanyarray(aligned.get_color_frame().get_data())
                depth_m = np.asanyarray(aligned.get_depth_frame().get_data()).astype(np.float32) / 1000.0
                T_curr = self.pose_est.estimator.register(K=self.K_MATRIX, rgb=img, depth=depth_m, ob_mask=refined_mask, iteration=30)
                if T_curr is not None:
                    self.visualize_result(img, T_curr)
                cv2.imshow("Vision", img); cv2.waitKey(1)

            if T_curr is not None:
                self.send_to_robot(name, T_curr, task)

    def visualize_result(self, image, T_cam_obj):
        """ 绘制坐标轴：红线=X轴, 绿线=Y轴, 蓝线=Z轴 """
        length = 0.05
        # 定义 3D 轴端点
        axis_pts_3d = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]])
        
        # 变换与投影
        pts_cam = (T_cam_obj[:3, :3] @ axis_pts_3d.T).T + T_cam_obj[:3, 3]
        pts_2d = []
        for p in pts_cam:
            u = int(self.K_MATRIX[0,0] * p[0]/p[2] + self.K_MATRIX[0,2])
            v = int(self.K_MATRIX[1,1] * p[1]/p[2] + self.K_MATRIX[1,2])
            pts_2d.append((u, v))
        
        # 绘图：pts_2d[1] 对应 axis_pts_3d[1] (X轴)
        cv2.line(image, pts_2d[0], pts_2d[1], (0,0,255), 3) # X-红
        cv2.line(image, pts_2d[0], pts_2d[2], (0,255,0), 2) # Y-绿
        cv2.line(image, pts_2d[0], pts_2d[3], (255,0,0), 2) # Z-蓝

        # 计算并显示 Yaw (严格基于红轴在 Base 系下的角度)
        T_base_vision = self.T_base_camera @ T_cam_obj
        R_base_obj = T_base_vision[:3, :3]
        # atan2(红轴Y分量, 红轴X分量)
        raw_yaw_deg = np.degrees(np.arctan2(R_base_obj[1, 0], R_base_obj[0, 0]))
        stable_yaw = ((raw_yaw_deg + 90) % 180) - 90
        
        txt = f"Base X:{T_base_vision[0,3]:.3f} Y:{T_base_vision[1,3]:.3f} Yaw:{stable_yaw:.1f}deg"
        cv2.putText(image, txt, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        """ 严格计算并发送指令 """
        T_base_vision = self.T_base_camera @ T_cam_obj
        R_base_obj = T_base_vision[:3, :3]

        # 1. 严格计算红轴(长边)偏角
        raw_yaw_deg = np.degrees(np.arctan2(R_base_obj[1, 0], R_base_obj[0, 0]))
        # 针对 4x2 对称性进行 180 度去跳变归一化
        stable_yaw_deg = ((raw_yaw_deg + 90) % 180) - 90
        final_yaw_rad = np.radians(stable_yaw_deg)

        # 2. 合成 Pick 姿态 (锁定垂直 + 现场角度)
        pick_quat = R.from_euler('xyz', [np.pi, 0, final_yaw_rad]).as_quat().tolist()

        # 3. 计算 Place 姿态 (物理基准 + 蓝图数据)
        rel_pos = np.array(task_cfg['place']['pos'])
        final_place_pos = (ASSEMBLY_CENTER_BASE + rel_pos).tolist()
        blueprint_yaw = task_cfg['place']['orientation'][2] # 假设蓝图是 [R,P,Y]
        place_quat = R.from_euler('xyz', [np.pi, 0, np.radians(blueprint_yaw)]).as_quat().tolist()

        data = {
            'name': name,
            'pick': {'pos': T_base_vision[:3, 3].tolist(), 'orientation': pick_quat},
            'place': {'pos': final_place_pos, 'orientation': place_quat}
        }
        
        with open(RESULT_FILE, 'w') as f:
            yaml.dump(data, f)
        
        print(f"✅ 指令已下发！红轴角度: {stable_yaw_deg:.1f}°")
        # 握手：等待文件被 C++ 节点消费
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