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
RESULT_FILE = "/shared_data/active_task.yaml"
TASKS_YAML = "/vision_code/tasks.yaml"
CAMERA_PARAMS_YAML = "/vision_code/camera_params.yaml"
MESH_DIR = "/FoundationPose/meshes"
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

            self.sam_predictor.set_image(cv2.cvtColor(best_img_bgr, cv2.COLOR_BGR2RGB))
            masks, _, _ = self.sam_predictor.predict(box=np.array(best_det.box.xyxy), multimask_output=False)
            refined_mask = masks[0]

            refine_start = time.time()
            T_curr = None
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
        length = 0.05
        axis_pts_3d = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]])
        pts_cam = (T_cam_obj[:3, :3] @ axis_pts_3d.T).T + T_cam_obj[:3, 3]
        pts_2d = []
        for p in pts_cam:
            u = int(self.K_MATRIX[0,0] * p[0]/p[2] + self.K_MATRIX[0,2])
            v = int(self.K_MATRIX[1,1] * p[1]/p[2] + self.K_MATRIX[1,2])
            pts_2d.append((u, v))
        
        cv2.line(image, pts_2d[0], pts_2d[1], (0,0,255), 3) 
        cv2.line(image, pts_2d[0], pts_2d[2], (0,255,0), 2) 
        cv2.line(image, pts_2d[0], pts_2d[3], (255,0,0), 2) 

        T_base_vision = self.T_base_camera @ T_cam_obj
        R_base_obj = T_base_vision[:3, :3]
        raw_yaw_deg = np.degrees(np.arctan2(R_base_obj[1, 0], R_base_obj[0, 0]))
        stable_yaw = ((raw_yaw_deg + 90) % 180) - 90
        
        # 画面显示逻辑同步：显示最终会被下发的二值化角度
        output_yaw = 90.0 if abs(stable_yaw) < 45 else 0.0
        
        txt = f"Detected:{stable_yaw:.1f} | Output:{output_yaw:.0f}deg"
        cv2.putText(image, txt, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        """ 严格离散化角度输出逻辑 """
        T_base_vision = self.T_base_camera @ T_cam_obj
        R_base_obj = T_base_vision[:3, :3]

        # 1. 提取原始角度并归一化到 [-90, 90]
        raw_yaw_deg = np.degrees(np.arctan2(R_base_obj[1, 0], R_base_obj[0, 0]))
        stable_yaw_deg = ((raw_yaw_deg + 90) % 180) - 90

        # 2. 核心逻辑：离散化二值判断
        # 如果长轴(红轴)平行于机器人X轴(即接近0度), 输出 90
        # 如果垂直于机器人X轴(即接近90或-90度), 输出 0
        if abs(stable_yaw_deg) < 45:
            final_pick_yaw_deg = 90.0
            status_str = "平行 (Parallel) -> 输出 90°"
        else:
            final_pick_yaw_deg = 0.0
            status_str = "垂直 (Perpendicular) -> 输出 0°"

        final_pick_yaw_rad = np.radians(final_pick_yaw_deg)

        # 3. 生成 Pick 四元数 (锁定垂直向下)
        pick_quat = R.from_euler('xyz', [np.pi, 0, final_pick_yaw_rad]).as_quat().tolist()

        # 4. 计算 Place 位姿 (遵循任务表设计)
        rel_pos = np.array(task_cfg['place']['pos'])
        final_place_pos = (ASSEMBLY_CENTER_BASE + rel_pos).tolist()
        
        # 放置角度同样建议离散化或遵循蓝图
        blueprint_orientation = task_cfg['place']['orientation']
        blueprint_yaw = blueprint_orientation[2] if isinstance(blueprint_orientation, list) else blueprint_orientation
        place_quat = R.from_euler('xyz', [np.pi, 0, np.radians(blueprint_yaw)]).as_quat().tolist()

        data = {
            'name': name,
            'pick': {
                'pos': T_base_vision[:3, 3].tolist(), 
                'orientation': pick_quat
            },
            'place': {
                'pos': final_place_pos, 
                'orientation': place_quat
            }
        }
        
        with open(RESULT_FILE, 'w') as f:
            yaml.dump(data, f)
        
        print(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print(f"✅ 指令下发: {name}")
        print(f"🌀 检测角度: {stable_yaw_deg:.1f}° | {status_str}")
        print(f"📍 Pick Pos: {data['pick']['pos']}")
        print(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        
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