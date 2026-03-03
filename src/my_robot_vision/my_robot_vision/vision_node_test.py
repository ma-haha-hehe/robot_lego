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

# ================= 1. Configuration =================
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

# ================= 2. 6D Pose Class =================
class LegoPoseEstimator:
    def __init__(self, scorer, refiner):
        self.scorer, self.refiner = scorer, refiner

    def update_mesh(self, mesh_path):
        """ Load STL and center it for precise 6D tracking """
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

# ================= 3. Main Real-Camera Node =================
class RealCameraVisionNode:
    def __init__(self):
        # Load Calibration
        with open(CAMERA_PARAMS_YAML, 'r') as f:
            params = yaml.safe_load(f)
        self.T_base_camera = np.array(params['extrinsic_matrix']).reshape(4, 4)
        
        with open(TASKS_YAML, 'r') as f:
            self.task_list = yaml.safe_load(f)['tasks']

        self.init_realsense()
        
        # Init AI Models
        print("🤖 Loading AI models to GPU...")
        self.detector = pipeline(model="IDEA-Research/grounding-dino-tiny", 
                                 task="zero-shot-object-detection", device="cuda")
        if SAM_AVAILABLE:
            sam = sam_model_registry["vit_h"](checkpoint="/FoundationPose/weights/sam_vit_h_4b8939.pth").to("cuda")
            self.sam_predictor = SamPredictor(sam)

        self.pose_est = LegoPoseEstimator(ScorePredictor(), PoseRefinePredictor())

    def init_realsense(self):
        """ Start RealSense pipeline with aligned Color and Depth """
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        profile = self.pipeline.start(config)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K_MATRIX = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        self.align = rs.align(rs.stream.color)
        print("📸 RealSense Camera Connected & Aligned.")

    def run(self):
        for task in self.task_list:
            name = task['name']
            print(f"\n🎯 Target: {name}")
            self.pose_est.update_mesh(self.get_mesh_path(name))

            # --- STAGE 1: Real-time Scanning ---
            best_det, max_score, best_img = None, -1, None
            obs_start = time.time()
            while (time.time() - obs_start) < 5.0:
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                color_frame = aligned.get_color_frame()
                if not color_frame: continue
                
                img = np.asanyarray(color_frame.get_data())
                img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                
                results = self.detector(img_pil, candidate_labels=[name, "lego"], threshold=0.3)
                for r in results:
                    if r['score'] > max_score:
                        max_score, best_img = r['score'], img.copy()
                        best_det = BoundingBox(**r['box'])
                
                cv2.imshow("Live Scan", img)
                cv2.waitKey(1)

            if not best_det: continue

            # --- STAGE 2: SAM Segmentation ---
            self.sam_predictor.set_image(cv2.cvtColor(best_img, cv2.COLOR_BGR2RGB))
            masks, _, _ = self.sam_predictor.predict(box=np.array(best_det.xyxy), multimask_output=False)
            mask = masks[0]

            # --- STAGE 3: 6D Pose Refinement ---
            refine_start = time.time()
            while (time.time() - refine_start) < 3.0:
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                img = np.asanyarray(aligned.get_color_frame().get_data())
                depth = np.asanyarray(aligned.get_depth_frame().get_data()).astype(np.float32) / 1000.0
                
                T_curr = self.pose_est.estimator.register(K=self.K_MATRIX, rgb=img, depth=depth, ob_mask=mask, iteration=30)
                if T_curr is not None:
                    vis_img = self.visualize(img.copy(), T_curr)
                    cv2.imshow("6D Tracking", vis_img)
                cv2.waitKey(1)

            if T_curr is not None:
                self.send_to_robot(name, T_curr, task)

    def visualize(self, img, T):
        """ Draw 3D Axes on the real image """
        l = 0.05
        pts_3d = np.float32([[0,0,0], [l,0,0], [0,l,0], [0,0,l]])
        pts_cam = (T[:3,:3] @ pts_3d.T).T + T[:3,3]
        pts_2d = []
        for p in pts_cam:
            u = int(self.K_MATRIX[0,0] * p[0]/p[2] + self.K_MATRIX[0,2])
            v = int(self.K_MATRIX[1,1] * p[1]/p[2] + self.K_MATRIX[1,2])
            pts_2d.append((u, v))
        cv2.line(img, pts_2d[0], pts_2d[1], (0,0,255), 2)
        cv2.line(img, pts_2d[0], pts_2d[2], (0,255,0), 2)
        cv2.line(img, pts_2d[0], pts_2d[3], (255,0,0), 2)
        return img

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        T_base = self.T_base_camera @ T_cam_obj
        r = R.from_matrix(T_base[:3,:3])
        # Force gripper orientation: Pointing down (180 deg roll)
        final_r = R.from_euler('xyz', [np.pi, 0, r.as_euler('zyx')[0]])
        
        data = {
            'name': name,
            'pick': {'pos': T_base[:3,3].tolist(), 'orientation': final_r.as_quat().tolist()},
            'place': {
                'pos': (ASSEMBLY_CENTER_BASE + np.array(task_cfg['place']['pos'])).tolist(),
                'orientation': final_r.as_quat().tolist()
            }
        }
        with open(RESULT_FILE, 'w') as f:
            yaml.dump(data, f)
        print(f"✅ Pose sent for {name}. Waiting for Robot...")
        while os.path.exists(RESULT_FILE): time.sleep(0.5)

    def get_mesh_path(self, name):
        keyword = "4x2" if "4x2" in name else "2x2"
        return [os.path.join(MESH_DIR, f) for f in os.listdir(MESH_DIR) if keyword in f][0]

if __name__ == "__main__":
    node = RealCameraVisionNode()
    try:
        node.run()
    finally:
        cv2.destroyAllWindows()