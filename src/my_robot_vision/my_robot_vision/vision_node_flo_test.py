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
from transformers import AutoProcessor, AutoModelForCausalLM
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

# ================= 3. Florence-2 视觉节点 =================
class RobotVisionNode:
    def __init__(self):
        # 3.1 加载外参与任务
        with open(CAMERA_PARAMS_YAML, 'r') as f:
            params = yaml.safe_load(f)
        self.T_base_camera = np.array(params['extrinsic_matrix']).reshape(4, 4)
        
        with open(TASKS_YAML, 'r') as f:
            self.task_list = yaml.safe_load(f)['tasks']

        self.init_realsense()
        
        # 3.2 加载 Florence-2
        print(">>> 正在初始化 Florence-2 (microsoft/Florence-2-base)...")
        model_id = "microsoft/Florence-2-base"
        self.fl_model = AutoModelForCausalLM.from_pretrained(model_id, trust_remote_code=True).to("cuda").eval()
        self.fl_processor = AutoProcessor.from_pretrained(model_id, trust_remote_code=True)
        
        # 3.3 初始化 FoundationPose
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

    def run_florence(self, image_pil, text_prompt):
        """ 执行 Florence-2 语义接地任务 """
        task_prompt = '<CAPTION_TO_PHRASE_GROUNDING>'
        prompt = task_prompt + text_prompt
        inputs = self.fl_processor(text=prompt, images=image_pil, return_tensors="pt").to("cuda")
        generated_ids = self.fl_model.generate(
            input_ids=inputs["input_ids"], pixel_values=inputs["pixel_values"],
            max_new_tokens=1024, num_beams=3
        )
        generated_text = self.fl_processor.batch_decode(generated_ids, skip_special_tokens=False)[0]
        parsed_answer = self.fl_processor.post_process_generation(
            generated_text, task=task_prompt, image_size=(image_pil.width, image_pil.height)
        )
        return parsed_answer[task_prompt]

    def run(self):
        for task in self.task_list:
            name = task['name']
            print(f"\n" + "="*60 + f"\n🎯 目标确认: {name}")
            self.pose_est.update_mesh(self.get_mesh_path(name))
            
            best_det = None
            obs_start = time.time()
            
            # --- 阶段 1: Florence-2 语义观察 ---
            while (time.time() - obs_start) < 8.0:
                frames = self.pipeline.wait_for_frames()
                img = np.asanyarray(self.align.process(frames).get_color_frame().get_data())
                img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

                results = self.run_florence(img_pil, name)
                
                vis_img = img.copy()
                if results['bboxes']:
                    box = results['bboxes'][0]
                    label = results['labels'][0]
                    # 语义检查
                    if all(word in label.lower() for word in name.lower().split()):
                        best_det = DetectionResult(1.0, label, BoundingBox(int(box[0]), int(box[1]), int(box[2]), int(box[3])))
                        best_img_bgr = img.copy()
                        
                        # 可视化检测框
                        cv2.rectangle(vis_img, (best_det.box.xmin, best_det.box.ymin), 
                                      (best_det.box.xmax, best_det.box.ymax), (0, 255, 0), 2)
                        cv2.putText(vis_img, f"MATCH: {label}", (best_det.box.xmin, best_det.box.ymin-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                cv2.putText(vis_img, f"Florence-2 Scanning... {8-(time.time()-obs_start):.1f}s", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.imshow("Step 1: Florence-2 Semantic Detection", vis_img)
                if cv2.waitKey(1) & 0xFF == ord('q'): break

            if not best_det: continue

            # --- 阶段 2: Mask 准备 ---
            refined_mask = np.zeros(best_img_bgr.shape[:2], dtype=bool)
            b = best_det.box
            refined_mask[b.ymin:b.ymax, b.xmin:b.xmax] = True

            # --- 阶段 3: FoundationPose 姿态精炼 ---
            print(f"💎 正在精炼 {name} 的 6D 位姿...")
            pose_samples = []
            refine_start = time.time()
            while (time.time() - refine_start) < 4.0:
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                img = np.asanyarray(aligned.get_color_frame().get_data())
                depth_m = np.asanyarray(aligned.get_depth_frame().get_data()).astype(np.float32) / 1000.0
                
                T_curr = self.pose_est.estimator.register(
                    K=self.K_MATRIX, rgb=img, depth=depth_m, ob_mask=refined_mask, iteration=30
                )
                if T_curr is not None:
                    pose_samples.append(T_curr)
                    self.visualize_result(img, T_curr)
                
                cv2.imshow("Step 2: FoundationPose 6D Tracking", img)
                if cv2.waitKey(1) & 0xFF == ord('q'): break

            if pose_samples:
                self.send_to_robot(name, pose_samples[-1], task)

    def visualize_result(self, image, T_cam_obj):
        """ 绘制 3D 轴与机器人 Base 坐标 """
        # 1. 投影 3D 轴
        length = 0.05
        axis_pts_3d = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]])
        pts_cam = (T_cam_obj[:3, :3] @ axis_pts_3d.T).T + T_cam_obj[:3, 3]
        pts_2d = []
        for p in pts_cam:
            u = int(self.K_MATRIX[0,0] * p[0]/p[2] + self.K_MATRIX[0,2])
            v = int(self.K_MATRIX[1,1] * p[1]/p[2] + self.K_MATRIX[1,2])
            pts_2d.append((u, v))
        
        cv2.line(image, pts_2d[0], pts_2d[1], (0,0,255), 3) # X-Red
        cv2.line(image, pts_2d[0], pts_2d[2], (0,255,0), 3) # Y-Green
        cv2.line(image, pts_2d[0], pts_2d[3], (255,0,0), 3) # Z-Blue

        # 2. 计算并显示 Base 坐标
        P_base = self.T_base_camera @ np.append(T_cam_obj[:3, 3], 1.0)
        txt = f"Base: [{P_base[0]:.3f}, {P_base[1]:.3f}, {P_base[2]:.3f}]"
        cv2.putText(image, txt, (pts_2d[0][0] + 15, pts_2d[0][1] - 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        """ 垂直向下 + 动态 Yaw 逻辑 """
        T_base_pick = self.T_base_camera @ T_cam_obj
        r_pick = R.from_matrix(T_base_pick[:3, :3])
        # 提取偏航角 (Yaw)
        detected_yaw = r_pick.as_euler('zyx', degrees=False)[0]

        # 构造垂直向下位姿
        r_place = R.from_euler('xyz', [np.pi, 0, detected_yaw], degrees=False)
        place_q = r_place.as_quat()

        place_pos = ASSEMBLY_CENTER_BASE + np.array(task_cfg['place']['pos'])
        data = {
            'name': name,
            'pick': {'pos': T_base_pick[:3, 3].tolist(), 'orientation': r_pick.as_quat().tolist()},
            'place': {'pos': place_pos.tolist(), 'orientation': place_q.tolist()}
        }
        with open(RESULT_FILE, 'w') as f: yaml.dump(data, f)
        print(f"✅ 信号已发送。")
        while os.path.exists(RESULT_FILE): time.sleep(0.5)

    def get_mesh_path(self, task_name):
        keyword = "4x2" if "4x2" in task_name else "2x2"
        for f in os.listdir(MESH_DIR):
            if keyword in f and f.endswith(".stl"): return os.path.join(MESH_DIR, f)
        return ""

if __name__ == "__main__":
    node = RobotVisionNode()
    try:
        node.run()
    finally:
        cv2.destroyAllWindows()