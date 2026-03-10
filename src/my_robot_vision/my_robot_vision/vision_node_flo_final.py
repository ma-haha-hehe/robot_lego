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
# 替换为你最新的任务文件
TASKS_YAML = "/vision_code/tasksh_true.yaml"
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
        
        # 🔥 修改: 初始化 Florence-2
        print("🚀 正在加载 Florence-2 模型...")
        self.florence_model = AutoModelForCausalLM.from_pretrained("microsoft/Florence-2-base", trust_remote_code=True).to("cuda").eval()
        self.florence_processor = AutoProcessor.from_pretrained("microsoft/Florence-2-base", trust_remote_code=True)

        if SAM_AVAILABLE:
            print("🚀 正在加载 SAM 模型...")
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

    def run_florence_detection(self, image_pil, text_input):
        """使用 Florence-2 进行目标检测"""
        prompt = f"<CAPTION_TO_PHRASE_GROUNDING>{text_input}"
        inputs = self.florence_processor(text=prompt, images=image_pil, return_tensors="pt").to("cuda")
        
        with torch.no_grad():
            generated_ids = self.florence_model.generate(
                input_ids=inputs["input_ids"],
                pixel_values=inputs["pixel_values"],
                max_new_tokens=1024,
                early_stopping=False,
                do_sample=False,
                num_beams=3,
            )
        
        generated_text = self.florence_processor.batch_decode(generated_ids, skip_special_tokens=False)[0]
        parsed_answer = self.florence_processor.post_process_generation(generated_text, task="<CAPTION_TO_PHRASE_GROUNDING>", image_size=(image_pil.width, image_pil.height))
        
        # 提取结果 (Florence-2 返回格式为 {'<CAPTION_TO_PHRASE_GROUNDING>': {'bboxes': [[x1,y1,x2,y2],...], 'labels': [...]}})
        results = parsed_answer["<CAPTION_TO_PHRASE_GROUNDING>"]
        if len(results['bboxes']) > 0:
            # 取第一个匹配项
            box = results['bboxes'][0]
            return DetectionResult(score=1.0, label=text_input, box=BoundingBox(int(box[0]), int(box[1]), int(box[2]), int(box[3])))
        return None

    def run(self):
        for task in self.task_list:
            name = task['name']
            while os.path.exists(RESULT_FILE):
                print("⏳ 等待旧任务完成信号清除...")
                time.sleep(1.0)

            self.clear_buffer()
            print(f"\n🎯 [开始任务] 目标: {name}")
            mesh_path = self.get_mesh_path(name)
            self.pose_est.update_mesh(mesh_path)

            # --- 步骤 1: Florence-2 目标检测 ---
            print("👁️ 步骤 1: Florence-2 检测中...")
            frames = self.pipeline.wait_for_frames()
            img_bgr = np.asanyarray(self.align.process(frames).get_color_frame().get_data())
            img_pil = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
            
            best_det = self.run_florence_detection(img_pil, name)
            
            if best_det:
                viz_det = img_bgr.copy()
                cv2.rectangle(viz_det, (best_det.box.xmin, best_det.box.ymin), (best_det.box.xmax, best_det.box.ymax), (0, 255, 255), 2)
                cv2.putText(viz_det, f"Florence-2: {name}", (best_det.box.xmin, best_det.box.ymin-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.imshow("Vision - Detection", viz_det)
                cv2.waitKey(1500) # 🔥 停留 1.5 秒
            else:
                print("⚠️ 未检测到目标，重试...")
                continue

            # --- 步骤 2: SAM 分割 ---
            print("🎭 步骤 2: SAM 分割中...")
            if SAM_AVAILABLE:
                self.sam_predictor.set_image(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
                masks, _, _ = self.sam_predictor.predict(box=np.array(best_det.box.xyxy), multimask_output=False)
                refined_mask = masks[0]
                
                mask_viz = img_bgr.copy()
                color_mask = np.zeros_like(mask_viz)
                color_mask[refined_mask] = [0, 255, 0]
                mask_viz = cv2.addWeighted(mask_viz, 0.7, color_mask, 0.3, 0)
                cv2.imshow("Vision - Segmentation", mask_viz)
                cv2.waitKey(1500) # 🔥 停留 1.5 秒
            else:
                refined_mask = np.zeros(img_bgr.shape[:2], dtype=bool)
                refined_mask[best_det.box.ymin:best_det.box.ymax, best_det.box.xmin:best_det.box.xmax] = True

            # --- 步骤 3: FoundationPose 6D 姿态估计 ---
            print("📏 步骤 3: FoundationPose 姿态计算中...")
            pose_samples = []
            refine_start = time.time()
            # 持续 4 秒收集样本以保证稳定性
            while (time.time() - refine_start) < 4.0:
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                curr_img = np.asanyarray(aligned.get_color_frame().get_data())
                depth_m = np.asanyarray(aligned.get_depth_frame().get_data()).astype(np.float32) / 1000.0
                
                T_curr = self.pose_est.estimator.register(K=self.K_MATRIX, rgb=curr_img, depth=depth_m, ob_mask=refined_mask, iteration=10)
                
                if T_curr is not None:
                    pose_samples.append(T_curr)
                    self.visualize_result(curr_img, T_curr)
                
                cv2.imshow("Vision - 6D Pose", curr_img)
                cv2.waitKey(1)

            if pose_samples:
                cv2.waitKey(1500) # 🔥 最后结果停留 1.5 秒
                self.send_to_robot(name, pose_samples[-1], task)
                print(f"✅ 任务 {name} 数据已发送。")

    def visualize_result(self, image, T_cam_obj):
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

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        T_base_obj = self.T_base_camera @ T_cam_obj
        R_base = T_base_obj[:3, :3]
        raw_yaw_deg = np.degrees(np.arctan2(R_base[1, 0], R_base[0, 0]))
        refined_yaw = ((raw_yaw_deg + 90) % 180) - 90
        
        strategy_spin = float(task_cfg.get('grasp_spin', 0))
        blueprint_yaw = float(task_cfg.get('blueprint_yaw', 0))
        
        brick_q = R.from_euler('xyz', [0, 0, refined_yaw], degrees=True).as_quat().tolist()
        final_pick_yaw = refined_yaw + strategy_spin - 135
        pick_q = R.from_euler('xyz', [180, 0, final_pick_yaw], degrees=True).as_quat().tolist()
        final_place_yaw = blueprint_yaw + strategy_spin - 45.0
        place_q = R.from_euler('xyz', [180, 0, final_place_yaw], degrees=True).as_quat().tolist()

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