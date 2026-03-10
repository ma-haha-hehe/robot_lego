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
TASKS_YAML = "/vision_code/task_test_t.yaml"#t and bigt
CAMERA_PARAMS_YAML = "/vision_code/camera_params.yaml"
MESH_DIR = "/FoundationPose/meshes"
ASSEMBLY_CENTER_BASE = np.array([0.35, 0.2, 0.025])

#  仿真修正：删除 -45度补偿，设为 0.0
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
            """
            全流程视觉控制逻辑：从任务读取到 6D 位姿输出
            """
            # 定义硬过滤容差：比例偏差超过 0.3 (30%) 则判定为错误积木
            RATIO_TOLERANCE = 0.3 
            
            for task in self.task_list:
                name = task['name']  # 任务名，例如 "brick_4x2_L1_1"
                
                # --- 0. 任务同步：等待机器人完成上一个动作 ---
                while os.path.exists(RESULT_FILE):
                    print("⏳ 正在等待机器人清除旧任务信号...")
                    time.sleep(0.5)

                print(f"\n🎯 [视觉启动] 正在寻找目标: {name}")
                
                # 根据任务名加载对应的 3D 模型 (2x2 或 2x4)
                mesh_path = self.get_mesh_path(name)
                self.pose_est.update_mesh(mesh_path)
                
                # 确定期望的物理比例：2x4 -> 2.0, 2x2 -> 1.0
                target_ratio = 2.0 if ("4x2" in name or "2x4" in name) else 1.0

                # --- 1. 图像采集与全图检测 ---
                self.clear_buffer() # 确保拿到的是最新的一帧
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                img_bgr = np.asanyarray(aligned_frames.get_color_frame().get_data())
                img_pil = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
                
                # 使用 DINO 找出场景中所有的候选积木
                results = self.detector(img_pil, candidate_labels=["lego block"], threshold=0.15)
                
                valid_candidates = []
                viz_frame = img_bgr.copy()

                # --- 2. 几何校验：硬过滤（Hard Reject）逻辑 ---
                for r in results:
                    box = [r['box']['xmin'], r['box']['ymin'], r['box']['xmax'], r['box']['ymax']]
                    
                    # A. 调用 SAM 获取精确掩码
                    self.sam_predictor.set_image(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
                    masks, _, _ = self.sam_predictor.predict(box=np.array(box), multimask_output=False)
                    mask = masks[0]
                    
                    # B. 计算最小外接矩形 (OBB) 得到真实长宽比
                    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if not contours: continue
                    rect = cv2.minAreaRect(max(contours, key=cv2.contourArea))
                    w, h = rect[1]
                    actual_ratio = max(w, h) / (min(w, h) + 1e-6)
                    
                    # C. 比例对比与硬过滤
                    ratio_diff = abs(actual_ratio - target_ratio)
                    if ratio_diff > RATIO_TOLERANCE:
                        # 比例不对，直接在图中画红色 X 并跳过
                        cv2.rectangle(viz_frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 0, 255), 2)
                        cv2.putText(viz_frame, f"REJECT:{actual_ratio:.1f}", (int(box[0]), int(box[1])-5), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                        continue 
                    
                    # D. 只有通过过滤的才加入候选集
                    # 评分逻辑：DINO 置信度 * 几何契合度
                    match_score = r['score'] * (1.0 / (ratio_diff + 0.1))
                    valid_candidates.append({'mask': mask, 'score': match_score, 'ratio': actual_ratio, 'box': box})
                    
                    # 在图中标记候选者
                    cv2.rectangle(viz_frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 2)

                # --- 3. 择优录取 ---
                if not valid_candidates:
                    print(f"❌ 未找到符合比例 {target_ratio} 的积木。")
                    cv2.imshow("Detection Logic", viz_frame); cv2.waitKey(1000)
                    continue

                # 选出得分最高的一个作为最终目标
                winner = max(valid_candidates, key=lambda x: x['score'])
                
                # 可视化演示：高亮绿色并停顿 2 秒
                cv2.rectangle(viz_frame, (int(winner['box'][0]), int(winner['box'][1])), 
                            (int(winner['box'][2]), int(winner['box'][3])), (0, 255, 0), 4)
                cv2.putText(viz_frame, "WINNER: RATIO MATCHED", (int(winner['box'][0]), int(winner['box'][1])-20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("Detection Logic", viz_frame)
                cv2.waitKey(2000) 

                # --- 4. 6D 姿态追踪 (FoundationPose) ---
                print("📐 正在精炼 6D 位姿...")
                pose_samples = []
                start_time = time.time()
                
                # 持续追踪 4 秒以获得稳定的结果
                while (time.time() - start_time) < 4.0:
                    frames = self.pipeline.wait_for_frames()
                    aligned = self.align.process(frames)
                    rgb = np.asanyarray(aligned.get_color_frame().get_data())
                    depth = np.asanyarray(aligned.get_depth_frame().get_data()).astype(np.float32) / 1000.0 # 转为米
                    
                    # 使用胜出者的掩码进行配准
                    T_curr = self.pose_est.estimator.register(
                        K=self.K_MATRIX, rgb=rgb, depth=depth, 
                        ob_mask=winner['mask'], iteration=20
                    )
                    
                    if T_curr is not None:
                        pose_samples.append(T_curr)
                        self.visualize_result(rgb, T_curr) # 实时画出 3D 坐标轴
                    
                    cv2.imshow("6D Pose Tracking", rgb)
                    if cv2.waitKey(1) & 0xFF == ord('q'): break

                # --- 5. 数据输出：将结果发送给 C++ 节点 ---
                if pose_samples:
                    print(f"✅ 位姿解算成功，正在保存任务...")
                    cv2.waitKey(1500) # 展示最后的追踪结果
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