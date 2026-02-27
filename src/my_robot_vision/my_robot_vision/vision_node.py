#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import yaml
import torch
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
import warnings

warnings.filterwarnings("ignore", category=UserWarning)

# ================= 配置路径 (请根据 Docker 挂载点修改) =================
# 建议在 Docker 启动时挂载到 /root/shared_data
RESULT_FILE = "/root/shared_data/active_task.yaml"
TASKS_YAML = "/root/vision_code/config/tasks.yaml"
CAMERA_PARAMS_YAML = "/root/vision_code/config/camera_params.yaml"
FP_REPO = "/root/FoundationPose"

# 装配中心在机械臂基座坐标系下的绝对位置
ASSEMBLY_CENTER_BASE = np.array([0.45, -0.20, 0.02])

class LegoVisionPure:
    def __init__(self):
        # 1. 加载参数
        self.load_camera_params()
        with open(TASKS_YAML, 'r') as f:
            self.task_list = yaml.safe_load(f)['tasks']
        
        # 2. 路径挂载
        if FP_REPO not in sys.path:
            sys.path.append(FP_REPO)
            sys.path.append(os.path.join(FP_REPO, "root"))
        
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        # 3. 初始化 AI 模型
        self.init_ai_models()
        self.init_camera()
        
        print(f"🚀 视觉节点启动成功 (非 ROS 版)")
        print(f"📂 结果将输出至: {RESULT_FILE}")

    def load_camera_params(self):
        with open(CAMERA_PARAMS_YAML, 'r') as f:
            params = yaml.safe_load(f)
        self.T_base_camera = np.array(params['extrinsic_matrix']).reshape(4, 4)

    def init_ai_models(self):
        print("正在加载 AI 模型 (GroundingDINO & FoundationPose)...")
        from transformers import AutoProcessor, GroundingDinoForObjectDetection
        model_id = "IDEA-Research/grounding-dino-tiny"
        self.dino_processor = AutoProcessor.from_pretrained(model_id)
        self.dino_model = GroundingDinoForObjectDetection.from_pretrained(model_id).to(self.device)

        from estimater import PoseRefinePredictor 
        self.fp_refiner = PoseRefinePredictor()

    def init_camera(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

    def get_stable_pose(self, block_name, duration=6.0):
        start_time = time.time()
        latest_pose_cam = None
        print(f"⏳ 正在识别并稳定位姿: {block_name} (持续 {duration} 秒)...")
        
        while (time.time() - start_time) < duration:
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            color_img = np.asanyarray(aligned.get_color_frame().get_data())
            depth_img = np.asanyarray(aligned.get_depth_frame().get_data())
            
            inputs = self.dino_processor(images=Image.fromarray(color_img), text=block_name, return_tensors="pt").to(self.device)
            with torch.no_grad():
                outputs = self.dino_model(**inputs)
            
            results = self.dino_processor.post_process_grounded_object_detection(
                outputs, inputs.input_ids, target_sizes=[color_img.shape[:2]])
            
            if len(results[0]["boxes"]) > 0:
                bbox = results[0]["boxes"][0].cpu().numpy()
                K = np.array([[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1]]) 
                latest_pose_cam = self.fp_refiner.predict(color_img, depth_img, K, bbox=bbox)
            
            time.sleep(0.05) 
        return latest_pose_cam

    def run_sequence(self):
        for task in self.task_list:
            name = task['name']
            
            # 1. 稳定识别
            T_c_obj = self.get_stable_pose(name, duration=6.0)

            if T_c_obj is None:
                print(f"❌ 错误: 无法识别积木 '{name}'")
                continue

            # 2. 计算坐标变换 (相机 -> 基座)
            off_p = np.array(task['pick']['pos'])
            off_q = task['pick']['orientation']
            T_offset = np.eye(4)
            T_offset[:3, :3] = R.from_quat(off_q).as_matrix()
            T_offset[:3, 3] = off_p

            T_base_pick = self.T_base_camera @ T_c_obj @ T_offset
            pick_q = R.from_matrix(T_base_pick[:3, :3]).as_quat()
            
            # 3. 计算放置位姿
            off_place_p = np.array(task['place']['pos'])
            off_place_q = task['place']['orientation']
            final_place_pos = ASSEMBLY_CENTER_BASE + off_place_p
            
            # 4. 构建数据结构
            # 格式适配你的 C++ 节点解析逻辑
            res = {
                'id': task.get('id', 0),
                'name': name,
                'pick': {
                    'pos': [float(T_base_pick[0, 3]), float(T_base_pick[1, 3]), float(T_base_pick[2, 3])],
                    'orientation': [float(pick_q[0]), float(pick_q[1]), float(pick_q[2]), float(pick_q[3])]
                },
                'place': {
                    'pos': [float(final_place_pos[0]), float(final_place_pos[1]), float(final_place_pos[2])],
                    'orientation': [float(off_place_q[0]), float(off_place_q[1]), float(off_place_q[2]), float(off_place_q[3])]
                }
            }

            # 5. 写入文件（覆盖模式）
            with open(RESULT_FILE, 'w') as f:
                yaml.dump(res, f)
            print(f"✅ 任务已就绪: {name}。坐标已写入 {RESULT_FILE}")
            
            # 6. 等待 C++ 节点处理完毕 (握手)
            print("⏳ 等待机械臂完成动作并删除文件...")
            while os.path.exists(RESULT_FILE):
                time.sleep(0.5)
            print("🔔 检测到文件已删除，开始下一个任务。")

if __name__ == '__main__':
    vision = LegoVisionPure()
    try:
        vision.run_sequence()
    except KeyboardInterrupt:
        print("\n👋 正在退出视觉节点...")
        if os.path.exists(RESULT_FILE): os.remove(RESULT_FILE)