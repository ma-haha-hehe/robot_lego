#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import numpy as np
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

# --- 测试专用：在这里指定你的本地测试图片路径 ---
TEST_IMAGE_PATH = "/Users/Zhuanz/Downloads/RWTH/robot_lego/src/my_robot_vision/my_robot_vision/c01e2c0f3ec1ca430100ca5addb9c4db.jpg" 

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

# ================= 3. 静态图片测试节点类 =================
class RobotVisionNode:
    def __init__(self):
        # 加载相机内参和外参
        with open(CAMERA_PARAMS_YAML, 'r') as f:
            params = yaml.safe_load(f)
        self.T_base_camera = np.array(params['extrinsic_matrix']).reshape(4, 4)
        
        # 获取内参矩阵 K
        if 'intrinsic_matrix' in params:
            self.K_MATRIX = np.array(params['intrinsic_matrix']).reshape(3, 3)
        else:
            # 默认 fallback (针对 640x480)
            self.K_MATRIX = np.array([[615, 0, 320], [0, 615, 240], [0, 0, 1]])

        with open(TASKS_YAML, 'r') as f:
            data = yaml.safe_load(f)
            self.task_list = data.get('tasksh', data.get('tasks', []))

        from transformers import pipeline
        self.detector = pipeline(model="IDEA-Research/grounding-dino-tiny", task="zero-shot-object-detection", device="cuda")
        
        if SAM_AVAILABLE:
            sam = sam_model_registry["vit_h"](checkpoint="/FoundationPose/weights/sam_vit_h_4b8939.pth").to("cuda")
            self.sam_predictor = SamPredictor(sam)

        self.pose_est = LegoPoseEstimator(ScorePredictor(), PoseRefinePredictor())

    def run(self):
        """ 处理静态图片逻辑 """
        if not os.path.exists(TEST_IMAGE_PATH):
            print(f"❌ 找不到测试图片: {TEST_IMAGE_PATH}")
            return

        # 1. 加载图片
        img_bgr = cv2.imread(TEST_IMAGE_PATH)
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(img_rgb)

        for task in self.task_list:
            name = task['name']
            print(f"\n🎯 正在静态测试: {name}")
            self.pose_est.update_mesh(self.get_mesh_path(name))

            # 2. 目标检测
            results = self.detector(img_pil, candidate_labels=[name, "lego block."], threshold=0.3)
            if not results:
                print(f"⚠️ 图片中未发现: {name}")
                continue
            
            r = results[0] # 取最高分
            det = DetectionResult(r['score'], r['label'], BoundingBox(**r['box']))

            # 3. 分割
            self.sam_predictor.set_image(img_rgb)
            masks, _, _ = self.sam_predictor.predict(box=np.array(det.box.xyxy), multimask_output=False)
            refined_mask = masks[0]

            # 4. 生成虚拟深度图 (假设物体在相机前 0.6m 处)
            # FoundationPose 需要深度信息，我们手动模拟一个平整的桌面深度
            depth_m = np.full(img_bgr.shape[:2], 0.6, dtype=np.float32)

            # 5. 姿态估计
            T_curr = self.pose_est.estimator.register(K=self.K_MATRIX, rgb=img_bgr, depth=depth_m, ob_mask=refined_mask, iteration=50)
            
            if T_curr is not None:
                # 可视化并保存结果图
                res_img = img_bgr.copy()
                self.visualize_result(res_img, T_curr)
                cv2.imwrite("/vision_code/test_result.png", res_img)
                print(f"📊 结果图已保存至 /vision_code/test_result.png")
                
                # 发送原始预测角度给机器人
                self.send_to_robot(name, T_curr, task)

    def visualize_result(self, image, T_cam_obj):
        """ 绘制 3D 坐标轴投影 """
        length = 0.05
        axis_pts_3d = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]])
        pts_cam = (T_cam_obj[:3, :3] @ axis_pts_3d.T).T + T_cam_obj[:3, 3]
        pts_2d = []
        for p in pts_cam:
            u = int(self.K_MATRIX[0,0] * p[0]/p[2] + self.K_MATRIX[0,2])
            v = int(self.K_MATRIX[1,1] * p[1]/p[2] + self.K_MATRIX[1,2])
            pts_2d.append((u, v))
        
        cv2.line(image, pts_2d[0], pts_2d[1], (0,0,255), 3) # X-红
        cv2.line(image, pts_2d[0], pts_2d[2], (0,255,0), 2) # Y-绿
        cv2.line(image, pts_2d[0], pts_2d[3], (255,0,0), 2) # Z-蓝

        # 提取基座系下的预测角度
        T_base_vision = self.T_base_camera @ T_cam_obj
        R_base_obj = T_base_vision[:3, :3]
        raw_yaw_deg = np.degrees(np.arctan2(R_base_obj[1, 0], R_base_obj[0, 0]))
        stable_yaw = ((raw_yaw_deg + 90) % 180) - 90
        
        cv2.putText(image, f"Yaw: {stable_yaw:.1f}deg", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)

    def send_to_robot(self, name, T_cam_obj, task_cfg):
        """ 
        修正版：连续角度输出 + 45度初始位姿补偿
        不再进行 0/90 度强制二值化
        """
        # --- 1. 坐标系转换 ---
        # T_cam_obj 是视觉直接预测的结果，将其转到机器人基座 (Base) 坐标系
        T_base_vision = self.T_base_camera @ T_cam_obj
        R_base_obj = T_base_vision[:3, :3]
        vision_center_pos = T_base_vision[:3, 3].tolist() 

        # --- 2. 连续偏航角提取 ---
        # 从旋转矩阵提取物体 X 轴相对于机器人基座 X 轴的旋转弧度
        raw_yaw_rad = np.arctan2(R_base_obj[1, 0], R_base_obj[0, 0])
        raw_yaw_deg = np.degrees(raw_yaw_rad)

        # --- 3. 180度对称归一化 (核心物理逻辑) ---
        # 理由：积木是长方形，转 180 度是一样的。为了防止机械臂“绕远路”转圈，
        # 我们将角度锁定在 [-90, 90] 区间。这步是必须的，否则机器人会乱转。
        stable_yaw_deg = ((raw_yaw_deg + 90) % 180) - 90

        # --- 4. 执行 45 度补偿 ---
        # 逻辑：最终指令 = 视觉检测到的绝对角度 - 机器人初始 Ready 时的 45 度
        final_robot_yaw_deg = stable_yaw_deg - 45.0
        
        # 将结果再次限制在 [-180, 180]，确保四元数生成的路径最短
        final_robot_yaw_deg = (final_robot_yaw_deg + 180) % 360 - 180
        final_yaw_rad = np.radians(final_robot_yaw_deg)

        # --- 5. 生成位姿四元数 ---
        # Rx=180 (夹爪向下) | Ry=0 | Rz=final_yaw_rad
        pick_quat = R.from_euler('xyz', [np.pi, 0, final_yaw_rad]).as_quat().tolist()

        # --- 6. 生成 YAML 文件 ---
        # 组装区放置角度暂时保持为 0 (对应机器人坐标系的 0-45 = -45度)
        place_quat = R.from_euler('xyz', [np.pi, 0, np.radians(0 - 45.0)]).as_quat().tolist()

        data = {
            'name': name,
            'pick': {'pos': vision_center_pos, 'orientation': pick_quat},
            'place': {
                'pos': (ASSEMBLY_CENTER_BASE + np.array(task_cfg['place']['pos'])).tolist(),
                'orientation': place_quat
            }
        }
        
        with open(RESULT_FILE, 'w') as f:
            yaml.dump(data, f)
            
        print(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print(f"✅ 连续角度指令已发送")
        print(f"🔍 视觉检测绝对角度: {stable_yaw_deg:.2f}°")
        print(f"⚙️ 补偿45度后指令值: {final_robot_yaw_deg:.2f}°")
        print(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        
    def get_mesh_path(self, task_name):
        keyword = "4x2" if "4x2" in task_name else "2x2"
        for f in os.listdir(MESH_DIR):
            if keyword in f and f.endswith(".stl"): return os.path.join(MESH_DIR, f)
        return ""

if __name__ == "__main__":
    node = RobotVisionNode()
    node.run()