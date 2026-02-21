import cv2
import torch
import numpy as np
import pyrealsense2 as rs
import trimesh
import os
from PIL import Image
from dataclasses import dataclass
from transformers import pipeline

# --- 导入 FoundationPose 组件 ---
from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor

# 定义公共绝对路径
COORD_FILE_PATH = "/home/franka_ws/src/panda_pick/src/target_coord.txt"
# ================= 1. 核心转换逻辑 =================
x_offset = 0.016
y_offset = 0.008
z_offset = 0.005
def get_robot_center(pose_cam):
    """
    将 FoundationPose 输出的 4x4 矩阵中的平移向量提取并转换
    返回: [x, y, z] (单位: 米)
    """
    # 提取平移向量 (相机坐标系下的 x, y, z)
    tx, ty, tz = pose_cam[:3, 3]
    
    # 转换为机器人坐标系 (前进, 向左, 向上)
    robot_x = tz + x_offset     # 距离深度变为前进
    robot_y = -tx + y_offset     # 水平偏移取反
    robot_z = -ty + z_offset     # 垂直偏移取反
    
    return np.array([robot_x, robot_y, robot_z])

def save_for_robot(coords):
    """ 将坐标保存到公共目录下的绝对路径 """
    try:
        # 使用 os.makedirs 确保父目录存在（虽然 /home/i6user 肯定存在）
        os.makedirs(os.path.dirname(COORD_FILE_PATH), exist_ok=True)
        
        with open(COORD_FILE_PATH, "w") as f:
            # 格式: x,y,z
            f.write(f"{coords[0]:.6f},{coords[1]:.6f},{coords[2]:.6f}")
        print(f"✅ 坐标已成功写入: {COORD_FILE_PATH}")
    except Exception as e:
        print(f"❌ 写入文件失败: {e}")

# ================= 2. 增强型类定义 =================
class LegoPoseEstimator:
    def __init__(self, mesh_path):
        self.scorer = ScorePredictor()
        self.refiner = PoseRefinePredictor()
        self.mesh = trimesh.load(mesh_path)
        
        # 强制网格中心化 (确保估计的是积木几何中心)
        self.mesh.vertices -= self.mesh.bounds.mean(axis=0)
        
        model_pts, _ = trimesh.sample.sample_surface(self.mesh, 2048)
        self.model_pts = torch.from_numpy(model_pts.astype(np.float32)).cuda()
        self.estimator = FoundationPose(model_pts=self.model_pts, mesh=self.mesh, scorer=self.scorer, refiner=self.refiner)

    def run_once(self, rgb, depth, K, mask):
        # 运行估计并返回相机姿态
        return self.estimator.register(K=K, rgb=rgb, depth=depth, ob_mask=mask, iteration=10)

# ================= 3. 主程序逻辑 =================
if __name__ == "__main__":
    MESH_PATH = "meshes/LEGO_Duplo_brick_4x2.stl"
    
    # 初始化硬件与模型
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K_MATRIX = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
    align = rs.align(rs.stream.color)

    detector = pipeline(model="IDEA-Research/grounding-dino-tiny", task="zero-shot-object-detection", device="cuda")
    estimator = LegoPoseEstimator(MESH_PATH)

    print(">>> 准备就绪。按下 '1' 捕捉积木中心坐标并传递给机器人。")

    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            img = np.asanyarray(frames.get_color_frame().get_data())
            depth_m = np.asanyarray(frames.get_depth_frame().get_data()).astype(np.float32) / 1000.0
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('1'):
                # 1. 2D 检测
                img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                res = detector(img_pil, candidate_labels=["lego."], threshold=0.35)
                
                if res:
                    box = res[0]['box']
                    mask = np.zeros(img.shape[:2], dtype=bool)
                    mask[int(box['ymin']):int(box['ymax']), int(box['xmin']):int(box['xmax'])] = True
                    
                    # 2. 3D 姿态估计 (相机系)
                    pose_cam = estimator.run_once(img, depth_m, K_MATRIX, mask)
                    
                    if pose_cam is not None:
                        # 3. 转换到机器人坐标系并保存
                        robot_coords = get_robot_center(pose_cam)
                        save_for_robot(robot_coords)
                        print(f"already saved")
                        
                        # 在屏幕上绘制确认
                        cv2.putText(img, f"SENT: {robot_coords}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    print("❌ 未检测到积木")

            cv2.imshow("Robot Center Sender", img)
            if key == ord('q'): break
    finally:
        pipeline.stop()