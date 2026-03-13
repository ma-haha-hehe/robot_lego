import cv2
import torch
import numpy as np
import pyrealsense2 as rs
import os
import trimesh
from PIL import Image
from dataclasses import dataclass
from typing import Optional
from transformers import pipeline

# --- 尝试导入 FoundationPose 核心组件 ---
try:
    from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
    FOUNDATION_POSE_READY = True
except ImportError:
    print("❌ 警告: 未找到 estimater.py，请确保该文件在同一目录下。")
    FOUNDATION_POSE_READY = False

# ================= 数据结构 =================
@dataclass
class BoundingBox:
    xmin: int; ymin: int; xmax: int; ymax: int
    @property
    def xyxy(self): return [self.xmin, self.ymin, self.xmax, self.ymax]

@dataclass
class DetectionResult:
    score: float
    label: str
    box: BoundingBox
    pose: Optional[np.ndarray] = None

# ================= 绘图与数学工具 =================
def rotation_matrix_to_euler_angles(R):
    """ 将旋转矩阵转换为欧拉角 (Roll, Pitch, Yaw) """
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z]) * (180 / np.pi)

def draw_pose_visuals(image, pose, K, length=0.06):
    """ 绘制 3D 坐标轴和精确的中心坐标文本 """
    if pose is None: return image
    # 定义 3D 点：原点，X轴，Y轴，Z轴
    points_3d = np.float32([[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, length]])
    R, t = pose[:3, :3], pose[:3, 3]
    points_cam = (R @ points_3d.T).T + t
    
    points_2d = []
    for p in points_cam:
        if p[2] <= 0: return image
        u = int(K[0, 0] * p[0] / p[2] + K[0, 2])
        v = int(K[1, 1] * p[1] / p[2] + K[1, 2])
        points_2d.append((u, v))
    
    origin = points_2d[0]
    # 绘制 RGB 坐标轴
    cv2.line(image, origin, points_2d[1], (0, 0, 255), 3) # X-Red
    cv2.line(image, origin, points_2d[2], (0, 255, 0), 3) # Y-Green
    cv2.line(image, origin, points_2d[3], (255, 0, 0), 3) # Z-Blue
    
    # 绘制中心坐标文本
    coord_text = f"Pos: [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}]"
    cv2.circle(image, origin, 5, (0, 0, 255), -1)
    cv2.putText(image, coord_text, (origin[0] + 12, origin[1] - 12), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    return image

def save_vision_to_yaml(pose_dict, output_path="vision_output.yaml"):
    """
    将物体位姿字典保存为 YAML 文件
    pose_dict: { "obj_name": 4x4_numpy_array }
    """
    data_to_save = {}
    for name, pose in pose_dict.items():
        if pose is not None:
            # 将 numpy 矩阵转换为标准的 Python 列表列表 (List of Lists)
            data_to_save[name] = pose.tolist()
    
    with open(output_path, 'w') as f:
        yaml.dump(data_to_save, f)

# ================= 核心估计类 =================
class LegoPoseEstimator:
    def __init__(self, mesh_path):
        if not FOUNDATION_POSE_READY: return
        print(f">>> 正在加载 FoundationPose 模型...")
        self.scorer = ScorePredictor()
        self.refiner = PoseRefinePredictor()
        
        print(f" -> 正在加载并校准网格中心: {mesh_path}")
        self.mesh = trimesh.load(mesh_path)
        
        # 自动缩放修复
        scale_check = np.linalg.norm(self.mesh.extents)
        if scale_check > 0.1:
            self.mesh.apply_scale(0.001)
            
        # === 💡 核心修复：将网格几何中心平移至 (0,0,0) ===
        center_offset = self.mesh.bounds.mean(axis=0)
        self.mesh.vertices -= center_offset
        print(f"✅ 模型中心已校准。偏置量: {center_offset}")
            
        # 采样点云
        model_pts, _ = trimesh.sample.sample_surface(self.mesh, 2048)
        self.model_pts = torch.from_numpy(model_pts.astype(np.float32)).cuda()
        
        self.estimator = FoundationPose(
            model_pts=self.model_pts, model_normals=None, mesh=self.mesh,
            scorer=self.scorer, refiner=self.refiner
        )
        
        # 注入校准后的参数
        self.estimator.mesh = self.mesh
        self.estimator.model_pts = self.model_pts
        self.estimator.center = torch.mean(self.model_pts, dim=0)
        self.estimator.diameter = float(np.linalg.norm(self.mesh.extents))

    def run_once(self, rgb, depth, K, detections):
        depth = depth.astype(np.float32)
        K = K.astype(np.float32)
        H, W = depth.shape
        best_pose = None
        
        for det in detections:
            try:
                mask = np.zeros((H, W), dtype=bool)
                y1, y2 = max(0, det.box.ymin), min(H, det.box.ymax)
                x1, x2 = max(0, det.box.xmin), min(W, det.box.xmax)
                mask[y1:y2, x1:x2] = True

                pose = self.estimator.register(
                    K=K, rgb=rgb, depth=depth, ob_mask=mask, iteration=8 # 增加迭代次数以提高精度
                )
                best_pose = pose
                break
            except Exception as e:
                print(f"❌ 估计出错: {e}")
        return best_pose

# ================= 主程序逻辑 =================
if __name__ == "__main__":
    MESH_PATH = "meshes/LEGO_Duplo_brick_4x2.stl"
    
    # 1. 初始化 RealSense 硬件
    pipeline_rs = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        profile = pipeline_rs.start(config)
    except Exception as e:
        print(f"!!! 无法启动 RealSense: {e}. 请检查 USB 连接或 Docker 权限。")
        exit()

    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K_MATRIX = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
    align = rs.align(rs.stream.color)

    # 2. 初始化 AI 组件
    print(">>> 正在加载 DINO 检测器...")
    detector = pipeline(model="IDEA-Research/grounding-dino-tiny", task="zero-shot-object-detection", device="cuda")
    estimator = LegoPoseEstimator(MESH_PATH)

    # 状态变量
    pose_1 = None
    pose_2 = None
    last_text = "Ready: Press '1' for Start, '2' for End"

    print(">>> 系统就绪！")

    try:
        while True:
            frames = pipeline_rs.wait_for_frames()
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame: continue
            
            img = np.asanyarray(color_frame.get_data())
            depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) / 1000.0

            key = cv2.waitKey(1) & 0xFF
            
            # --- 姿态捕获逻辑 ---
            if key == ord('1') or key == ord('2'):
                print(">>> 正在计算...")
                img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                results = detector(img_pil, candidate_labels=["block.", "lego."], threshold=0.35)
                dets = [DetectionResult(0, r['label'], BoundingBox(**r['box'])) for r in results]
                
                captured_pose = estimator.run_once(img, depth_m, K_MATRIX, dets)
                
                if captured_pose is not None:
                    if key == ord('1'):
                        pose_1, pose_2 = captured_pose, None
                        last_text = "Pose 1 (Start) Captured!"
                    elif key == ord('2'):
                        if pose_1 is not None:
                            pose_2 = captured_pose
                            # 计算 6D 变换
                            T_diff = np.linalg.inv(pose_1) @ pose_2
                            trans = T_diff[:3, 3] * 100 # cm
                            last_text = f"Offset: X:{trans[0]:.1f} Y:{trans[1]:.1f} Z:{trans[2]:.1f} cm"
                            print(f"\n🚀 变换矩阵:\n{T_diff}")
                else:
                    last_text = "Detection Failed!"

            # --- 实时渲染 ---
            img_display = img.copy()
            if pose_1 is not None:
                img_display = draw_pose_visuals(img_display, pose_1, K_MATRIX)
                cv2.putText(img_display, "START", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            if pose_2 is not None:
                img_display = draw_pose_visuals(img_display, pose_2, K_MATRIX)
                cv2.putText(img_display, "END", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)

            cv2.putText(img_display, last_text, (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 实时窗口显示
            cv2.imshow("RealSense Live Pose Estimator", img_display)
            if key == ord('q'): break

    finally:
        pipeline_rs.stop()
        cv2.destroyAllWindows()