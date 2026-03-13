import cv2
import torch
import numpy as np
import pyrealsense2 as rs
import os
import trimesh
import yaml
import time
from PIL import Image
from dataclasses import dataclass
from typing import Optional
from transformers import pipeline

# --- 尝试导入 FoundationPose 核心组件 ---
try:
    from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
    FOUNDATION_POSE_READY = True
except ImportError:
    print("❌ 错误: 未找到 estimater.py，请确保该文件在同一目录下。")
    FOUNDATION_POSE_READY = False

# ================= 数据结构 =================
@dataclass
class BoundingBox:
    xmin: int
    ymin: int
    xmax: int
    ymax: int

    @property
    def xyxy(self):
        return [self.xmin, self.ymin, self.xmax, self.ymax]

@dataclass
class DetectionResult:
    score: float
    label: str
    box: BoundingBox
    pose: Optional[np.ndarray] = None

# ================= 工具函数 =================
def draw_pose_visuals(image, pose, K, label_name, length=0.06):
    """绘制 3D 坐标轴（RGB 对应 XYZ）"""
    if pose is None:
        return image
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
    cv2.line(image, origin, points_2d[1], (0, 0, 255), 3)   # X-红
    cv2.line(image, origin, points_2d[2], (0, 255, 0), 3)   # Y-绿
    cv2.line(image, origin, points_2d[3], (255, 0, 0), 3)   # Z-蓝
    cv2.putText(image, label_name, (origin[0] + 10, origin[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return image

def load_T_base_cam(extrinsic_yaml_path: str) -> np.ndarray:
    """加载相机外参"""
    if not os.path.exists(extrinsic_yaml_path):
        raise FileNotFoundError(f"未找到外参文件: {extrinsic_yaml_path}")
    with open(extrinsic_yaml_path, "r", encoding="utf-8") as f:
        extr = yaml.safe_load(f)
    return np.array(extr["T_base_cam"], dtype=np.float32)

# ================= 核心估计类 =================
class LegoPoseEstimator:
    def __init__(self, mesh_path):
        if not FOUNDATION_POSE_READY: return
        print(">>> 正在加载 FoundationPose 模型...")
        self.scorer = ScorePredictor()
        self.refiner = PoseRefinePredictor()
        self.mesh = trimesh.load(mesh_path)
        
        # 尺寸检查与中心化
        if np.linalg.norm(self.mesh.extents) > 0.1:
            self.mesh.apply_scale(0.001)
        self.mesh.vertices -= self.mesh.bounds.mean(axis=0)
        
        model_pts, _ = trimesh.sample.sample_surface(self.mesh, 2048)
        self.model_pts = torch.from_numpy(model_pts.astype(np.float32)).cuda()
        self.estimator = FoundationPose(model_pts=self.model_pts, model_normals=None, 
                                        mesh=self.mesh, scorer=self.scorer, refiner=self.refiner)
        self.estimator.mesh = self.mesh
        self.estimator.model_pts = self.model_pts
        self.estimator.diameter = float(np.linalg.norm(self.mesh.extents))

    def run_once(self, rgb, depth_m, K, detections):
        depth = depth_m.astype(np.float32)
        H, W = depth.shape
        for det in detections:
            try:
                mask = np.zeros((H, W), dtype=bool)
                y1, y2 = max(0, int(det.box.ymin)), min(H, int(det.box.ymax))
                x1, x2 = max(0, int(det.box.xmin)), min(W, int(det.box.xmax))
                mask[y1:y2, x1:x2] = True
                return self.estimator.register(K=K.astype(np.float32), rgb=rgb, 
                                               depth=depth, ob_mask=mask, iteration=10)
            except Exception as e:
                print(f"⚠️ 解算异常: {e}")
        return None

# ================= 主程序 =================
if __name__ == "__main__":
    # 配置
    ACC_TIME = 5.0  # 稳定识别时间（秒）
    MESH_PATH = "meshes/LEGO_Duplo_brick_4x2.stl"
    TARGET_FILE = "current_target.txt"
    OUTPUT_FILE = "vision_output.yaml"
    EXTR_FILE = "camera_to_base.yaml"

    # 1. 初始化外参与模型
    try:
        T_base_cam = load_T_base_cam(EXTR_FILE)
        estimator = LegoPoseEstimator(MESH_PATH)
        detector = pipeline(model="IDEA-Research/grounding-dino-tiny", 
                            task="zero-shot-object-detection", device="cuda")
    except Exception as e:
        print(f"❌ 初始化失败: {e}"); exit(1)

    # 2. RealSense 初始化
    pipeline_rs = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline_rs.start(config)
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K_MATRIX = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float32)
    align = rs.align(rs.stream.color)

    print(">>> 视觉服务就绪，等待 Executor 指令...")

    try:
        while True:
            if not os.path.exists(TARGET_FILE):
                # 待机模式：仅预览
                fs = pipeline_rs.wait_for_frames(); fs = align.process(fs)
                img = np.asanyarray(fs.get_color_frame().get_data())
                cv2.putText(img, "IDLE: Waiting for instruction...", (10, 30), 2, 0.7, (0,0,255), 2)
                cv2.imshow("Vision Service", img); cv2.waitKey(1); continue

            with open(TARGET_FILE, "r") as f: active_target = f.read().strip()
            if not active_target: continue

            print(f"🚀 识别任务: [{active_target}] - 正在进行 5s 稳定优化...")
            start_t = time.time()
            best_pose_cam = None

            # --- 5秒优化循环 ---
            while (time.time() - start_t) < ACC_TIME:
                fs = pipeline_rs.wait_for_frames(); fs = align.process(fs)
                color_f = fs.get_color_frame(); depth_f = fs.get_depth_frame()
                if not color_f or not depth_f: continue
                
                rgb = np.asanyarray(color_f.get_data())
                depth = np.asanyarray(depth_f.get_data()).astype(np.float32) / 1000.0
                disp = rgb.copy()

                # 检测与解算
                res = detector(Image.fromarray(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)), 
                               candidate_labels=[active_target], threshold=0.3)
                if res:
                    det = DetectionResult(score=float(res[0]["score"]), label=res[0]["label"], 
                                          box=BoundingBox(**res[0]["box"]))
                    curr_pose = estimator.run_once(rgb, depth, K_MATRIX, [det])
                    if curr_pose is not None:
                        best_pose_cam = curr_pose
                        disp = draw_pose_visuals(disp, best_pose_cam, K_MATRIX, active_target)

                # UI 倒计时
                rem = ACC_TIME - (time.time() - start_t)
                cv2.putText(disp, f"Optimizing: {max(0, rem):.1f}s left", (10, 30), 2, 0.7, (0,255,255), 2)
                cv2.imshow("Vision Service", disp); cv2.waitKey(1)

            # --- 5秒结束，保存结果 ---
            if best_pose_cam is not None:
                T_base_obj = T_base_cam @ best_pose_cam
                out = {
                    "target": active_target, "timestamp": time.time(),
                    "t_cam_xyz": best_pose_cam[:3, 3].tolist(),
                    "center_base_xyz": T_base_obj[:3, 3].tolist(),
                    "T_cam_obj": best_pose_cam.tolist(),
                    "T_base_obj": T_base_obj.tolist()
                }
                with open(OUTPUT_FILE, "w") as f: yaml.safe_dump(out, f)
                print(f"✅ 稳定位姿已写入 {OUTPUT_FILE}")

                # 等待 Executor 确认（删除 target 文件）
                while os.path.exists(TARGET_FILE):
                    fs = pipeline_rs.wait_for_frames(); fs = align.process(fs)
                    img_ok = np.asanyarray(fs.get_color_frame().get_data())
                    img_ok = draw_pose_visuals(img_ok, best_pose_cam, K_MATRIX, "LOCKED")
                    cv2.putText(img_ok, "READY - Waiting for Robot", (10, 30), 2, 0.7, (0,255,0), 2)
                    cv2.imshow("Vision Service", img_ok)
                    if cv2.waitKey(1) & 0xFF == ord("q"): break
            else:
                print("⚠️ 5秒内未能解算成功"); os.remove(TARGET_FILE)

    finally:
        pipeline_rs.stop(); cv2.destroyAllWindows()