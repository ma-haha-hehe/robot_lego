import cv2
import numpy as np
import pyrealsense2 as rs
from PIL import Image
from transformers import pipeline

# ================= PCA 核心算法 =================
def compute_pca_pose(depth_meter, box, K):
    """
    使用 PCA 计算点云的姿态
    """
    # 1. 提取框内的深度数据
    xmin, ymin, xmax, ymax = box
    crop_depth = depth_meter[ymin:ymax, xmin:xmax]
    
    # 2. 生成点云 (从 2D 像素 -> 3D 坐标)
    # 创建网格
    h, w = crop_depth.shape
    if h <= 0 or w <= 0: return None
    
    xx, yy = np.meshgrid(np.arange(xmin, xmax), np.arange(ymin, ymax))
    
    # 过滤掉无效深度 (0) 和 太远的背景 (> 1米)
    valid_mask = (crop_depth > 0) & (crop_depth < 1.0) 
    if np.sum(valid_mask) < 50: return None # 点太少不算
    
    z = crop_depth[valid_mask]
    x = (xx[valid_mask] - K[0, 2]) * z / K[0, 0]
    y = (yy[valid_mask] - K[1, 2]) * z / K[1, 1]
    
    # 堆叠成 (N, 3) 的点云数组
    points = np.stack([x, y, z], axis=1)
    
    # === 3. PCA 计算 ===
    # A. 计算中心 (Centroid) -> 作为平移向量 t
    centroid = np.mean(points, axis=0)
    
    # B. 计算协方差矩阵并特征分解
    cov_matrix = np.cov(points.T)
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
    
    # C. 排序特征向量 (按特征值从大到小: 长轴 -> 中轴 -> 短轴)
    sort_indices = np.argsort(eigenvalues)[::-1]
    R = eigenvectors[:, sort_indices]
    
    # D. 确保坐标系是右手系 (X cross Y = Z)
    # 这一步是为了防止 Z 轴莫名其妙反向
    R[:, 2] = np.cross(R[:, 0], R[:, 1])
    
    # 组装 4x4 矩阵
    pose = np.eye(4)
    pose[:3, :3] = R
    pose[:3, 3] = centroid
    
    return pose

# ================= 画图工具 =================
def draw_axis(img, pose, K, length=0.05):
    if pose is None: return img
    origin = pose[:3, 3]
    R = pose[:3, :3]
    
    # 定义轴的终点 (世界坐标系)
    points_3d = np.float32([
        [0,0,0],       # 原点
        [length,0,0],  # X (红)
        [0,length,0],  # Y (绿)
        [0,0,length]   # Z (蓝)
    ])
    
    # 变换到相机坐标系
    points_cam = (R @ points_3d.T).T + origin
    
    # 投影到像素
    points_2d = []
    for p in points_cam:
        u = int(K[0,0]*p[0]/p[2] + K[0,2])
        v = int(K[1,1]*p[1]/p[2] + K[1,2])
        points_2d.append((u,v))
        
    origin_uv = points_2d[0]
    cv2.line(img, origin_uv, points_2d[1], (0,0,255), 3) # X - Red (最长轴)
    cv2.line(img, origin_uv, points_2d[2], (0,255,0), 3) # Y - Green
    cv2.line(img, origin_uv, points_2d[3], (255,0,0), 3) # Z - Blue (法线/最短轴)
    return img

# ================= 主程序 =================
if __name__ == "__main__":
    # 初始化 Realsense
    pipeline_rs = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline_rs.start(config)
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])

    # 初始化 DINO (只用来找大概位置)
    print(">>> Loading DINO...")
    detector = pipeline(model="IDEA-Research/grounding-dino-tiny", task="zero-shot-object-detection", device="cuda")

    try:
        while True:
            frames = pipeline_rs.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame: continue
            
            img = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            depth_m = depth.astype(np.float32) / 1000.0

            # 1. 运行 DINO
            img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            results = detector(img_pil, candidate_labels=["lego brick."], threshold=0.35)

            for r in results:
                # 获取 2D 框
                b = r['box']
                box = [b['xmin'], b['ymin'], b['xmax'], b['ymax']]
                cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), (0,255,0), 2)
                
                # 2. 运行 PCA 姿态估计
                pca_pose = compute_pca_pose(depth_m, box, K)
                
                if pca_pose is not None:
                    draw_axis(img, pca_pose, K)
                    cv2.putText(img, "PCA Pose", (box[0], box[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

            cv2.imshow("PCA Method", img)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    finally:
        pipeline_rs.stop()
        cv2.destroyAllWindows()