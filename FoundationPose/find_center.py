import cv2
import torch
import numpy as np
import pyrealsense2 as rs
from PIL import Image
import os

# --- 导入 FoundationPose 相关库 ---
# from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor

# ================= 1. 增强型可视化工具 =================
def draw_pose_info(image, pose, K, length=0.08):
    """
    在图像上绘制:
    1. RGB 3D 坐标轴 (R:X, G:Y, B:Z)
    2. 中心红点
    3. 实时 3D 坐标文字
    """
    if pose is None: return image
    
    # 定义 3D 空间中的轴点 (原点, X轴终点, Y轴终点, Z轴终点)
    points_3d = np.float32([[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, length]])
    R, t = pose[:3, :3], pose[:3, 3]
    
    # 变换到相机坐标系并投影到 2D 像素
    points_cam = (R @ points_3d.T).T + t
    points_2d = []
    for p in points_cam:
        if p[2] <= 0: continue # 剔除相机背后的点
        u = int(K[0, 0] * p[0] / p[2] + K[0, 2])
        v = int(K[1, 1] * p[1] / p[2] + K[1, 2])
        points_2d.append((u, v))
    
    if len(points_2d) < 4: return image

    origin = points_2d[0]
    # 绘制轴线 (注意 OpenCV 默认 BGR: B=Z, G=Y, R=X)
    cv2.line(image, origin, points_2d[1], (0, 0, 255), 4) # X轴 - 红色
    cv2.line(image, origin, points_2d[2], (0, 255, 0), 4) # Y轴 - 绿色
    cv2.line(image, origin, points_2d[3], (255, 0, 0), 4) # Z轴 - 蓝色
    
    # 绘制中心点
    cv2.circle(image, origin, 8, (0, 0, 255), -1)
    cv2.circle(image, origin, 10, (255, 255, 255), 2) # 外圈白边更清晰
    
    # 绘制坐标信息背景板
    overlay = image.copy()
    cv2.rectangle(overlay, (10, 10), (320, 100), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
    
    # 打印 3D 坐标 (单位: 米)
    coord_txt = f"CENTER (m):"
    pos_txt = f"X:{t[0]:.3f} Y:{t[1]:.3f} Z:{t[2]:.3f}"
    cv2.putText(image, coord_txt, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, pos_txt, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    return image

# ================= 2. 主执行逻辑 =================
if __name__ == "__main__":
    # 配置 RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        profile = pipeline.start(config)
        print(">>> RealSense 已启动")
    except Exception as e:
        print(f"!!! 无法启动硬件: {e}")
        exit()

    # 获取内参
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K_MATRIX = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
    align = rs.align(rs.stream.color)

    # 初始化 FoundationPose (此处用你的模型初始化代码)
    # estimator = ... 

    print(">>> 正在运行... 结果将实时写入 output.jpg")
    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame: continue

            img = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            depth_m = depth.astype(np.float32) / 1000.0

            # --- 运行模型推理 ---
            # 这里你需要传入真实的 mask，可以通过 DINO 得到
            # 示例: 假设我们得到了估计的 pose
            # current_pose = estimator.register(K=K_MATRIX, rgb=img, depth=depth_m, ...)
            
            # 【临时演示专用】模拟一个在相机前 40cm 处的姿态
            mock_pose = np.eye(4)
            mock_pose[:3, 3] = [0.02, -0.05, 0.40] 

            # 可视化绘制
            result_img = draw_pose_info(img.copy(), mock_pose, K_MATRIX)

            # 保存结果 (针对 Docker Headless 环境)
            cv2.imwrite("output.jpg", result_img)
            
            # 在终端打印，确认程序没挂
            print(f"\r[ESTIMATING] Center Z: {mock_pose[2,3]:.3f}m", end="")

    except KeyboardInterrupt:
        print("\n>>> 程序已停止")
    finally:
        pipeline.stop()