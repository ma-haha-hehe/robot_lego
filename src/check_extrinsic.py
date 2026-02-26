import rclpy
from rclpy.node import Node
import numpy as np
import pyrealsense2 as rs
import yaml
from scipy.spatial.transform import Rotation as R

class ExtrinsicChecker(Node):
    def __init__(self):
        super().__init__('extrinsic_checker')
        
        # 1. 加载你刚才填写的 YAML
        with open("extr.yaml", "r") as f:
            extr = yaml.safe_load(f)
            self.T_base_cam = np.array(extr["T_base_cam"], dtype=np.float32)

        # 2. 初始化 RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        # 获取内参
        profile = self.pipeline.get_active_profile()
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K = [intr.fx, intr.fy, intr.ppx, intr.ppy]

        self.create_timer(0.5, self.timer_callback)
        print("🔍 验证开始：请观察画面中心点的坐标...")

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        
        if not depth_frame: return

        # 1. 获取图像中心点 (320, 240) 的深度
        u, v = 320, 240
        dist = depth_frame.get_distance(u, v)

        if dist > 0:
            # 2. 像素坐标 -> 相机坐标系 (Camera Frame)
            # 根据内参公式：X = (u-cx)*Z/fx, Y = (v-cy)*Z/fy
            x_cam = (u - self.K[2]) * dist / self.K[0]
            y_cam = (v - self.K[3]) * dist / self.K[1]
            z_cam = dist
            P_cam = np.array([x_cam, y_cam, z_cam, 1.0])

            # 3. 相机坐标系 -> 机械臂基座坐标系 (Base Frame)
            P_base = self.T_base_cam @ P_cam

            print(f"--- 视野中心点位姿 ---")
            print(f"相机坐标系 (Cam): X:{x_cam:.3f}, Y:{y_cam:.3f}, Z:{z_cam:.3f}")
            print(f"基座坐标系 (Base): X:{P_base[0]:.3f}, Y:{P_base[1]:.3f}, Z:{P_base[2]:.3f}")
            print(f"提示：如果点在地面，Base Z 应该接近 0")

def main():
    rclpy.init()
    node = ExtrinsicChecker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
