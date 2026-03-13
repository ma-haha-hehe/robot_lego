import pyrealsense2 as rs
import numpy as np
import cv2

def main():
    # 1. 初始化 RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    
    # 配置流：同时显示 RGB 和 深度图
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 开启流
    print(">>> 正在启动相机... 按 'q' 退出")
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # 将图像转为 numpy 数组
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # === 关键步骤：深度图可视化 ===
            # 原始深度图是黑乎乎的 16位 整数，人眼看不清。
            # 我们用伪彩色 (ColorMap) 把它渲染出来：
            # alpha=0.03 是为了把距离缩放到 0-255 之间方便显示
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # 将 RGB 和 深度图 并排显示
            images = np.hstack((color_image, depth_colormap))

            cv2.namedWindow('RealSense Depth Check', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense Depth Check', images)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()