#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import torch
from PIL import Image as PILImage
from ultralytics import YOLOWorld
from transformers import AutoModelForMaskGeneration, AutoProcessor

# 从你刚才那个文件里导入工具函数
from yolo_sam import (
    detect_video_yolo,
    segment_video,
    annotate_video_frame,
    DetectionResult
)

class VLMNode(Node):
    def __init__(self):
        super().__init__("vlm_node")

        # === 参数 ===
        self.image_topic = "/camera/color/image_raw"  # 这里改成你实际发布的相机话题
        self.annotated_topic = "/vlm/annotated_image"

        self.labels = ["box", "robot arm"]
        self.conf_threshold = 0.01
        self.polygon_refinement = False

        # 自动选择设备
        if torch.cuda.is_available():
            self.device = "cuda"
        elif torch.backends.mps.is_available():
            self.device = "mps"
        else:
            self.device = "cpu"

        self.get_logger().info(f"Using device: {self.device}")

        # === 加载模型（只加载一次） ===
        detector_model_name = "weights/yolov8s-world.pt"
        segmenter_id = "facebook/sam-vit-base"

        self.get_logger().info("Loading YOLO-World model...")
        self.detector_model = YOLOWorld(detector_model_name)
        self.detector_model.set_classes(self.labels)
        self.detector_model.to(self.device)
        self.get_logger().info("YOLO-World loaded.")

        self.get_logger().info("Loading SAM model...")
        self.sam_model = AutoModelForMaskGeneration.from_pretrained(segmenter_id).to(self.device)
        self.sam_processor = AutoProcessor.from_pretrained(segmenter_id)
        self.get_logger().info("SAM loaded.")

        # === ROS 通信 ===
        self.bridge = CvBridge()

        # 订阅 Mujoco 虚拟相机图像
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        # 发布标注后的图像
        self.annotated_pub = self.create_publisher(
            Image,
            self.annotated_topic,
            10
        )

        self.get_logger().info(
            f"VLM node initialized. Subscribing to {self.image_topic}, "
            f"publishing annotated image to {self.annotated_topic}"
        )

        # 你可以加一个计数器，只每 N 帧跑一次检测，减轻负载
        self.frame_count = 0
        self.process_every_n = 1  # 每帧都处理；如果卡，可以改成 3、5 等

    def image_callback(self, msg: Image):
        """收到一帧 Mujoco 相机图像之后的处理逻辑"""
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        # 1. ROS Image -> OpenCV BGR
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("Raw Input Check", frame_bgr)
            cv2.waitKey(1)  # 必须加这一行，给 OpenCV 1ms 时间刷新窗口
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # 2. BGR -> RGB + PIL Image（与你原本脚本里的流程保持一致）
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        image_pil = PILImage.fromarray(frame_rgb)
        

        try:
            # 3. YOLOWorld 检测
            detections = detect_video_yolo(
                image=image_pil,
                detector_model=self.detector_model,
                conf_threshold=self.conf_threshold
            )
            self.get_logger().info(f"Got {len(detections)} detections")
    
            # 4. SAM 分割
            detections_with_masks = segment_video(
                image=image_pil,
                detection_results=detections,
                segmentator=self.sam_model,
                processor=self.sam_processor,
                polygon_refinement=self.polygon_refinement,
                device=self.device
            )

            # 5. 画框 + 掩码（在 BGR 帧上画，保持和你原代码一致）
            annotated_bgr = annotate_video_frame(frame_bgr.copy(), detections_with_masks)

        except Exception as e:
            self.get_logger().error(f"VLM inference failed: {e}")
            return

        # 6. （可选）本地窗口显示（调试用）
        cv2.imshow("MuJoCo VLM", annotated_bgr)
        cv2.waitKey(1)

        # 7. BGR -> ROS Image，并发布
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_bgr, encoding="bgr8")
            annotated_msg.header = msg.header  # 保留时间戳和 frame_id
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = VLMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down VLM node.")
        node.destroy_node()
        rclpy.shutdown()
        # cv2.destroyAllWindows()


if __name__ == "__main__":
    main()