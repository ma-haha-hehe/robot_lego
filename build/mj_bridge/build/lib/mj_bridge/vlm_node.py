#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import json
from dataclasses import dataclass
from typing import List, Optional, Dict, Union

# --- 引入您提供的 yolo_sam.py 中的核心库 ---
from ultralytics import YOLOWorld
from transformers import AutoModelForMaskGeneration, AutoProcessor

# ==========================================
# 1. 辅助类定义 (源自您的 yolo_sam.py)
# ==========================================

@dataclass
class BoundingBox:
    xmin: int
    ymin: int
    xmax: int
    ymax: int

    @property
    def xyxy(self) -> List[float]:
        return [self.xmin, self.ymin, self.xmax, self.ymax]
    
    @property
    def center(self) -> List[int]:
        """计算中心点坐标"""
        return [int((self.xmin + self.xmax) / 2), int((self.ymin + self.ymax) / 2)]

@dataclass
class DetectionResult:
    score: float
    label: str
    box: BoundingBox
    mask: Optional[np.array] = None

# ==========================================
# 2. ROS 2 VLM 节点类
# ==========================================

class VLMNode(Node):
    def __init__(self):
        super().__init__('vlm_inference_node')
        
        # --- 配置参数 ---
        # 默认寻找的目标 (可以通过话题修改)
        self.target_labels = ["cube"]
        self.conf_threshold = 0.01  # 降低阈值以提高检出率
        # --- 关键修改 2: 适配 Mac M3 (MPS) ---
        if torch.cuda.is_available():
            self.device = "cuda"
        elif torch.backends.mps.is_available():
            self.device = "mps"  # 激活 Mac GPU 加速
        else:
            self.device = "cpu"
        
        self.get_logger().info(f"正在加载模型到 {self.device}，请稍候...")

        # --- 加载模型 (YOLO-World + MobileSAM) ---
        try:
            # 1. 加载检测器 (YOLO-World) - 这里是你的"眼睛"
            self.detector = YOLOWorld("weights/yolov8s-world.pt") 
            self.detector.set_classes(self.target_labels)
            self.detector.to(self.device)
            
            # 2. 加载分割器 (MobileSAM) - 可选，为了更精细
            self.segmenter_id = "facebook/sam-vit-base" # 或者使用更快的 "mobile-sam"
            self.sam_model = AutoModelForMaskGeneration.from_pretrained(self.segmenter_id).to(self.device)
            self.sam_processor = AutoProcessor.from_pretrained(self.segmenter_id)
            
            self.get_logger().info("✅ 模型加载完成！")
        except Exception as e:
            self.get_logger().error(f"❌ 模型加载失败: {e}")
            return

        # --- ROS 通信接口 ---
        self.bridge = CvBridge()

        # 1. 订阅摄像头图像
        self.sub_image = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            1 # queue size 1，保证只处理最新帧，不积压
        )

        # 2. 订阅 Prompt 修改指令 (例如发送 "cup" 就会改为找杯子)
        self.sub_prompt = self.create_subscription(
            String,
            "/vlm/prompt",
            self.prompt_callback,
            10
        )

        # 3. 发布检测结果 (JSON 格式的坐标)
        self.pub_results = self.create_publisher(String, "/vlm/results", 10)

        # 4. 发布调试图像 (画框的图)
        self.pub_debug_img = self.create_publisher(Image, "/vlm/debug_image", 10)

        self.get_logger().info(f"VLM 节点已启动。默认寻找: {self.target_labels}")

    def prompt_callback(self, msg):
        """动态修改要寻找的目标"""
        new_labels = [label.strip() for label in msg.data.split(",")]
        self.target_labels = new_labels
        # YOLO-World 特性：可以在运行时重新设置类别
        self.detector.set_classes(self.target_labels)
        self.get_logger().info(f"🔄 目标列表已更新为: {self.target_labels}")

    def image_callback(self, msg):
        """核心处理循环"""
        try:
            # 1. 转换图像 ROS -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            pil_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) # YOLO 需要 RGB
            pil_image = PILImage.fromarray(pil_image)
        except Exception as e:
            self.get_logger().error(f"图像转换错误: {e}")
            return

        # 2. 推理: 检测 (YOLO-World)
        # verbose=False 防止刷屏
        results = self.detector.predict(pil_image, conf=self.conf_threshold, verbose=False)
        
        detections = []
        if len(results) > 0:
            result = results[0]
            boxes = result.boxes.xyxy.cpu().tolist()
            scores = result.boxes.conf.cpu().tolist()
            cls_ids = result.boxes.cls.cpu().tolist()
            names = result.names

            for box, score, cls_id in zip(boxes, scores, cls_ids):
                label = names[int(cls_id)]
                det = DetectionResult(
                    score=score,
                    label=label,
                    box=BoundingBox(
                        xmin=int(box[0]), ymin=int(box[1]),
                        xmax=int(box[2]), ymax=int(box[3])
                    )
                )
                detections.append(det)

        # 3. (可选) 推理: 分割 (SAM)
        # 如果只需要抓取坐标，其实 Box Center 就够了，SAM 会增加计算耗时。
        # 这里为了保持和你提供的功能一致，我加上了。
        if len(detections) > 0:
            detections = self.run_sam(pil_image, detections)

        # 4. 打包结果并发布
        results_json = []
        
        # 用于画图的画布
        debug_img = cv_image.copy()

        for det in detections:
            # --- 数据打包 ---
            center = det.box.center
            obj_data = {
                "label": det.label,
                "score": round(det.score, 2),
                "center_x": center[0],
                "center_y": center[1],
                "bbox": det.box.xyxy
            }
            results_json.append(obj_data)

            # --- 画图 (Debug) ---
            # 画框
            cv2.rectangle(debug_img, (det.box.xmin, det.box.ymin), 
                          (det.box.xmax, det.box.ymax), (0, 255, 0), 2)
            # 画中心点
            cv2.circle(debug_img, (center[0], center[1]), 5, (0, 0, 255), -1)
            # 写字
            cv2.putText(debug_img, f"{det.label}: {center}", 
                        (det.box.xmin, det.box.ymin - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 发布 JSON 字符串
        if results_json:
            msg_str = String()
            msg_str.data = json.dumps(results_json)
            self.pub_results.publish(msg_str)
            self.get_logger().info(f"检测到: {msg_str.data}")

        # 发布 Debug 图像
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            self.pub_debug_img.publish(debug_msg)
        except Exception as e:
            pass

    def run_sam(self, image_pil, detections):
        """运行 SAM 进行分割 (直接复用你的逻辑)"""
        # 提取 boxes
        input_boxes = [[d.box.xyxy for d in detections]] # SAM 需要这种嵌套 list
        
        if not input_boxes or len(input_boxes[0]) == 0:
            return detections

        inputs = self.sam_processor(images=image_pil, input_boxes=input_boxes, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        with torch.no_grad():
            outputs = self.sam_model(**inputs)

        masks = self.sam_processor.post_process_masks(
            masks=outputs.pred_masks,
            original_sizes=inputs["original_sizes"],
            reshaped_input_sizes=inputs["reshaped_input_sizes"]
        )[0]
        
        # 将 mask 存回 detection 对象 (这里简化处理，只做推理不一定要存回)
        # 因为我们主要目的是输出坐标，Mask 主要用于更高级的避障或精细抓取
        return detections

# ==========================================
# 3. 主程序入口
# ==========================================
def main(args=None):
    from PIL import Image # 确保内部能引用
    rclpy.init(args=args)
    
    node = VLMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()