#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import yaml
import cv2
import torch
import numpy as np
from PIL import Image

# 解决 SciPy 兼容性警告
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import pyrealsense2 as rs
from my_robot_interfaces.srv import GetBlockPose

# 注入 Torch 库路径
T_LIB = "/home/i6user/.local/lib/python3.10/site-packages/torch/lib"
if os.path.exists(T_LIB):
    os.environ['LD_LIBRARY_PATH'] = f"{T_LIB}:{os.environ.get('LD_LIBRARY_PATH', '')}"

class LegoVisionService(Node):
    def __init__(self):
        super().__init__('lego_vision_service')
        self.dino_ready = False
        self.fp_ready = False

        # 1. 路径初始化
        try:
            share_dir = get_package_share_directory('my_robot_vision')
        except:
            share_dir = "/home/i6user/Desktop/robot_lego/install/my_robot_vision/share/my_robot_vision"
        
        # FoundationPose 仓库挂载
        fp_repo = "/home/i6user/Desktop/robot_lego/src/my_robot_vision/FoundationPose"
        if fp_repo not in sys.path:
            sys.path.append(fp_repo)
            sys.path.append(os.path.join(fp_repo, "root"))

        # 2. 初始化 GroundingDINO
        self.get_logger().info("正在初始化 GroundingDINO...")
        try:
            # 必须注入 torch 命名空间
            import transformers
            transformers.pipelines.torch = torch
            
            from transformers import AutoProcessor, GroundingDinoForObjectDetection
            model_id = "IDEA-Research/grounding-dino-tiny"
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            
            self.processor = AutoProcessor.from_pretrained(model_id)
            self.model = GroundingDinoForObjectDetection.from_pretrained(model_id).to(self.device)
            self.dino_ready = True
            self.get_logger().info("✅ GroundingDINO 加载成功")
        except Exception as e:
            self.get_logger().error(f"❌ DINO 加载失败: {e}")

        # 3. 初始化 FoundationPose (带包名检查)
        try:
            import nvdiffrast.torch as dr
            from estimater import ScorePredictor, PoseRefinePredictor
            self.scorer = ScorePredictor()
            self.refiner = PoseRefinePredictor()
            self.fp_ready = True
            self.get_logger().info("✅ FoundationPose 加载成功")
        except Exception as e:
            self.get_logger().warn(f"⚠️ FoundationPose 暂时不可用: {e}")

        # 4. 相机启动
        try:
            self.pipeline = rs.pipeline()
            self.pipeline.start(rs.config())
            self.align = rs.align(rs.stream.color)
            self.get_logger().info("✅ RealSense 相机已就绪")
        except Exception as e:
            self.get_logger().error(f"❌ 相机故障: {e}")

        self.srv = self.create_service(GetBlockPose, 'get_block_pose', self.handle_get_pose)
        self.get_logger().info("🚀 视觉服务已完全就绪")

    def handle_get_pose(self, request, response):
        self.get_logger().info(f"收到请求: {request.block_name}")
        response.success = True
        response.real_pose.position.z = 0.2
        response.real_pose.orientation.w = 1.0
        return response

def main():
    rclpy.init()
    node = LegoVisionService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 修正 shutdown 重复调用问题
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()