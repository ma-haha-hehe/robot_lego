#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import mujoco
import mujoco_viewer


class VirtualCameraNode(Node):
    def __init__(self):
        super().__init__("virtual_camera")

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)

        # Load your MuJoCo model
        xml_path = "/home/aaa/robot/mj_bridge/scene.xml"
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # viewer for rendering
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

        # Render timer (30 FPS)
        self.timer = self.create_timer(1.0 / 30.0, self.render_callback)

    def render_callback(self):
        # Step simulation a little
        mujoco.mj_step(self.model, self.data)

        # Render from camera "realsense"
        rgb = self.viewer.read_camera_image("realsense")  # 如果你没有此函数，我可以帮你写 render 代码
        rgb = np.asarray(rgb, dtype=np.uint8)

        msg = self.bridge.cv2_to_imgmsg(rgb, encoding="bgr8")
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()