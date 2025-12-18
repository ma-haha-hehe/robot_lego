#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import mujoco
import mujoco_viewer

# Panda 的 7 个主关节名字（和 MoveIt 中保持一致）
JOINTS = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]


class MjBridgeNode(Node):
    def __init__(self):
        super().__init__("mj_bridge_node")

        # 1. MuJoCo 模型路径（改成你自己的绝对路径）
        xml_path = "/home/aaa/robot/ros2_ws/src/mj_bridge/mj_bridge/scene.xml"
        self.get_logger().info(f"[mj_bridge] Loading MuJoCo model from: {xml_path}")

        # 2. 加载 MuJoCo 模型
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

        # 3. 预计算：每个关节在 qpos 里的索引
        self.qpos_id = []
        for name in JOINTS:
            j = self.model.joint(name)       # 根据名字找到这个 joint 对象
            idx = j.qposadr[0]               # 这个 joint 使用的 qpos 起始索引
            self.qpos_id.append(idx)
            self.get_logger().info(f"[mj_bridge] Joint {name} -> qpos index {idx}")

        # 4. 保存最新关节角的字典（初始全部置 0）
        self.joint_pos = {name: 0.0 for name in JOINTS}

        # 一个小标志，用来只打印一次调试信息
        self._printed_once = False

        # 5. 订阅 /joint_states
        self.sub_joint = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10,
        )

        # 6. 定时器：周期性地把最新的关节角写进 MuJoCo，然后渲染
        self.timer = self.create_timer(0.01, self.update_mujoco)

        self.get_logger().info(
            "[mj_bridge] MjBridgeNode initialized, waiting for joint states..."
        )

    # 订阅回调：每当 /joint_states 有新消息，就更新 joint_pos 字典
    def joint_state_cb(self, msg: JointState):
        # 只在第一次收到的时候打印一下前几个关节，方便你确认名字
        if not self._printed_once:
            self._printed_once = True
            self.get_logger().info(
                f"[mj_bridge] Received JointState, first 5 joints: "
                f"{list(msg.name[:5])} -> {list(msg.position[:5])}"
            )

        # 更新我们关心的 7 个主关节
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_pos:
                self.joint_pos[name] = pos

    # 定时器回调：把 joint_pos 写入 MuJoCo，然后前向计算 + 渲染
    def update_mujoco(self):
        # 把 joint_pos 里的角度写进 qpos
        for name, idx in zip(JOINTS, self.qpos_id):
            self.data.qpos[idx] = self.joint_pos[name]

        # 不走动力学积分，只做前向运动学计算
        mujoco.mj_forward(self.model, self.data)

        # 渲染到窗口
        self.viewer.render()


def main(args=None):
    rclpy.init(args=args)
    node = MjBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
