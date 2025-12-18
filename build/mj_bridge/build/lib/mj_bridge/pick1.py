import mujoco
import numpy as np
import numpy as np
from geometry_msgs.msg import PoseStamped
import transforms3d.euler

def euler_from_quaternion(quaternion):
    """
    将四元数转换为欧拉角 (roll, pitch, yaw)。
    兼容 tf_transformations 的输入格式 [x, y, z, w]。
    """
    # 1. 解包 ROS 格式的四元数 (x, y, z, w)
    x, y, z, w = quaternion
    
    # 2. 转换为 transforms3d 需要的格式 (w, x, y, z)
    # 注意：transforms3d 把实部 w 放在第一位
    return transforms3d.euler.quat2euler([w, x, y, z])

model = mujoco.MjModel.from_xml_path("/home/aaa/robot/ros2_ws/src/mj_bridge/mj_bridge/scene.xml")
data = mujoco.MjData(model)

# 先前向一次，保证 xform 有效
mujoco.mj_forward(model, data)

# 找到 brick1 的 body id
brick1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "brick1")

# 砖块质心在世界坐标系下的位置
brick1_pos = data.xpos[brick1_id].copy()   # [x, y, z]
print("brick1_pos:", brick1_pos)

# 砖块中心
bx, by, bz = brick1_pos

# 砖块高度大约 0.02m，因此顶部大概在 bz + 0.01
brick_top_z = bz + 0.01

# 让夹爪 z 轴朝 -Z，x 轴沿世界 X（你可以之后改）
# quat = tft.quaternion_from_euler(np.pi, 0, 0)  # 绕 X 轴转 180°，z 朝下
w, x, y, z = transforms3d.euler.euler2quat(np.pi, 0, 0)
quat = [x, y, z, w]  # 转回 ROS 格式

# pre-grasp
pre_grasp = PoseStamped()
pre_grasp.header.frame_id = "panda_link0"  # 或你的 base_link
pre_grasp.pose.position.x = bx
pre_grasp.pose.position.y = by
pre_grasp.pose.position.z = brick_top_z + 0.10   # 高 10cm
pre_grasp.pose.orientation.x = quat[0]
pre_grasp.pose.orientation.y = quat[1]
pre_grasp.pose.orientation.z = quat[2]
pre_grasp.pose.orientation.w = quat[3]

# grasp
grasp = PoseStamped()
grasp.header.frame_id = "panda_link0"
grasp.pose.position.x = bx
grasp.pose.position.y = by
grasp.pose.position.z = brick_top_z + 0.01  # 距离砖块表面 1cm
grasp.pose.orientation = pre_grasp.pose.orientation