import numpy as np
from scipy.spatial.transform import Rotation as R

# 填入你目前的物理参数
CAM_X = 0.51  # 前
CAM_Y = -0.37 # 右
CAM_Z = 0.11  # 高
TILT_DEG = 20 # 低头度数

def check_transform(v_x, v_y, v_z):
    # 1. 构造变换矩阵 (完全模拟 Executor 逻辑)
    T_CAM_TO_BASE = np.eye(4)
    T_CAM_TO_BASE[0:3, 3] = [CAM_X, CAM_Y, CAM_Z]
    
    # 对齐矩阵：相机Z->前，相机X->左，相机Y->下
    r_align = np.array([[0,0,1], [-1,0,0], [0,-1,0]])
    r_tilt = R.from_euler('x', TILT_DEG, degrees=True).as_matrix()
    T_CAM_TO_BASE[0:3, 0:3] = r_align @ r_tilt

    # 2. 模拟视觉输入的平移部分 (t_obj_cam)
    t_obj_cam = np.eye(4)
    t_obj_cam[0:3, 3] = [v_x, v_y, v_z]
    
    # 3. 计算结果
    res = T_CAM_TO_BASE @ t_obj_cam
    pos = res[0:3, 3]
    
    print(f"--- 测试输入 (相机视角) ---")
    print(f"物体在相机前: {v_z}m, 横向: {v_x}m, 垂直: {v_y}m")
    print(f"--- 计算结果 (机器人底座视角) ---")
    print(f"机器人 X (前后): {pos[0]:.4f}m")
    print(f"机器人 Y (左右): {pos[1]:.4f}m")
    print(f"机器人 Z (高度): {pos[2]:.4f}m")

# 使用你 vision_output.yaml 里的真实数字测试一下
# 示例：X=-0.0179, Y=0.0266, Z=0.2968
check_transform(-0.0179, 0.0266, 0.2968)