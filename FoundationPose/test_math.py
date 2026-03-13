import numpy as np
from scipy.spatial.transform import Rotation as R

# 物理参数
C_POS = [0.51, -0.37, 0.11] # 前, 右, 高
TILT = 20

def test_all_configs(v_x, v_y, v_z):
    # 视觉输入 (来自你的截图 image_c7b0b9.png)
    t_obj_cam = np.eye(4)
    t_obj_cam[0:3, 3] = [v_x, v_y, v_z]

    # 方案列表
    configs = {
        "方案 A (当前): X->X, Z->Y, -Y->Z": np.array([[1,0,0],[0,0,1],[0,-1,0]]),
        "方案 B (镜像): X->X, -Z->Y, Y->Z": np.array([[1,0,0],[0,0,-1],[0,1,0]]),
        "方案 C (标准): Z->X, -X->Y, -Y->Z": np.array([[0,0,1],[-1,0,0],[0,-1,0]]),
        "方案 D (侧翻): Z->X, Y->Y, -X->Z": np.array([[0,0,1],[0,1,0],[-1,0,0]])
    }

    for name, r_align in configs.items():
        T = np.eye(4)
        T[0:3, 3] = C_POS
        # 俯仰补偿 (尝试绕相机 X 轴)
        r_tilt = R.from_euler('x', TILT, degrees=True).as_matrix()
        T[0:3, 0:3] = r_align @ r_tilt
        
        res = T @ t_obj_cam
        pos = res[0:3, 3]
        print(f"{name} -> 机器人坐标: X={pos[0]:.3f}, Y={pos[1]:.3f}, Z={pos[2]:.3f}")

# 使用你截图中的数值
test_all_configs(-0.0032, 0.0207, 0.3072)