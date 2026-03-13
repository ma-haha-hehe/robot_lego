import numpy as np
import yaml

def normalize(v):
    """归一化向量"""
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError("Vector magnitude too small to normalize.")
    return v / n

def look_at_rotation(cam_pos, target, up=np.array([0, 0, 1], dtype=np.float32)):
    """
    构造旋转矩阵 R_base_cam。
    约定：相机坐标系的 +Z 轴表示“看向前方”的方向（OpenCV/ROS optical frame 常用）。
    """
    # 1. 计算相机 Z 轴（看向目标的方向）
    forward = normalize(target - cam_pos)
    
    # 2. 计算相机 X 轴（右侧方向）
    # 叉积顺序：forward x up -> right (符合右手定则)
    right = normalize(np.cross(forward, up))
    
    # 3. 计算相机 Y 轴（上方方向）
    # 叉积顺序：right x forward -> true_up (确保三个轴互相垂直)
    true_up = np.cross(right, forward)

    # 构建 3x3 旋转矩阵
    R = np.eye(3, dtype=np.float32)
    R[:, 0] = right      # 相机 x 轴在 base 下的投影
    R[:, 1] = true_up    # 相机 y 轴在 base 下的投影
    R[:, 2] = forward    # 相机 z 轴在 base 下的投影
    return R

def make_T(R, t):
    """将旋转矩阵 R 和平移向量 t 组合成 4x4 齐次变换矩阵"""
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

if __name__ == "__main__":
    # ===== 1) 相机在基座坐标系（Base Frame）下的位置（单位：米）=====
    # 假设 Base 坐标系：x 向前, y 向左, z 向上
    cam_x = 0.50    # 前方 50cm
    cam_y = -0.30   # 右侧 30cm -> y 为负
    cam_z = 0.13    # 高度 13cm
    cam_pos = np.array([cam_x, cam_y, cam_z], dtype=np.float32)

    # ===== 2) 定义观察目标（Target）=====
    # 描述：相机位于右前方，看向机械臂的正左方（Base 的 +Y 方向）
    # 增加了一个 -0.25 的 Z 偏移，模拟相机向下俯视桌面的角度
    target = cam_pos + np.array([0.0, 1.0, -0.25], dtype=np.float32)

    # ===== 3) 计算外参矩阵 =====
    R_base_cam = look_at_rotation(cam_pos, target)
    T_base_cam = make_T(R_base_cam, cam_pos)

    # ===== 4) 整理数据并保存为 YAML =====
    out = {
        "base_frame": "panda_link0",
        "camera_frame": "camera_color_optical_frame",
        "T_base_cam": T_base_cam.tolist(),  # 转换为 list 以便 YAML 序列化
        "cam_pos_m": cam_pos.tolist(),
        "target_m": target.tolist(),
        "note": "Rough extrinsic from manual measurement; camera is front-right, looking to robot-left"
    }

    filename = "camera_to_base.yaml"
    try:
        with open(filename, "w", encoding="utf-8") as f:
            yaml.safe_dump(out, f, default_flow_style=False)
        
        print(f"✅ 成功写入文件: {filename}")
        print("\n生成的 T_base_cam 矩阵如下:")
        print(np.array(out["T_base_cam"]))
    except Exception as e:
        print(f"❌ 写入文件失败: {e}")