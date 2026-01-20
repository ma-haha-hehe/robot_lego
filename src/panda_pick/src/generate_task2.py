import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import yaml
import os

# ================= 配置 =================
SCALE_FACTOR = 100  # m -> cm
OFFSET_VAL = 0.5    # 抓取点距离表面的缓冲距离 (cm)

# Franka Panda 的常用下抓取姿态 (四元数 x, y, z, w)
# 1. 默认朝下 (手指平行于 Y 轴) -> 对应抓取 X 轴宽度的物体
ORN_DOWN_DEFAULT = [1.0, 0.0, 0.0, 0.0] 
# 2. 旋转90度朝下 (手指平行于 X 轴) -> 对应抓取 Y 轴宽度的物体
ORN_DOWN_ROTATED = [0.707, 0.707, 0.0, 0.0]

# ================= 碰撞检测逻辑 =================
def is_point_blocked(point, all_blocks, self_id):
    px, py, pz = point
    tolerance = 0.2
    for block in all_blocks:
        if block['id'] == self_id: continue
        bx_min, bx_max = block['bounds_x']
        by_min, by_max = block['bounds_y']
        bz_min, bz_max = block['bounds_z']
        if (bx_min + tolerance < px < bx_max - tolerance) and \
           (by_min + tolerance < py < by_max - tolerance) and \
           (bz_min + tolerance < pz < bz_max - tolerance):
            return True
    return False

def check_pair_availability(p1, p2, all_blocks, self_id):
    return not (is_point_blocked(p1, all_blocks, self_id) or is_point_blocked(p2, all_blocks, self_id))

# ================= 主处理逻辑 =================
def process_layout(yaml_path):
    if not os.path.exists(yaml_path):
        print(f"[Error] Not found: {yaml_path}"); return []
    
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    raw_blocks = data.get("blocks", [])
    
    processed_blocks = []
    
    # 1. 初始化数据
    for i, item in enumerate(raw_blocks):
        dims = np.array(item.get("dims", [0.03, 0.03, 0.03])) * SCALE_FACTOR
        pos = np.array(item.get("pos", [0, 0, 0])) * SCALE_FACTOR
        origin = pos - dims / 2
        
        processed_blocks.append({
            "id": i,
            "name": item.get("name", f"block_{i}"),
            "dims": dims,
            "center": pos,
            "origin": origin,
            "bounds_x": (origin[0], origin[0] + dims[0]),
            "bounds_y": (origin[1], origin[1] + dims[1]),
            "bounds_z": (origin[2], origin[2] + dims[2]),
            "rotate_gripper_flag": True, # 标记是否需要旋转夹爪
            "final_grasp": None
        })

    # 2. 判定抓取策略
    for block in processed_blocks:
        dx, dy, _ = block['dims']
        cx, cy, cz = block['center']
        
        # X轴方向点 (左右抓) -> 夹爪需要旋转 (手指朝Y) 其实是默认姿态
        x_points = (np.array([cx - dx/2 - OFFSET_VAL, cy, cz]), np.array([cx + dx/2 + OFFSET_VAL, cy, cz]))
        # Y轴方向点 (前后抓) -> 夹爪不需要旋转 (手指朝X)
        y_points = (np.array([cx, cy - dy/2 - OFFSET_VAL, cz]), np.array([cx, cy + dy/2 + OFFSET_VAL, cz]))

        # 策略：总是优先尝试抓长边
        if dx >= dy:
            pairs = [(x_points, True), (y_points, False)] # (点对, 是否是X轴方向)
        else:
            pairs = [(y_points, False), (x_points, True)]

        for pts, is_x_axis in pairs:
            if check_pair_availability(pts[0], pts[1], processed_blocks, block['id']):
                block['final_grasp'] = {'p1': pts[0], 'p2': pts[1]}
                # 逻辑修正：
                # 如果抓取点连线是 X 轴 (左右抓)，夹爪手指要垂直于 X，即平行于 Y。这是 Panda 的默认方向。
                # 如果抓取点连线是 Y 轴 (前后抓)，夹爪手指要平行于 X。这需要旋转 90 度。
                block['rotate_gripper_flag'] = not is_x_axis 
                break
        
        if not block['final_grasp']:
            print(f"[Warn] Block {block['id']} 无法安全抓取！")

    return processed_blocks

# ================= 导出为 tasks.yaml =================
def export_tasks_to_yaml(blocks, output_file="tasks.yaml"):
    tasks = []
    # 原料区起始位置 (米)
    source_x = 0.3
    source_y_start = 0.4
    source_z = 0.435 # 假设放在桌子上 (0.42 + 0.015)

    for i, block in enumerate(blocks):
        if block['final_grasp'] is None: continue

        # 1. 放置目标 (米)
        place_pos = block['center'] / 100.0
        
        # 2. 放置姿态
        # 如果 rotate_gripper_flag 为 True，说明要抓短边/Y轴边，需要旋转
        place_orn = ORN_DOWN_ROTATED if block['rotate_gripper_flag'] else ORN_DOWN_DEFAULT

        # 3. 抓取源 (米) - 模拟排成一排
        pick_pos = [source_x, source_y_start - (i * 0.1), source_z]
        
        task = {
            "id": block['id'],
            "name": block['name'],
            "pick": {
                "pos": [float(x) for x in pick_pos],
                "orientation": ORN_DOWN_DEFAULT, # 原料区假设都是整齐平放的
            },
            "place": {
                "pos": [float(x) for x in place_pos],
                "orientation": place_orn
            }
        }
        tasks.append(task)

    with open(output_file, 'w') as f:
        yaml.dump({"tasks": tasks}, f, default_flow_style=None)
    print(f"✅ 成功导出任务清单: {os.path.abspath(output_file)}")

if __name__ == "__main__":
    output_path = "/home/aaa/robot/ros2_ws/src/panda_pick/src/tasks.yaml"
    blocks = process_layout("/home/aaa/robot/ros2_ws/src/panda_pick/src/final_product.yaml")
    if blocks:
        export_tasks_to_yaml(blocks, output_path)
        # visualize_logic(blocks) # 可选：调用你之前的可视化函数