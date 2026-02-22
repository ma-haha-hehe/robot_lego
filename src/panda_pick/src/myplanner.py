import numpy as np
import yaml
import os
import copy
from scipy.spatial.transform import Rotation as R

# ================= 1. 全局物理配置 =================
TABLE_HEIGHT = 0.0         # 机械臂在桌面上，桌面即 Z=0 平面
MIN_Z_CLEARANCE = 0.01     # 夹爪距离桌面的最小安全高度(1cm)

# 碰撞排查参数
EDGE_OFFSET_RATIO = 0.40   
BOTTOM_BIAS_RATIO = -0.35  
SAFE_RADIUS = 0.045        # 夹爪安全半径 (4.5cm)
SAFE_Z_DIFF = 0.025        # 垂直重叠容差 (2.5cm)
TOLERANCE = 0.002          

# ================= 2. 坐标系矩阵转换 =================

def get_hand_eye_matrix():
    """构建 相机 -> 机械臂底座 的 4x4 变换矩阵"""
    T_cam_to_base = np.eye(4)
    T_cam_to_base[0:3, 3] = [0.35, 0.0, 0.20] 

    # 将标准相机(Z前Y下X右)转为机械臂底座(X前Y左Z上)
    R_base_to_cam_standard = R.from_matrix([
        [ 0,  0,  1],
        [-1,  0,  0],
        [ 0, -1,  0]
    ])
    R_tilt = R.from_euler('x', 20, degrees=True)
    T_cam_to_base[0:3, 0:3] = (R_base_to_cam_standard * R_tilt).as_matrix()
    return T_cam_to_base

def get_assembly_base_matrix():
    """构建 成品图纸 -> 机械臂底座 的 4x4 变换矩阵"""
    T_blueprint_to_base = np.eye(4)
    T_blueprint_to_base[0:3, 3] = [0.40, 0.0, TABLE_HEIGHT] 
    T_blueprint_to_base[0:3, 0:3] = R.from_euler('z', 0, degrees=True).as_matrix()
    return T_blueprint_to_base

# ================= 3. 基础工具与碰撞检测 =================

def to_native(obj):
    if isinstance(obj, (np.integer, int)): return int(obj)
    elif isinstance(obj, (np.floating, float)): return float(obj)
    elif isinstance(obj, (np.ndarray, list)): return [to_native(x) for x in obj]
    else: return obj

def transform_local_to_world(local_offset, world_pos, world_rot_matrix):
    rotated_offset = np.dot(world_rot_matrix, local_offset)
    return np.array(world_pos) + rotated_offset

def is_blocked(point, self_name, obstacle_list, pos_key):
    px, py, pz = point
    if pz < (TABLE_HEIGHT + MIN_Z_CLEARANCE):
        return True
        
    for blk in obstacle_list:
        if blk['name'] == self_name: continue
        ox, oy, oz = blk[pos_key]
        z_dist = abs(pz - oz)
        xy_dist = np.sqrt((px - ox)**2 + (py - oy)**2)
        if z_dist < SAFE_Z_DIFF and xy_dist < SAFE_RADIUS: 
            return True
    return False

def get_quaternion(rot_matrix_3x3, yaw_angle=0.0):
    """生成机械臂末端执行器的四元数姿态"""
    # 默认机械臂末端向下(绕X转180)，并根据需求微调
    base = R.from_euler('x', 180, degrees=True) * R.from_euler('z', -45, degrees=True)
    spin = R.from_euler('z', yaw_angle, degrees=True)
    obj_rot = R.from_matrix(rot_matrix_3x3)
    final = base * obj_rot * spin 
    return to_native(final.as_quat().tolist())

# ================= 4. 候选抓取点生成 =================

def generate_candidates(dims):
    dx, dy, dz = dims
    candidates = []
    if (max(dx, dy, dz) - min(dx, dy, dz)) < TOLERANCE:
        for angle in range(0, 360, 45):
            candidates.append({"offset": [0,0,0], "yaw": angle, "desc": f"Cube Center {angle}°"})
        return candidates

    bottom_z = dz * BOTTOM_BIAS_RATIO
    x_off = dx * EDGE_OFFSET_RATIO; y_off = dy * EDGE_OFFSET_RATIO; z_off = dz * EDGE_OFFSET_RATIO
    
    for a in [0, 90]:
        candidates.append({"offset": [x_off, 0, bottom_z], "yaw": a, "desc": f"Face X+ {a}°"})
        candidates.append({"offset": [-x_off, 0, bottom_z], "yaw": a, "desc": f"Face X- {a}°"})
        candidates.append({"offset": [0, y_off, bottom_z], "yaw": a, "desc": f"Face Y+ {a}°"})
        candidates.append({"offset": [0, -y_off, bottom_z], "yaw": a, "desc": f"Face Y- {a}°"})
    return candidates

# ================= 5. 主干逻辑 =================

def process(input_yaml, output_yaml, foundation_poses):
    if not os.path.exists(input_yaml):
        print(f"❌ 找不到输入文件: {input_yaml}")
        return
        
    with open(input_yaml, 'r') as f: data = yaml.safe_load(f)
    
    T_cam_to_base = get_hand_eye_matrix()
    T_blueprint_to_base = get_assembly_base_matrix()
    
    blocks = copy.deepcopy(data.get('blocks', []))
    blocks.sort(key=lambda x: x['pos'][2]) 
    
    tasks = []
    source_blocks = []
    assembled_blocks = []
    
    for blk in blocks:
        name = blk['name']
        if name not in foundation_poses: continue
        
        # Pick位姿转换
        T_obj_cam = foundation_poses[name]
        T_obj_base_init = np.dot(T_cam_to_base, T_obj_cam)
        blk['pick_pos'] = T_obj_base_init[0:3, 3].tolist()
        blk['pick_rot_mat'] = T_obj_base_init[0:3, 0:3]
        
        # Place位姿转换
        T_obj_blueprint = np.eye(4)
        T_obj_blueprint[0:3, 3] = blk.get('pos', [0,0,0])
        rot_val = blk.get('rotation', [0,0,0])
        if isinstance(rot_val, (int, float)): rot_val = [0,0,rot_val]
        T_obj_blueprint[0:3, 0:3] = R.from_euler('xyz', rot_val, degrees=True).as_matrix()
        
        T_obj_base_final = np.dot(T_blueprint_to_base, T_obj_blueprint)
        blk['place_pos'] = T_obj_base_final[0:3, 3].tolist()
        blk['place_rot_mat'] = T_obj_base_final[0:3, 0:3]
        
        source_blocks.append(blk)

    print("\n🚀 开始双端联合排查...")
    for i, block in enumerate(source_blocks.copy()):
        name = block['name']
        dims = block.get('dims', [0.03, 0.03, 0.03])
        candidates = generate_candidates(dims)
        best_cand = None
        
        for cand in candidates:
            place_try_pos = transform_local_to_world(cand['offset'], block['place_pos'], block['place_rot_mat'])
            if not is_blocked(place_try_pos, name, assembled_blocks, 'place_pos'):
                pick_try_pos = transform_local_to_world(cand['offset'], block['pick_pos'], block['pick_rot_mat'])
                if not is_blocked(pick_try_pos, name, source_blocks, 'pick_pos'):
                    best_cand = cand
                    print(f"[{name}] ✅ 验证通过: {cand['desc']}")
                    break
                    
        if not best_cand:
            print(f"[{name}] ⚠️ Fallback至默认中心。")
            best_cand = {"offset":[0,0,0], "yaw": 0}
            pick_try_pos = transform_local_to_world([0,0,0], block['pick_pos'], block['pick_rot_mat'])
            place_try_pos = transform_local_to_world([0,0,0], block['place_pos'], block['place_rot_mat'])

        tasks.append({
            "id": i, "name": name,
            "pick":  {"pos": to_native(pick_try_pos.tolist()),  "orientation": get_quaternion(block['pick_rot_mat'], best_cand['yaw'])},
            "place": {"pos": to_native(place_try_pos.tolist()), "orientation": get_quaternion(block['place_rot_mat'], best_cand['yaw'])}
        })
        assembled_blocks.append(block)

    with open(output_yaml, 'w') as f:
        yaml.dump({"tasks": tasks}, f, sort_keys=False)
    print(f"\n✅ 任务清单生成成功: {output_yaml}")

if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(base_dir, "final_product.yaml")
    output_file = os.path.join(base_dir, "tasks4.yaml")

    # 1. 自动创建一个模拟的成品图纸文件 (如果没有的话)
    if not os.path.exists(input_file):
        mock_product = {
            "blocks": [
                {"name": "base_link", "pos": [0, 0, 0.015], "dims": [0.03, 0.03, 0.03]},
                {"name": "top_link", "pos": [0, 0, 0.045], "dims": [0.03, 0.03, 0.03]}
            ]
        }
        with open(input_file, 'w') as f:
            yaml.dump(mock_product, f)

    # 2. 模拟 FoundationPose 视觉识别结果
    mock_fp_results = {}
    for n in ["base_link", "top_link"]:
        T = np.eye(4)
        T[0:3, 3] = [0.0, 0.0, 0.40] # 假设积木在相机下方 40cm 处
        mock_fp_results[n] = T
    
    # 3. 执行
    process(input_file, output_file, mock_fp_results)