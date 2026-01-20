import numpy as np
import yaml
import os
import copy
from scipy.spatial.transform import Rotation as R

# ================= 配置区域 =================
TABLE_HEIGHT = 0.42       
ASSEMBLY_CENTER = np.array([0.35, 0.0, TABLE_HEIGHT]) 

# 抓取参数
EDGE_OFFSET_RATIO = 0.35   
MAX_GRIPPER_WIDTH = 0.075  
TOLERANCE = 0.002          

SOURCE_LOCATIONS = {
    "leg_left":   [0.55,  0.25],
    "leg_right":  [0.55,  0.15],
    "hips":       [0.55,  -0.3],
    "torso":      [0.55, -0.15],
    "head":       [0.62,  0.00],
    "hand_left":  [0.62,  0.15],
    "hand_right": [0.62, -0.15]
}

# ================= 基础工具 =================
def to_native(obj):
    if isinstance(obj, (np.integer, int)): return int(obj)
    elif isinstance(obj, (np.floating, float)): return float(obj)
    elif isinstance(obj, (np.ndarray, list)): return [to_native(x) for x in obj]
    else: return obj

def is_blocked(point, self_name, current_scene_blocks):
    px, py, pz = point
    for blk in current_scene_blocks:
        if blk['name'] == self_name: continue
        ox, oy, oz = blk['final_pos']
        # 严格碰撞检测：Z高度重叠 且 平面距离过近
        # 考虑到立柱很高，Z轴容差要放大，只要在垂直方向有重叠就算
        z_dist = abs(pz - oz)
        xy_dist = np.sqrt((px - ox)**2 + (py - oy)**2)
        
        # 碰撞逻辑：如果两个物体高度差小于 3cm (层高)，且平面靠太近
        if z_dist < 0.025: 
            if xy_dist < 0.045: # 4.5cm安全半径
                return True
    return False

def get_quaternion(rpy, spin_90=False):
    base = R.from_euler('x', 180, degrees=True)
    # 试一试 45, -45, 90, 或 -90，直到 RViz 里的箭头对齐方块边
    correction_offset = R.from_euler('z', -45, degrees=True) 
    base = base * correction_offset 
    # 旋转 90 度
    spin = R.from_euler('z', 90 if spin_90 else 0, degrees=True)
    obj_rot = R.from_euler('xyz', rpy, degrees=True)
    final = base *  obj_rot * spin 
    return to_native(final.as_quat().tolist())

# ================= 🔥 核心：候选点生成 (含Z轴逻辑) =================

def generate_candidates(dims, name):
    dx, dy, dz = dims
    candidates = []
    
    # 找出最大轴
    max_dim = max(dx, dy, dz)
    print(f"  Shape Analysis for {name}: {dx}x{dy}x{dz}")
    
    # --- Case 1: 正方体 ---
    if (max_dim - min(dx, dy, dz)) < TOLERANCE:
        print(f"    -> Type: Perfect Cube")
        candidates.append({"offset": [0,0,0], "spin": False, "desc": "Center"})
        return candidates

    # --- Case 2: 站立立柱 (Z 轴最大) ---
    if dz == max_dim:
        print(f"    -> Type: Standing Pillar (Z is max)")
        offset_val = dz * EDGE_OFFSET_RATIO
        
        # 策略：检查 Z轴 的两头 (High/Low)
        # 对于每一个高度，我们还要检查两个进手方向 (Spin 0 和 Spin 90)
        # 因为立柱通常也是方形截面，两个方向都可以抓
        
        # 1. High Grasp (抓上面) - 最安全
        candidates.append({"offset": [0, 0, offset_val], "spin": False, "desc": "High Grasp (Z+) Spin 0"})
        candidates.append({"offset": [0, 0, offset_val], "spin": True,  "desc": "High Grasp (Z+) Spin 90"})
        
        # 2. Low Grasp (抓下面) - 风险高(可能撞桌子)，但在拥挤时很有用
        candidates.append({"offset": [0, 0, -offset_val], "spin": False, "desc": "Low Grasp (Z-) Spin 0"})
        candidates.append({"offset": [0, 0, -offset_val], "spin": True,  "desc": "Low Grasp (Z-) Spin 90"})
        
        # 3. Center (抓中间) - 兜底
        candidates.append({"offset": [0, 0, 0], "spin": False, "desc": "Center Spin 0"})
        candidates.append({"offset": [0, 0, 0], "spin": True,  "desc": "Center Spin 90"})

    # --- Case 3: 躺着的 Y 长条 ---
    elif dy == max_dim:
        print(f"    -> Type: Flat Bar lying on Y")
        offset_val = dy * EDGE_OFFSET_RATIO
        # 抓 Y 轴两头
        candidates.append({"offset": [0, offset_val, 0], "spin": False, "desc": "Edge Y+ (Up)"})
        candidates.append({"offset": [0, -offset_val, 0], "spin": False, "desc": "Edge Y- (Down)"})
        # 抓中间
        if dy < MAX_GRIPPER_WIDTH:
            candidates.append({"offset": [0, 0, 0], "spin": True, "desc": "Center Spin"})

    # --- Case 4: 躺着的 X 长条 ---
    else: # dx == max_dim
        print(f"    -> Type: Flat Bar lying on X")
        offset_val = dx * EDGE_OFFSET_RATIO
        # 抓 X 轴两头
        candidates.append({"offset": [offset_val, 0, 0], "spin": False, "desc": "Edge X+ (Right)"})
        candidates.append({"offset": [-offset_val, 0, 0], "spin": False, "desc": "Edge X- (Left)"})
        # 抓中间
        if dx < MAX_GRIPPER_WIDTH:
            candidates.append({"offset": [0, 0, 0], "spin": True, "desc": "Center Spin"})

    return candidates

# ================= 主流程 =================
def generate_assembly_sequence(blocks_data):
    remaining = copy.deepcopy(blocks_data)
    order = []
    print("\n--- 1. Calculating ABD Sequence ---")
    while len(remaining) > 0:
        max_z = -1.0
        for b in remaining:
            if b['pos'][2] > max_z: max_z = b['pos'][2]
        
        top_layer = [b for b in remaining if abs(b['pos'][2] - max_z) < 0.001]
        top_layer.sort(key=lambda x: x['pos'][1], reverse=True)
        
        for blk in top_layer:
            order.append(blk)
            remaining = [b for b in remaining if b['name'] != blk['name']]
    return order[::-1]

def process(input_yaml, output_yaml):
    if not os.path.exists(input_yaml): print("YAML missing"); return
    with open(input_yaml, 'r') as f: data = yaml.safe_load(f)
    
    seq = generate_assembly_sequence(data.get('blocks', []))
    tasks = []
    current_assembled_blocks = []
    
    print("\n--- 2. Generating Grasp Points ---")
    
    for i, block in enumerate(seq):
        name = block['name']
        dims = block.get('dims', [0.03, 0.03, 0.03])
        local_pos = block.get('pos', [0,0,0])
        rot = block.get('rotation', [0,0,0])
        if isinstance(rot, (int, float)): rot = [0,0,rot]
        
        if name not in SOURCE_LOCATIONS: continue
        
        final_abs_pos = ASSEMBLY_CENTER + np.array(local_pos)
        final_abs_pos[2] = TABLE_HEIGHT + local_pos[2]
        block['final_pos'] = final_abs_pos
        
        # 🔥 生成候选点
        candidates = generate_candidates(dims, name)
        best_cand = None
        
        for cand in candidates:
            try_pos = final_abs_pos + np.array(cand['offset'])
            try_pos[2] = final_abs_pos[2]
            
            if not is_blocked(try_pos, name, current_assembled_blocks):
                best_cand = cand
                print(f"    ✅ Selected: {cand['desc']}")
                print(f"       -> XYZ: [{try_pos[0]:.3f}, {try_pos[1]:.3f}, {try_pos[2]:.3f}]")
                break
            else:
                pass
                
        if not best_cand:
            print("    ⚠️ All Blocked! Fallback.")
            best_cand = {"offset":[0,0,0], "spin":False}
            
        offset = np.array(best_cand['offset'])
        need_spin = best_cand['spin']
        
        src_center = np.array([SOURCE_LOCATIONS[name][0], SOURCE_LOCATIONS[name][1], TABLE_HEIGHT + dims[2]/2])
        pick_pos = src_center + offset
        place_pos = final_abs_pos + offset
        
        pick_orn = get_quaternion([0,0,0], need_spin)
        place_orn = get_quaternion(rot, need_spin)
        
        tasks.append({
            "id": i, "name": name,
            "pick":  {"pos": to_native(pick_pos.tolist()),  "orientation": pick_orn},
            "place": {"pos": to_native(place_pos.tolist()), "orientation": place_orn}
        })
        current_assembled_blocks.append(block)

    with open(output_yaml, 'w') as f:
        yaml.dump({"tasks": tasks}, f, default_flow_style=None)
    print(f"\n✅ Tasks saved to {output_yaml}")

if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    process(os.path.join(base_dir, "final_product.yaml"), 
            os.path.join(base_dir, "tasks.yaml"))