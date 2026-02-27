import numpy as np
import yaml
import os
import copy
from scipy.spatial.transform import Rotation as R

# ================= 1. 配置逻辑 =================
# 碰撞检测依然需要一个虚拟空间来判断，我们假设在局部空间进行
SAFE_RADIUS = 0.045        
SAFE_Z_DIFF = 0.025        
TOLERANCE = 0.002          

# ================= 2. 工具函数 =================

def to_native(obj):
    if isinstance(obj, (np.integer, int)): return int(obj)
    elif isinstance(obj, (np.floating, float)): return float(obj)
    elif isinstance(obj, (np.ndarray, list)): return [to_native(x) for x in obj]
    else: return obj

def is_blocked_local(test_offset, self_name, other_blocks):
    """
    在局部/图纸坐标系下检查碰撞
    test_offset: 抓取点相对于该积木中心的偏移
    """
    tx, ty, tz = test_offset
    # 这里简单模拟：检查抓取点是否会撞到图纸中已有的其它积木
    # 实际项目中，这里应结合 place 坐标进行逻辑判断
    return False 

def get_action_quaternion(yaw_angle=0.0):
    """
    生成动作姿态四元数。
    注意：这里的四元数是相对于物体坐标系的。
    """
    # 基础姿态：夹爪垂直向下
    base = R.from_euler('x', 180, degrees=True) * R.from_euler('z', -45, degrees=True)
    spin = R.from_euler('z', yaw_angle, degrees=True)
    final = base * spin 
    return to_native(final.as_quat().tolist())

# ================= 3. 抓取候选生成 =================

def generate_candidates(dims):
    """根据积木尺寸生成相对于中心点的偏移组"""
    dx, dy, dz = dims
    candidates = []
    
    # 针对 2x4 这种长条积木的典型偏移逻辑
    # 0.4 倍率代表往边缘挪一点
    x_off = dx * 0.4
    y_off = dy * 0.4
    
    # 优先级 1: 抓长边中心 (Yaw 90 或 0 视具体朝向)
    candidates.append({"offset": [0, 0, 0], "yaw": 90, "desc": "Center Side"})
    
    # 优先级 2: 抓两头
    candidates.append({"offset": [x_off, 0, 0], "yaw": 0, "desc": "Edge X+"})
    candidates.append({"offset": [-x_off, 0, 0], "yaw": 0, "desc": "Edge X-"})
    
    return candidates

# ================= 4. 主干逻辑 =================

def process(input_yaml, output_yaml):
    if not os.path.exists(input_yaml):
        print(f"❌ 找不到输入文件: {input_yaml}")
        return
        
    with open(input_yaml, 'r') as f: 
        data = yaml.safe_load(f)
    
    # 获取原始积木列表
    blocks = copy.deepcopy(data.get('blocks', []))
    
    # 依然需要根据高度排序，确定装配顺序
    blocks.sort(key=lambda x: x['pos'][2]) 
    
    tasks = []
    
    print("🚀 正在生成几何动作序列...")
    for i, blk in enumerate(blocks):
        name = blk['name']
        dims = blk.get('dims', [0.03, 0.03, 0.03])
        
        # 1. 获取候选偏移点
        candidates = generate_candidates(dims)
        
        # 2. 简单挑选（这里你可以加入 is_blocked_local 的逻辑）
        best_cand = candidates[0] 
        
        # 3. 构建任务项
        # pick: 仅存储相对于物体中心的偏移 [dx, dy, dz]
        # place: 存储 final_product.yaml 里的绝对坐标 [x, y, z] + 偏移
        
        raw_pos = blk['pos'] # 图纸里的原始坐标
        offset = best_cand['offset']
        
        # 计算带偏移的放置坐标 (图纸坐标系下)
        final_place_pos = [
            raw_pos[0] + offset[0],
            raw_pos[1] + offset[1],
            raw_pos[2] + offset[2]
        ]

        tasks.append({
            "id": i,
            "name": name,
            "strategy": best_cand['desc'],
            "pick": {
                "pos": to_native(offset), # 相对于物体中心的偏移
                "orientation": get_action_quaternion(best_cand['yaw'])
            },
            "place": {
                "pos": to_native(final_place_pos), # 图纸坐标系下的绝对坐标
                "orientation": get_action_quaternion(best_cand['yaw'])
            }
        })
        print(f"  [OK] {name} -> 策略: {best_cand['desc']}")

    with open(output_yaml, 'w') as f:
        yaml.dump({"tasks": tasks}, f, sort_keys=False)
    print(f"\n✅ 纯几何任务清单已生成: {output_yaml}")

if __name__ == "__main__":
    process("final_product.yaml", "tasks.yaml")