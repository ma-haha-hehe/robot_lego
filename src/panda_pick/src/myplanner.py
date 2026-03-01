#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import yaml
import os
import copy
from scipy.spatial.transform import Rotation as R

# 禁用 YAML 锚点/别名功能，确保输出完整的列表数据
yaml.Dumper.ignore_aliases = lambda *args: True

# ================= 1. 配置区域 =================
TABLE_HEIGHT = 0.42       
ASSEMBLY_CENTER = np.array([0.35, 0.0, TABLE_HEIGHT]) 
SAFE_RADIUS = 0.045       

SOURCE_LOCATIONS = {
    "base_4x2_lvl1":    [0.55,  0.25],
    "support_2x2_left": [0.55,  0.10],
    "support_2x2_right":[0.55, -0.10],
    "mid_4x2_lvl3":     [0.55, -0.25],
    "top_4x2_lvl4":     [0.62,  0.00]
}

# ================= 2. 工具函数 =================
def to_native(obj):
    if isinstance(obj, (np.integer, int)): return int(obj)
    elif isinstance(obj, (np.floating, float)): return float(obj)
    elif isinstance(obj, (np.ndarray, list)): return [to_native(x) for x in obj]
    else: return obj

def get_quaternion(obj_yaw_deg, grasp_spin_90=False):
    base = R.from_euler('x', 180, degrees=True)
    spin = R.from_euler('z', 90 if grasp_spin_90 else 0, degrees=True)
    obj_rot = R.from_euler('z', obj_yaw_deg, degrees=True)
    final = base * obj_rot * spin 
    return to_native(final.as_quat().tolist())

# ================= 3. 核心：逆向拆卸逻辑 =================

def check_accessibility(test_pos, scene_blocks, self_name):
    """ 在拆卸过程中检查：夹爪在 test_pos 处是否会撞到场景中“还没被拆掉”的其他积木 """
    for blk in scene_blocks:
        if blk['name'] == self_name: continue
        ox, oy, oz = blk['abs_pos']
        dist_xy = np.sqrt((test_pos[0] - ox)**2 + (test_pos[1] - oy)**2)
        # 只要 Z 轴有重叠或在上方，且平面距离太近，就认为会碰撞
        if abs(test_pos[2] - oz) < 0.03 and dist_xy < SAFE_RADIUS:
            return False
    return True

def process_blueprint(input_yaml, output_yaml):
    with open(input_yaml, 'r') as f:
        blueprint = yaml.safe_load(f)

    # 初始状态：所有积木都在场
    all_blocks = []
    for b in blueprint.get('blocks', []):
        b['abs_pos'] = ASSEMBLY_CENTER + np.array(b['pos'])
        b['abs_pos'][2] = TABLE_HEIGHT + b['pos'][2]
        all_blocks.append(b)

    # 存放拆卸顺序的列表
    disassembly_tasks = []
    remaining_blocks = copy.deepcopy(all_blocks)

    print("--- 开始逆向拆卸规划 (从上往下) ---")
    
    while len(remaining_blocks) > 0:
        # 1. 寻找当前剩余积木中最顶层的（Z 最大的）
        remaining_blocks.sort(key=lambda x: x['abs_pos'][2], reverse=True)
        target = remaining_blocks[0]
        name = target['name']
        
        # 2. 尝试 0 度和 90 度抓取方案
        # 在“当前所有积木都在”的情况下检查可达性
        need_spin_90 = False
        if not check_accessibility(target['abs_pos'], remaining_blocks, name):
            need_spin_90 = True
            print(f"  [拆卸] {name}: 0度受阻，切换90度")
        else:
            print(f"  [拆卸] {name}: 0度可达")

        # 3. 记录这个动作
        rot_rpy = target.get('rotation', [0, 0, 0])
        obj_yaw = rot_rpy[2] if isinstance(rot_rpy, list) else rot_rpy
        common_orn = get_quaternion(obj_yaw, need_spin_90)
        
        src_xy = SOURCE_LOCATIONS.get(name, [0.5, 0.0])
        dims = target.get('dims', [0.032, 0.032, 0.03])
        
        disassembly_tasks.append({
            "name": name,
            "pick": {
                "pos": to_native([src_xy[0], src_xy[1], TABLE_HEIGHT + dims[2]/2]),
                "orientation": common_orn
            },
            "place": {
                "pos": to_native(target['abs_pos'].tolist()),
                "orientation": common_orn
            },
            "relative_offset": [0, 0, 0]
        })

        # 4. “拆掉”这块积木，继续规划剩下的
        remaining_blocks.pop(0)

    # --- 关键：将拆卸序列反转，得到装配序列 ---
    assembly_tasks = disassembly_tasks[::-1]
    for i, task in enumerate(assembly_tasks):
        task['id'] = i  # 重新分配 ID

    # 保存结果，禁止别名
    with open(output_yaml, 'w') as f:
        yaml.dump({"tasksh": assembly_tasks}, f, default_flow_style=None, sort_keys=False)
    
    print(f"\n✅ 逆向规划完成！输出文件: {output_yaml}")

if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(base_dir, "final_product_simple.yaml")
    output_file = os.path.join(base_dir, "tasksh.yaml")
    process_blueprint(input_file, output_file)