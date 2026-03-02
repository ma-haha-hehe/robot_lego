#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import yaml
import os
import copy
from scipy.spatial.transform import Rotation as R

# 禁用 YAML 锚点，确保输出格式纯净，不含 &id 等符号
yaml.Dumper.ignore_aliases = lambda *args: True

# ================= 1. 配置区域 =================
# 仅用于内部避障逻辑计算，不输出到 tasks.yaml 的 place 部分
TABLE_HEIGHT = 0.42       
INTERNAL_ASSEMBLY_CENTER = np.array([0.35, 0.0, TABLE_HEIGHT]) 
SAFE_RADIUS = 0.045       

# 原材料区固定位置（Pick 的基准）
SOURCE_LOCATIONS = {
    "base_4x2_lvl1":    [0.55,  0.25],
    "support_2x2_left": [0.55,  0.10],
    "support_2x2_right":[0.55, -0.10],
    "mid_4x2_lvl3":     [0.55, -0.25],
    "top_4x2_lvl4":     [0.62,  0.00]
}

# ================= 2. 工具函数 =================
def to_native(obj):
    """ 将数据转换为 YAML 可写的原生类型 """
    if isinstance(obj, (np.integer, int)): return int(obj)
    elif isinstance(obj, (np.floating, float)): return float(obj)
    elif isinstance(obj, (np.ndarray, list)): return [to_native(x) for x in obj]
    else: return obj

def get_pick_quaternion(grasp_spin_90=False):
    """ 
    仅用于 Pick 段的参考姿态：垂直向下 + 0/90度偏置 
    (注：最终抓取姿态由视觉节点实时计算)
    """
    base = R.from_euler('x', 180, degrees=True)
    spin = R.from_euler('z', 90 if grasp_spin_90 else 0, degrees=True)
    return to_native((base * spin).as_quat().tolist())

# ================= 3. 避障逻辑 (内部使用) =================

def check_accessibility(test_pos, scene_blocks, self_name):
    """ 内部计算：检查夹爪是否会撞到其他积木 """
    for blk in scene_blocks:
        if blk['name'] == self_name: continue
        ox, oy, oz = blk['abs_pos']
        dist_xy = np.sqrt((test_pos[0] - ox)**2 + (test_pos[1] - oy)**2)
        if abs(test_pos[2] - oz) < 0.03 and dist_xy < SAFE_RADIUS:
            return False
    return True

# ================= 4. 主处理函数 =================

def process_blueprint(input_yaml, output_yaml):
    if not os.path.exists(input_yaml):
        print(f"❌ 错误: 找不到蓝图 {input_yaml}")
        return

    with open(input_yaml, 'r') as f:
        blueprint = yaml.safe_load(f)

    # 1. 内部构建完整场景用于碰撞分析
    all_blocks = []
    for b in blueprint.get('blocks', []):
        block_copy = copy.deepcopy(b)
        # 计算内部绝对坐标
        block_copy['abs_pos'] = INTERNAL_ASSEMBLY_CENTER + np.array(b['pos'])
        block_copy['abs_pos'][2] = TABLE_HEIGHT + b['pos'][2]
        all_blocks.append(block_copy)

    # 2. 逆向拆卸分析
    disassembly_tasks = []
    remaining_blocks = copy.deepcopy(all_blocks)

    print("--- 正在进行逆向规划与相位决策 ---")
    
    while len(remaining_blocks) > 0:
        remaining_blocks.sort(key=lambda x: x['abs_pos'][2], reverse=True)
        target = remaining_blocks[0]
        name = target['name']
        
        # 决策：0度还是90度抓取相位
        need_spin_90 = False
        if not check_accessibility(target['abs_pos'], remaining_blocks, name):
            need_spin_90 = True
            print(f"  [分析] {name}: 切换为 90度抓取策略")
        else:
            print(f"  [分析] {name}: 保持 0度抓取策略")

        # 记录蓝图原始旋转角度 (Yaw)
        rot_rpy = target.get('rotation', [0, 0, 0])
        blueprint_yaw = rot_rpy[2] if isinstance(rot_rpy, list) else rot_rpy
        
        # 获取原材料区 Pick 坐标
        src_xy = SOURCE_LOCATIONS.get(name, [0.5, 0.0])
        dims = target.get('dims', [0.032, 0.032, 0.03])
        
        # 构建任务 (重点：place 部分完全保留蓝图原始数据)
        disassembly_tasks.append({
            "name": name,
            "grasp_spin": 90 if need_spin_90 else 0,
            "blueprint_yaw": to_native(blueprint_yaw),
            "pick": {
                "pos": to_native([src_xy[0], src_xy[1], TABLE_HEIGHT + dims[2]/2]),
                "orientation": get_pick_quaternion(need_spin_90)
            },
            "place": {
                # 直接使用 blueprint 里的原始 pos 和 rotation，不做任何加法或四元数转换
                "pos": to_native(target['pos']), 
                "orientation": to_native(target.get('rotation', [0, 0, 0])) 
            }
        })

        remaining_blocks.pop(0)

    # 3. 反转为装配顺序
    assembly_tasks = disassembly_tasks[::-1]
    for i, task in enumerate(assembly_tasks):
        task['id'] = i

    with open(output_yaml, 'w') as f:
        yaml.dump({"tasksh": assembly_tasks}, f, default_flow_style=None, sort_keys=False)
    
    print(f"\n✅ 规划完成！tasks.yaml 的 place 段已与蓝图保持一致。")

if __name__ == "__main__":
    process_blueprint("final_product_simple.yaml", "tasksh.yaml")