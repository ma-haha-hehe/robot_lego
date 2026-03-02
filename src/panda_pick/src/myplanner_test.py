#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import yaml
import os
import copy
from scipy.spatial.transform import Rotation as R

yaml.Dumper.ignore_aliases = lambda *args: True

# ================= 1. 配置区域 =================
TABLE_HEIGHT = 0.42       
INTERNAL_ASSEMBLY_CENTER = np.array([0.35, 0.0, TABLE_HEIGHT]) 

# 原材料区固定位置 (仅作为参考)
SOURCE_LOCATIONS = {
    "white 4x2 brick":  [0.55,  0.25],
    "blue 2x2 brick":   [0.55,  0.10],
    "red 2x2 brick":    [0.55, -0.10],
    "yellow 4x2 brick": [0.55, -0.25]
}

# ================= 2. 工具函数 =================
def to_native(obj):
    if isinstance(obj, (np.integer, int)): return int(obj)
    elif isinstance(obj, (np.floating, float)): return float(obj)
    elif isinstance(obj, (np.ndarray, list)): return [to_native(x) for x in obj]
    else: return obj

# ================= 3. 核心：带旋转补偿的指尖检测 =================

def check_accessibility(target_pos, dims, blueprint_yaw, scene_blocks, self_name, try_spin_90=False):
    """
    根据积木自身的旋转角度动态计算指尖位置
    blueprint_yaw: 蓝图里定义的初始角度 (角度制)
    """
    FINGER_CLEARANCE = 0.006  # 手指距离边缘的净空 (6mm)
    FINGER_SAFE_RADIUS = 0.012 # 手指本身的物理占用半径

    # 计算最终探测的总角度
    # 如果 0 度抓：沿积木长边方向探测
    # 如果 90 度抓：沿积木短边方向探测
    total_yaw_deg = blueprint_yaw + (90 if try_spin_90 else 0)
    theta = np.radians(total_yaw_deg)

    # 计算探测距离：基于积木的长度或宽度
    # 0度抓取对应长边边沿，90度抓取对应短边边沿
    dist = (dims[0] / 2 + FINGER_CLEARANCE) if not try_spin_90 else (dims[1] / 2 + FINGER_CLEARANCE)

    # 旋转矩阵计算探测点在世界系下的偏移量
    # x = r * cos(theta), y = r * sin(theta)
    dx = dist * np.cos(theta)
    dy = dist * np.sin(theta)

    fingers = [
        np.array([target_pos[0] + dx, target_pos[1] + dy, target_pos[2]]),
        np.array([target_pos[0] - dx, target_pos[1] - dy, target_pos[2]])
    ]

    # 碰撞检测逻辑
    for blk in scene_blocks:
        if blk['name'] == self_name: continue
        ox, oy, oz = blk['abs_pos']
        
        # 仅检查同层积木的碰撞风险
        if abs(target_pos[2] - oz) < 0.025:
            for f_pos in fingers:
                dist_to_neighbor = np.sqrt((f_pos[0] - ox)**2 + (f_pos[1] - oy)**2)
                if dist_to_neighbor < FINGER_SAFE_RADIUS:
                    return False 
                    
    return True

# ================= 4. 主处理函数 =================

def process_blueprint(input_yaml, output_yaml):
    if not os.path.exists(input_yaml):
        print(f"❌ 错误: 找不到蓝图 {input_yaml}")
        return

    with open(input_yaml, 'r') as f:
        blueprint = yaml.safe_load(f)

    # 1. 场景重建
    all_blocks = []
    for b in blueprint.get('blocks', []):
        block_copy = copy.deepcopy(b)
        block_copy['abs_pos'] = INTERNAL_ASSEMBLY_CENTER + np.array(b['pos'])
        block_copy['abs_pos'][2] = TABLE_HEIGHT + b['pos'][2]
        all_blocks.append(block_copy)

    # 2. 逆向模拟规划
    disassembly_tasks = []
    remaining_blocks = copy.deepcopy(all_blocks)

    print("--- 正在执行旋转补偿避障分析 ---")
    
    while len(remaining_blocks) > 0:
        remaining_blocks.sort(key=lambda x: x['abs_pos'][2], reverse=True)
        target = remaining_blocks[0]
        name = target['name']
        dims = target.get('dims', [0.032, 0.016, 0.012])
        
        # 提取蓝图中的原始偏航角 (Blueprint Yaw)
        rot_rpy = target.get('rotation', [0, 0, 0])
        blueprint_yaw = rot_rpy[2] if isinstance(rot_rpy, list) else rot_rpy
        
        # 核心决策：带角度补偿的探测
        need_spin_90 = False
        if check_accessibility(target['abs_pos'], dims, blueprint_yaw, remaining_blocks, name, try_spin_90=False):
            need_spin_90 = False
            print(f"  [分析] {name}: 平行抓取(0°) 路径安全")
        elif check_accessibility(target['abs_pos'], dims, blueprint_yaw, remaining_blocks, name, try_spin_90=True):
            need_spin_90 = True
            print(f"  [分析] {name}: 平行受阻，垂直抓取(90°) 路径安全")
        else:
            print(f"  [警告] {name}: 空间受限，建议手动检查！")

        # 生成任务项
        src_xy = SOURCE_LOCATIONS.get(name, [0.5, 0.0])
        disassembly_tasks.append({
            "name": name,
            "grasp_spin": 90 if need_spin_90 else 0,
            "blueprint_yaw": to_native(blueprint_yaw),
            "place": {
                "pos": to_native(target['pos']), 
                "orientation": to_native(rot_rpy) 
            },
            "id": 0 # 后续统一分配
        })

        remaining_blocks.pop(0)

    # 3. 反转并保存
    assembly_tasks = disassembly_tasks[::-1]
    for i, task in enumerate(assembly_tasks):
        task['id'] = i

    with open(output_yaml, 'w') as f:
        yaml.dump({"tasksh": assembly_tasks}, f, sort_keys=False)
    
    print(f"\n✅ 规划完成！已处理积木旋转补偿。")

if __name__ == "__main__":
    process_blueprint("final_product_simple.yaml", "tasksh_test.yaml")