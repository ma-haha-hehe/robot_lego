#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import yaml
import os
import copy
from scipy.spatial.transform import Rotation as R

# 禁用 YAML 锚点
yaml.Dumper.ignore_aliases = lambda *args: True

# ================= 1. 配置区域 =================
THRESHOLD = 0.045       # 避障探测距离
TABLE_HEIGHT = 0.42     # 桌面高度
ASSEMBLY_CENTER = np.array([0.35, 0.0, TABLE_HEIGHT]) 

# 原材料区位置 (Pick 区域)
SOURCE_LOCATIONS = {
    "base_4x2_lvl1":    [0.55,  0.25],
    "support_2x2_left": [0.55,  0.10],
    "support_2x2_right":[0.55, -0.10],
    "top_4x2_lvl4":     [0.62,  0.00]
}

def to_native(obj):
    if isinstance(obj, (np.integer, int)): return int(obj)
    elif isinstance(obj, (np.floating, float)): return float(obj)
    elif isinstance(obj, (np.ndarray, list)): return [to_native(x) for x in obj]
    else: return obj

def get_strategy_quaternion(spin_deg):
    base_rot = R.from_euler('x', 180, degrees=True)
    strategy_rot = R.from_euler('z', spin_deg, degrees=True)
    return to_native((strategy_rot * base_rot).as_quat().tolist())

# ================= 2. 物理与稳定性判定 =================

def get_base_footprint(all_blocks):
    """找到最低层(Z=0)积木四个角连成的最大外接矩形范围"""
    base_blocks = [b for b in all_blocks if b['pos'][2] <= 0.005]
    if not base_blocks: return 0, 0, 0, 0
    xs, ys = [], []
    for b in base_blocks:
        hx = 0.016 # 2x2 半宽
        hy = 0.032 if "4x2" in b.get('type', '') else 0.016
        px, py = b['abs_pos'][0], b['abs_pos'][1]
        xs.extend([px - hx, px + hx])
        ys.extend([py - hy, py + hy])
    return min(xs), max(xs), min(ys), max(ys)

def is_within_base_support(target, footprint):
    """判定积木中心点投影是否落在底座最大面积内"""
    xmin, xmax, ymin, ymax = footprint
    tx, ty = target['abs_pos'][0], target['abs_pos'][1]
    return xmin <= tx <= xmax and ymin <= ty <= ymax

def find_direct_support(target, all_blocks):
    """寻找积木正下方的那个直接物理支撑物"""
    tx, ty, tz = target['abs_pos']
    for blk in all_blocks:
        if blk == target: continue
        ox, oy, oz = blk['abs_pos']
        # 检查下方一层 (1.5cm 间隔)
        if abs(tz - (oz + 0.015)) < 0.005:
            # 只要水平方向有重合即可
            if abs(tx - ox) < 0.033 and abs(ty - oy) < 0.033:
                return blk
    return None

def is_covered(target, remaining_blocks):
    """检查上方是否有积木遮挡"""
    tx, ty, tz = target['abs_pos']
    for blk in remaining_blocks:
        if blk == target: continue
        ox, oy, oz = blk['abs_pos']
        if oz > (tz + 0.01) and abs(tx - ox) < 0.016 and abs(ty - oy) < 0.016:
            return True
    return False

def check_accessibility(test_pos, scene_blocks, self_name, check_mode=90):
    """路径避障探测"""
    tx, ty, tz = test_pos
    for blk in scene_blocks:
        if blk['name'] == self_name: continue
        ox, oy, oz = blk['abs_pos']
        if abs(tz - oz) < 0.01:
            dx, dy = abs(tx - ox), abs(ty - oy)
            if check_mode == 0 and dx < 0.005 and dy < THRESHOLD: return False
            if check_mode == 90 and dy < 0.005 and dx < THRESHOLD: return False
    return True

# ================= 3. 主规划程序 =================

def process_blueprint(input_yaml, output_yaml):
    with open(input_yaml, 'r') as f:
        blueprint = yaml.safe_load(f)

    all_blocks = []
    for b in blueprint.get('blocks', []):
        bd = copy.deepcopy(b)
        bd['abs_pos'] = ASSEMBLY_CENTER + np.array(b['pos'])
        bd['abs_pos'][2] = TABLE_HEIGHT + b['pos'][2]
        all_blocks.append(bd)

    # 获取底座终极范围
    base_area = get_base_footprint(all_blocks)
    
    remaining_blocks = copy.deepcopy(all_blocks)
    disassembly_tasks = []
    shadow_map = {} # 影子存储 {支撑物ID: [悬挑积木列表]}

    print(f"\n" + "="*50)
    print(f" 🚀 影子规划器启动 | 底座范围判定模式")
    print("="*50)

    while len(remaining_blocks) > 0:
        # 按高度从大到小排列 (逆向拆卸)
        remaining_blocks.sort(key=lambda x: x['abs_pos'][2], reverse=True)
        
        target = None
        final_spin = 90
        
        for cand in remaining_blocks:
            # 1. 遮挡检查
            if is_covered(cand, remaining_blocks): continue
            
            # 2. 重心判定 (底座投影逻辑)
            if not is_within_base_support(cand, base_area):
                supp = find_direct_support(cand, remaining_blocks)
                if supp:
                    s_id = id(supp)
                    if s_id not in shadow_map: shadow_map[s_id] = []
                    # 挂载到影子队列，暂不处理
                    if cand not in [item for sublist in shadow_map.values() for item in sublist]:
                        shadow_map[s_id].append(cand)
                    continue 

            # 3. 避障检测
            if check_accessibility(cand['abs_pos'], remaining_blocks, cand['name'], 90):
                target, final_spin = cand, 90; break
            elif check_accessibility(cand['abs_pos'], remaining_blocks, cand['name'], 0):
                target, final_spin = cand, 0; break

        # 如果主循环没找到能拆的，说明存在死锁或仅剩底座投影外的积木
        if target is None: target = remaining_blocks[0]

        # --- 影子任务触发逻辑 ---
        # 如果当前要拆的是别人的地基，必须先拆掉所有依赖它的影子
        current_id = id(target)
        if current_id in shadow_map:
            for child in shadow_map[current_id]:
                print(f"  ⛓️ 影子激活: 由于支撑物即将移动，先拆除其悬挑子块 {child['name']}")
                record_task_data(child, 90, disassembly_tasks)
                remaining_blocks.remove(child)
            del shadow_map[current_id]

        # 记录地基/正常积木任务
        record_task_data(target, final_spin, disassembly_tasks)
        print(f"  确认拆卸: {target['name']} | Z={target['pos'][2]}")
        remaining_blocks.remove(target)

    # 生成最终文件 (反转为正向组装)
    save_final_task(disassembly_tasks[::-1], output_yaml)

def record_task_data(blk, spin, task_list):
    rot = blk.get('rotation', [0, 0, 0])
    yaw = rot[2] if isinstance(rot, list) else rot
    src = SOURCE_LOCATIONS.get(blk['name'], [0.5, 0.0])
    task_list.append({
        "name": blk['name'],
        "type": blk.get('type', 'brick_4x2'),
        "grasp_spin": spin,
        "blueprint_yaw": to_native(yaw),
        "pick": {
            "pos": to_native([src[0], src[1], TABLE_HEIGHT + 0.02]),
            "orientation": get_strategy_quaternion(spin)
        },
        "place": {
            "pos": to_native(blk['pos']), 
            "orientation": get_strategy_quaternion(spin)
        }
    })

def save_final_task(tasks, path):
    # 自动编号和名称唯一化
    name_counts = {}
    for i, t in enumerate(tasks):
        base = t['name']
        name_counts[base] = name_counts.get(base, 0) + 1
        t['name'] = f"{base}_{name_counts[base]}"
        t['id'] = i

    with open(path, 'w') as f:
        yaml.dump({"tasks": tasks}, f, sort_keys=False)
    print(f"\n✅ 任务清单生成成功: {path}")

if __name__ == "__main__":
    process_blueprint("final_product_flower.yaml", "task_test_flower.yaml")