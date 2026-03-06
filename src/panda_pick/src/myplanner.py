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
THRESHOLD = 0.045       # 避障探测距离 (4.5cm)
TABLE_HEIGHT = 0.42     # 桌面高度
ASSEMBLY_CENTER = np.array([0.35, 0.0, TABLE_HEIGHT]) 

# 原材料区位置
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

# ================= 2. 核心判定逻辑 (带详细推理打印) =================

def check_accessibility(test_pos, scene_blocks, self_name, check_mode=90):
    tx, ty, tz = test_pos
    mode_str = "90度(探测X轴轨道)" if check_mode == 90 else "0度(探测Y轴轨道)"
    
    for blk in scene_blocks:
        if blk['name'] == self_name: continue
        ox, oy, oz = blk['abs_pos']
        
        # 只检查同一层 (dz < 1cm)
        if abs(tz - oz) < 0.01:
            dx = abs(tx - ox)
            dy = abs(ty - oy)

            if check_mode == 0:
                # 0度抓：检查 Y 轴轨道 (要求 X 坐标相同)
                if dx < 0.016: # 在同一条纵向线上
                    if dy < THRESHOLD:
                        print(f"      [❌ 碰撞] {mode_str} 轨道受阻！邻居: {blk['name']} | 距离 dy={dy:.3f} < {THRESHOLD}")
                        return False
            elif check_mode == 90:
                # 90度抓：检查 X 轴轨道 (要求 Y 坐标相同)
                if dy < 0.016: # 在同一条横向线上
                    if dx < THRESHOLD:
                        print(f"      [❌ 碰撞] {mode_str} 轨道受阻！邻居: {blk['name']} | 距离 dx={dx:.3f} < {THRESHOLD}")
                        return False
    
    print(f"      [✅ 安全] {mode_str} 路径清空")
    return True

def is_covered(target, remaining_blocks):
    tx, ty, tz = target['abs_pos']
    for blk in remaining_blocks:
        if blk == target: continue
        ox, oy, oz = blk['abs_pos']
        if oz > (tz + 0.01) and abs(tx - ox) < 0.016 and abs(ty - oy) < 0.016:
            return True
    return False

def check_balance(target, remaining_blocks):
    if target['pos'][2] <= 0.005: return True
    tx, ty, tz = target['abs_pos']
    for blk in remaining_blocks:
        if blk == target: continue
        ox, oy, oz = blk['abs_pos']
        if abs(tz - (oz + 0.03)) < 0.005:
            if abs(tx - ox) < 0.0165 and abs(ty - oy) < 0.0165:
                return True 
    return False

# ================= 3. 主规划程序 =================

def process_blueprint(input_yaml, output_yaml):
    with open(input_yaml, 'r') as f:
        blueprint = yaml.safe_load(f)

    remaining_blocks = []
    for b in blueprint.get('blocks', []):
        bd = copy.deepcopy(b)
        bd['abs_pos'] = ASSEMBLY_CENTER + np.array(b['pos'])
        bd['abs_pos'][2] = TABLE_HEIGHT + b['pos'][2]
        remaining_blocks.append(bd)

    disassembly_tasks = []
    print(f"\n" + "="*50)
    print(f"🚀 开始逆向推理规划 | 避障阈值: {THRESHOLD}m")
    print(f"逻辑设定: 0度看Y轴轨道, 90度看X轴轨道")
    print("="*50)

    while len(remaining_blocks) > 0:
        # 始终先尝试拆掉最高层的积木
        remaining_blocks.sort(key=lambda x: x['abs_pos'][2], reverse=True)
        
        target = None
        final_spin = 90
        
        print(f"\n正在寻找可拆卸的目标... (当前剩余: {len(remaining_blocks)} 块)")
        
        for cand in remaining_blocks:
            print(f"  ▶ 检查积木: {cand['name']} (相对高度: {cand['pos'][2]})")
            
            # 1. 检查物理可行性
            if is_covered(cand, remaining_blocks):
                print(f"    - 跳过: 上方有积木遮挡")
                continue
            if not check_balance(cand, remaining_blocks):
                print(f"    - 跳过: 拆掉后下方无支撑点或重心不稳")
                continue
            
            # 2. 尝试 90 度抓取 (优先策略)
            if check_accessibility(cand['abs_pos'], remaining_blocks, cand['name'], check_mode=90):
                target, final_spin = cand, 90
                print(f"    - 判定结果: 采用 90 度抓取")
                break
            
            # 3. 尝试 0 度抓取 (备选策略)
            print(f"    - 90度检测失败，正在尝试切换到 0度检测...")
            if check_accessibility(cand['abs_pos'], remaining_blocks, cand['name'], check_mode=0):
                target, final_spin = cand, 0
                print(f"    - 判定结果: 采用 0 度抓取")
                break
            
            print(f"    - ❌ 错误: 该积木在所有相位下均会发生碰撞")

        if target is None:
            target, final_spin = remaining_blocks[0], 90
            print(f"  [⚠️ 警告] 无法找到完美避障路径，强制拆卸最高块: {target['name']}")

        # 记录拆卸任务
        rot_rpy = target.get('rotation', [0, 0, 0])
        blueprint_yaw = rot_rpy[2] if isinstance(rot_rpy, list) else rot_rpy
        src_xy = SOURCE_LOCATIONS.get(target['name'], [0.5, 0.0])
        
        disassembly_tasks.append({
            "name": target['name'],
            "grasp_spin": final_spin,
            "blueprint_yaw": to_native(blueprint_yaw),
            "pick": {
                "pos": to_native([src_xy[0], src_xy[1], TABLE_HEIGHT + 0.015]),
                "orientation": get_strategy_quaternion(final_spin)
            },
            "place": {
                "pos": to_native(target['pos']), 
                "orientation": get_strategy_quaternion(final_spin)
            }
        })
        
        # 模拟物理移除
        print(f"  ✨ 确认拆卸: {target['name']} | 轨道已清空")
        remaining_blocks.remove(target)

    # 将逆向拆卸顺序转为正向装配顺序
    print(f"\n" + "="*50)
    print(f"✅ 所有积木处理完毕，正在生成正向装配清单...")
    assembly_tasks = disassembly_tasks[::-1]
    for i, task in enumerate(assembly_tasks):
        task['id'] = i

    with open(output_yaml, 'w') as f:
        yaml.dump({"tasksh": assembly_tasks}, f, sort_keys=False)
    print(f"🎉 最终文件已生成: {output_yaml}")
    print("="*50 + "\n")

if __name__ == "__main__":
    process_blueprint("final_product.yaml", "tasksh.yaml")