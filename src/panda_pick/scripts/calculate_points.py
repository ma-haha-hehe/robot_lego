import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

# ==========================================
# 1. 基础配置
# ==========================================
VOXEL_PER_BRICK = 6 

# ==========================================
# 2. 定义机器人结构
# ==========================================
def get_robot_layout():
    layout = []
    def add(x, y, z, c):
        layout.append({"grid": (x, y, z), "color_id": c})

    # A. 双腿 (Z=0,1,2)
    for z in range(3):
        add(0, 1, z, 0); add(0, -1, z, 0)
    # B. 胯部 (Z=3)
    add(0, 1, 3, 1); add(0, 0, 3, 2); add(0, -1, 3, 1)
    # C. 躯干 (Z=4)
    add(0, 2, 4, 3); add(0, 1, 4, 3); add(0, 0, 4, 4); add(0, -1, 4, 3); add(0, -2, 4, 3)
    # D. 手部 (Z=5)
    add(0, 2, 5, 5); add(0, -2, 5, 5)
    # E. 头部 (Z=5,6,7)
    add(0, 0, 5, 4); add(0, 0, 6, 6); add(0, 0, 7, 6) # <--- 最顶层是 Z=7

    return layout

# ==========================================
# 3. 体素世界构建
# ==========================================
class VoxelWorld:
    def __init__(self, layout):
        self.layout = layout
        self.brick_size = VOXEL_PER_BRICK
        
        coords = np.array([item["grid"] for item in layout])
        min_grid = coords.min(axis=0)
        max_grid = coords.max(axis=0)
        
        # 这里的 padding 影响可视化，但不影响相对坐标计算
        self.xy_padding = 3 
        self.grid_offset = np.array([-min_grid[0]+self.xy_padding, -min_grid[1]+self.xy_padding, -min_grid[2]])
        
        self._build_voxel_grid(max_grid, min_grid)

    def _build_voxel_grid(self, max_grid, min_grid):
        span = max_grid - min_grid + 1
        span[0] += self.xy_padding * 2
        span[1] += self.xy_padding * 2
        
        self.shape = span * self.brick_size
        self.voxels = np.zeros(self.shape, dtype=int)

        for i, item in enumerate(self.layout):
            gx, gy, gz = item["grid"]
            # 计算体素坐标
            vx = (gx + self.grid_offset[0]) * self.brick_size
            vy = (gy + self.grid_offset[1]) * self.brick_size
            vz = (gz + self.grid_offset[2]) * self.brick_size
            
            self.voxels[vx:vx+self.brick_size, vy:vy+self.brick_size, vz:vz+self.brick_size] = 1
            
            item["id"] = i
            item["voxel_origin"] = np.array([vx, vy, vz])
            item["dims"] = np.array([self.brick_size, self.brick_size, self.brick_size])

    def is_empty(self, x, y, z):
        if x < 0 or y < 0 or z < 0: return True
        if x >= self.shape[0] or y >= self.shape[1] or z >= self.shape[2]: return True
        return self.voxels[x, y, z] == 0

# ==========================================
# 4. 智能抓取规划算法
# ==========================================
def plan_grasps(world):
    candidates = []
    for item in world.layout:
        vx, vy, vz = item["voxel_origin"]
        dx, dy, dz = item["dims"]
        
        cx = vx + dx // 2
        cy = vy + dy // 2
        cz = vz + dz // 2
        
        possible_pairs = []
        # X轴抓取
        if world.is_empty(vx - 1, cy, cz) and world.is_empty(vx + dx, cy, cz):
            p1 = np.array([vx, cy, cz])
            p2 = np.array([vx + dx, cy, cz])
            possible_pairs.append({'axis': 'x', 'p1': p1, 'p2': p2, 'center': np.array([cx, cy, cz])})
        # Y轴抓取
        if world.is_empty(cx, vy - 1, cz) and world.is_empty(cx, vy + dy, cz):
            p1 = np.array([cx, vy, cz])
            p2 = np.array([cx, vy + dy, cz])
            possible_pairs.append({'axis': 'y', 'p1': p1, 'p2': p2, 'center': np.array([cx, cy, cz])})

        if not possible_pairs: continue
        
        # 简单策略
        selected_pair = possible_pairs[0]
        candidates.append(selected_pair)

    # 排序: 这里的 key 非常关键，负号表示 Z 越大排越前 (从上往下拆)
    sorted_grasps = sorted(candidates, key=lambda g: (-g['center'][2], g['center'][1]))
    return sorted_grasps

# ==========================================
# 5. [新功能] 坐标转换器 (Sim-to-Real)
# ==========================================
def convert_to_robot_coords(grasp_pair, voxel_world_obj):
    """
    将体素坐标转换为真实世界的机器人坐标 (meters)
    """
    center_voxel = grasp_pair['center']
    
    # --- 校准参数 (Calibration) ---
    # 我们知道真实世界中最顶部的块 (Head) 高度是 z=0.635
    # 我们知道在 Voxel 世界里，最顶部的中心高度是:
    #   Z_grid = 7 (layout define)
    #   Z_voxel_origin = 7 * 6 = 42
    #   Z_center = 42 + 3 = 45
    # 所以比例 Scale = 0.635 / 45 ≈ 0.01411
    
    SCALE = 0.635 / 45.0  # 每一个体素单位对应多少米
    
    # 真实世界的积木中心位置 (根据你的 XML 配置)
    # 机器人在 x=0, 积木在 x=0.5
    REAL_OFFSET_X = 0.5   
    REAL_OFFSET_Y = 0.0
    
    # --- 计算转换 ---
    # 1. Z轴: 直接缩放 (Z是从地面算起的)
    real_z = center_voxel[2] * SCALE
    
    # 2. X/Y轴: 需要处理 Padding 偏移
    # 在 Voxel 代码里，我们加了 xy_padding = 3
    # 真正的中心在 grid_offset 修正后的位置
    
    # 获取体素世界的中心点 (假设积木整体是以 layout 原点为中心的)
    # layout 原点 (0,0) 对应 Voxel 里的:
    origin_vx = (0 + voxel_world_obj.grid_offset[0]) * voxel_world_obj.brick_size
    origin_vy = (0 + voxel_world_obj.grid_offset[1]) * voxel_world_obj.brick_size
    
    # 计算相对于中心的偏移量
    delta_x_voxel = center_voxel[0] - origin_vx
    delta_y_voxel = center_voxel[1] - origin_vy
    
    # 映射到真实世界
    # Voxel X 是深度(前后) -> 对应机器人 X ? 还是 Y?
    # 在 layout 中: 腿是 (0, 1) 和 (0, -1). 也就是 Y 轴分布.
    # 所以 Voxel Y 对应 Real Y. Voxel X 对应 Real X.
    
    real_y = REAL_OFFSET_Y + (delta_y_voxel * SCALE)
    
    # 这里的 X 比较特殊，Voxel里都是 0 (扁平的)，所以直接用 Real Offset
    # 如果积木有厚度，可以用 delta_x_voxel * SCALE 微调
    real_x = REAL_OFFSET_X + (delta_x_voxel * SCALE)

    return real_x, real_y, real_z

# ==========================================
# 6. 运行主程序
# ==========================================
if __name__ == "__main__":
    layout = get_robot_layout()
    world = VoxelWorld(layout)
    grasp_sequence = plan_grasps(world)
    
    # 1. 打印所有抓取点
    print(f"计算出 {len(grasp_sequence)} 个抓取点。")
    print("正在按顺序(从上到下)转换为机器人坐标...\n")
    
    print("-" * 60)
    print(f"{'Order':<6} | {'Voxel Center (Z,Y,X)':<20} | {'Robot Target (x,y,z)':<25}")
    print("-" * 60)

    for i, grasp in enumerate(grasp_sequence):
        # 转换坐标
        rx, ry, rz = convert_to_robot_coords(grasp, world)
        
        # 修正：为了方便抓取，Z轴稍微降低一点点(0.03m)以包住物体
        # 就像我们之前在 pick_block.py 里做的那样
        pick_z = rz - 0.03 
        
        vx, vy, vz = grasp['center']
        
        print(f"No.{i+1:<3} | ({vz}, {vy}, {vx})              | x={rx:.3f}, y={ry:.3f}, z={rz:.3f}")
        
        # 我们只关心第一个（最顶上的）
        if i == 0:
            top_block_target = (rx, ry, pick_z)

    print("-" * 60)
    print("\n>>> 下一步操作 <<<")
    print(f"请将以下坐标填入 pick_block.py 的 node.pick_at_point(...) 中:")
    print(f"Target X = {top_block_target[0]:.3f}")
    print(f"Target Y = {top_block_target[1]:.3f}")
    print(f"Target Z = {top_block_target[2]:.3f} (已自动减去3cm以包住物体)")
    
    # 可视化(可选)
    # visualize(world, grasp_sequence)