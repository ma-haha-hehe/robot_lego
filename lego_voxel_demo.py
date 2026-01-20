import numpy as np
import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

# ==========================================
# 1. 输入数据 (YAML String)
# ==========================================
YAML_DATA = """
common_properties:
  color: [0.1, 0.1, 0.1, 1.0]
  unit_size: 0.03

bricks:
  # --- 1. 双腿 (Legs) ---
  - name: "leg_left_1"
    type: "brick_2x2"
    position: [0.0, 0.04, 0.015]
    orientation: [0, 0, 0]
  - name: "leg_left_2"
    type: "brick_2x2"
    position: [0.0, 0.04, 0.045]
  - name: "leg_left_3"
    type: "brick_2x2"
    position: [0.0, 0.04, 0.075]

  - name: "leg_right_1"
    type: "brick_2x2"
    position: [0.0, -0.04, 0.015]
  - name: "leg_right_2"
    type: "brick_2x2"
    position: [0.0, -0.04, 0.045]
  - name: "leg_right_3"
    type: "brick_2x2"
    position: [0.0, -0.04, 0.075]

  # --- 2. 躯干 (Torso) ---
  - name: "torso_bridge_left"
    type: "brick_2x2"
    position: [0.0, 0.04, 0.105]
  - name: "torso_bridge_center"
    type: "brick_2x2"
    position: [0.0, 0.0, 0.105]
  - name: "torso_bridge_right"
    type: "brick_2x2"
    position: [0.0, -0.04, 0.105]

  # --- 3. 肩膀/手臂 (Shoulders) ---
  - name: "shoulder_left_far"
    type: "brick_2x2"
    position: [0.0, 0.08, 0.135]
  - name: "shoulder_left_inner"
    type: "brick_2x2"
    position: [0.0, 0.04, 0.135]
  - name: "shoulder_center"
    type: "brick_2x2"
    position: [0.0, 0.0, 0.135]
  - name: "shoulder_right_inner"
    type: "brick_2x2"
    position: [0.0, -0.04, 0.135]
  - name: "shoulder_right_far"
    type: "brick_2x2"
    position: [0.0, -0.08, 0.135]

  # --- 4. 手部突起 (Hands) ---
  - name: "hand_tip_left"
    type: "brick_1x2"
    position: [0.0, 0.08, 0.165]
  - name: "hand_tip_right"
    type: "brick_1x2"
    position: [0.0, -0.08, 0.165]

  # --- 5. 头部 (Head) ---
  - name: "neck"
    type: "brick_2x2"
    position: [0.0, 0.0, 0.165]
  
  - name: "head_base"
    type: "brick_2x4"
    position: [0.0, 0.0, 0.195]
    orientation: [0, 0, 1.57] # 90度旋转

  - name: "head_top"
    type: "brick_2x2"
    position: [0.0, 0.0, 0.225]
"""

# ==========================================
# 2. 配置与定义
# ==========================================

# 定义体素缩放比例：1个单位代表多少米
# 这里设定 0.01m (1cm) = 1个体素单位
VOXEL_SCALE = 100 

# 定义不同类型积木的体素尺寸 (长, 宽, 高)
# 假设 2x2 积木大约是 3cm x 3cm x 3cm (为了配合 YAML 中的 0.03 pos 步长)
BRICK_DIMS = {
    "brick_2x2": np.array([3, 3, 3]),
    "brick_1x2": np.array([2, 3, 3]), # 稍微窄一点
    "brick_2x4": np.array([3, 6, 3]), # 长一点
}

def load_bricks_from_yaml(yaml_str):
    data = yaml.safe_load(yaml_str)
    return data["bricks"]

# ==========================================
# 3. 核心逻辑：坐标转换与体素化
# ==========================================

class VoxelWorld:
    def __init__(self, bricks):
        self.bricks = bricks
        self.processed_bricks = [] # 存储转换后的整数坐标积木
        self.voxel_grid = None
        self.offsets = np.array([10, 15, 0]) # x, y, z 偏移量，防止负坐标越界
        
        self._process_bricks()
        self._build_grid()

    def _process_bricks(self):
        """将世界坐标(float)转换为网格坐标(int)，并处理旋转"""
        for i, b in enumerate(self.bricks):
            b_type = b["type"]
            pos = np.array(b["position"])
            orientation = b.get("orientation", [0, 0, 0])
            
            # 1. 确定尺寸
            dims = BRICK_DIMS.get(b_type, np.array([3, 3, 3])).copy()
            
            # 2. 处理旋转 (简单的 90度 check)
            # 如果 yaw (z轴旋转) 接近 1.57 (pi/2) 或 -1.57，交换 x 和 y 尺寸
            yaw = orientation[2] if len(orientation) > 2 else 0
            if abs(abs(yaw) - 1.57) < 0.1: 
                dims[0], dims[1] = dims[1], dims[0]

            # 3. 坐标转换
            # 世界坐标通常是中心点，体素坐标通常是左下角
            # grid_pos = (world_pos * scale) - (size / 2) + offset
            grid_center = pos * VOXEL_SCALE
            grid_corner = grid_center - (dims / 2)
            grid_corner_int = np.round(grid_corner + self.offsets).astype(int)
            
            self.processed_bricks.append({
                "id": i + 1, # ID从1开始，0是空
                "name": b["name"],
                "origin": grid_corner_int,
                "dims": dims
            })

    def _build_grid(self):
        """创建 3D numpy 数组"""
        # 计算边界
        max_x, max_y, max_z = 0, 0, 0
        for b in self.processed_bricks:
            end = b["origin"] + b["dims"]
            max_x = max(max_x, end[0])
            max_y = max(max_y, end[1])
            max_z = max(max_z, end[2])
        
        # 增加一些 Padding
        self.grid_shape = (max_x + 2, max_y + 2, max_z + 2)
        self.voxel_grid = np.zeros(self.grid_shape, dtype=int)

        # 填充
        for b in self.processed_bricks:
            x, y, z = b["origin"]
            dx, dy, dz = b["dims"]
            self.voxel_grid[x:x+dx, y:y+dy, z:z+dz] = b["id"]

    def is_empty(self, x, y, z):
        """检查某个体素是否为空"""
        if x < 0 or y < 0 or z < 0: return True
        if x >= self.grid_shape[0] or y >= self.grid_shape[1] or z >= self.grid_shape[2]: return True
        return self.voxel_grid[x, y, z] == 0

    def get_voxel_center(self, x, y, z):
        return np.array([x + 0.5, y + 0.5, z + 0.5])

# ==========================================
# 4. 抓取检测算法
# ==========================================

def find_grasp_points(world):
    """
    遍历每个积木，检测 X 轴和 Y 轴方向是否可以抓取。
    规则：积木某一行 (voxel row) 的左边是空的 AND 右边是空的 -> 这是一个抓取点对。
    """
    grasp_points = [] # list of coordinates

    for b in world.processed_bricks:
        x0, y0, z0 = b["origin"]
        dx, dy, dz = b["dims"]
        
        # --- 扫描 X 轴方向抓取 (夹子在左右两侧) ---
        # 我们遍历该积木 Y-Z 平面上的每一个点
        for y in range(y0, y0 + dy):
            for z in range(z0, z0 + dz):
                # 检查积木左侧面外一格 (x0 - 1) 和 右侧面外一格 (x0 + dx)
                left_voxel_x = x0 - 1
                right_voxel_x = x0 + dx
                
                if world.is_empty(left_voxel_x, y, z) and world.is_empty(right_voxel_x, y, z):
                    # 记录两个点：抓取点左 和 抓取点右
                    grasp_points.append(world.get_voxel_center(left_voxel_x, y, z))
                    grasp_points.append(world.get_voxel_center(right_voxel_x, y, z))

        # --- 扫描 Y 轴方向抓取 (夹子在前后两侧) ---
        # 我们遍历该积木 X-Z 平面上的每一个点
        for x in range(x0, x0 + dx):
            for z in range(z0, z0 + dz):
                # 检查积木前侧面外一格 (y0 - 1) 和 后侧面外一格 (y0 + dy)
                front_voxel_y = y0 - 1
                back_voxel_y = y0 + dy
                
                if world.is_empty(x, front_voxel_y, z) and world.is_empty(x, back_voxel_y, z):
                    grasp_points.append(world.get_voxel_center(x, front_voxel_y, z))
                    grasp_points.append(world.get_voxel_center(x, back_voxel_y, z))

    return np.array(grasp_points)

# ==========================================
# 5. 可视化
# ==========================================

def visualize(world, grasp_points):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 颜色映射
    cmap = cm.get_cmap("tab20")
    
    # 1. 绘制积木
    for b in world.processed_bricks:
        x, y, z = b["origin"]
        dx, dy, dz = b["dims"]
        
        color = cmap((b["id"] * 3) % 20) # 随机取色
        
        # bar3d 绘制长方体
        # 注意：bar3d 需要底面坐标和长宽高
        ax.bar3d(x, y, z, dx, dy, dz, color=color, alpha=0.8, edgecolor='k', linewidth=0.5)

    # 2. 绘制抓取点
    if len(grasp_points) > 0:
        # 为了美观，过滤掉重叠的点或稍微随机化一点点
        ax.scatter(grasp_points[:, 0], grasp_points[:, 1], grasp_points[:, 2], 
                   color="black", s=30, marker='o', depthshade=False, label="Valid Grasp Void")

    # 设置轴
    ax.set_xlabel("X (Voxel)")
    ax.set_ylabel("Y (Voxel)")
    ax.set_zlabel("Z (Voxel)")
    ax.set_title(f"Lego Voxel Analysis: {len(world.processed_bricks)} Bricks\nBlack dots = Valid Grasp Space")
    
    # 设置比例一致，防止拉伸
    try:
        ax.set_box_aspect((np.ptp(world.voxel_grid.shape[0]), 
                           np.ptp(world.voxel_grid.shape[1]), 
                           np.ptp(world.voxel_grid.shape[2])))
    except:
        pass

    plt.legend()
    plt.show()

# ==========================================
# 6. Main
# ==========================================
if __name__ == "__main__":
    # 加载数据
    raw_bricks = load_bricks_from_yaml(YAML_DATA)
    
    # 构建世界
    world = VoxelWorld(raw_bricks)
    print(f"Grid Shape: {world.grid_shape}")
    
    # 计算抓取点
    points = find_grasp_points(world)
    print(f"Found {len(points)} potential grasp contact points (voxel faces).")
    
    # 绘图
    visualize(world, points)