import numpy as np
import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
from matplotlib import cm

# ------------------------------
# 1. 砖块尺寸定义（单位：stud）
# ------------------------------
BRICK_SHAPES = {
    "1x1": (1, 1, 1),
    "1x2": (1, 2, 1),
    "2x2": (2, 2, 1),
    "2x4": (2, 4, 1),
}

# ------------------------------
# 2. 示例 YAML
# ------------------------------
YAML_EXAMPLE = """
bricks:
  # --- Layer 0 ---
  - id: 1
    type: "2x4"
    color: "blue"
    pos: [0, 0, 0]
    yaw: 0

  - id: 2
    type: "2x2"
    color: "green"
    pos: [3, 0, 0]
    yaw: 0

  - id: 3
    type: "2x2"
    color: "yellow"
    pos: [1, 3, 0]
    yaw: 0

  # --- Layer 1 (stacked on blocks 1 & 2) ---
  - id: 4
    type: "1x2"
    color: "orange"
    pos: [0, 1, 1]   # partly above block 1
    yaw: 0

  - id: 5
    type: "2x2"
    color: "red"
    pos: [3, 1, 1]   # directly above green block
    yaw: 0

  # --- Layer 2 (one more stacked brick) ---
  - id: 6
    type: "1x1"
    color: "pink"
    pos: [3, 1, 2]   # on top of red block
    yaw: 0

  # --- A block isolated to test free space ---
  - id: 7
    type: "2x2"
    color: "purple"
    pos: [6, 4, 0]
    yaw: 0

  # --- Floating block to test air gaps ---
  - id: 8
    type: "2x2"
    color: "lightblue"
    pos: [6, 4, 2]   # floating above purple block (air gap)
    yaw: 0

  # --- Block touching only corner ---
  - id: 9
    type: "1x1"
    color: "gray"
    pos: [5, 3, 0]
    yaw: 0
 """   

def load_yaml_string(yaml_str):
    data = yaml.safe_load(yaml_str)
    return data["bricks"]

# ------------------------------
# 3. 构建体素网格
# ------------------------------
def compute_grid_size(bricks):
    max_x = max_y = max_z = 0
    for b in bricks:
        x0, y0, z0 = b["pos"]
        sx, sy, sz = BRICK_SHAPES[b["type"]]
        max_x = max(max_x, x0 + sx)
        max_y = max(max_y, y0 + sy)
        max_z = max(max_z, z0 + sz)
    # 多留点边界
    return max_x + 3, max_y + 3, max_z + 3

def bricks_to_voxels(bricks):
    gx, gy, gz = compute_grid_size(bricks)
    vox = np.zeros((gx, gy, gz), dtype=int)

    for b in bricks:
        bid = b["id"]
        x0, y0, z0 = b["pos"]
        yaw = b.get("yaw", 0)
        sx, sy, sz = BRICK_SHAPES[b["type"]]

        if yaw != 0:
            raise NotImplementedError("示例只支持 yaw=0")

        for i in range(sx):
            for j in range(sy):
                for k in range(sz):
                    vox[x0 + i, y0 + j, z0 + k] = bid

    return vox

# ------------------------------
# 4. 小工具
# ------------------------------
def is_inside(vox, x, y, z):
    X, Y, Z = vox.shape
    return (0 <= x < X) and (0 <= y < Y) and (0 <= z < Z)

def voxel_center(x, y, z):
    return np.array([x + 0.5, y + 0.5, z + 0.5], dtype=float)

def brick_bbox(brick):
    x0, y0, z0 = brick["pos"]
    sx, sy, sz = BRICK_SHAPES[brick["type"]]   # 修正：用 brick 而不是 b
    yaw = brick.get("yaw", 0)
    if yaw != 0:
        raise NotImplementedError("示例只支持 yaw=0")
    return x0, y0, z0, sx, sy, sz

# ------------------------------
# 5. 沿整条边逐格检查：每一行 x / y 若两侧外邻都空，就各画一颗黑点
# ------------------------------
def scan_edge_pairs_for_brick(vox, brick, check_z=False):
    """
    对一块砖：
      - X 方向：对每个 (y,z) 行，如果 x_min-1 和 x_max+1 都是空的，就在这两个空 voxel 中心画点；
      - Y 方向：对每个 (x,z) 行同理；
      - Z 方向可选。
    """
    x0, y0, z0, sx, sy, sz = brick_bbox(brick)
    pts = []

    x_min = x0
    x_max = x0 + sx - 1
    y_min = y0
    y_max = y0 + sy - 1
    z_min = z0
    z_max = z0 + sz - 1

    X, Y, Z = vox.shape

    # ---- X 方向：对每个 (y,z) 行检查两侧外邻 ----
    for y in range(y_min, y_max + 1):
        for z in range(z_min, z_max + 1):
            nx_left  = x_min - 1
            nx_right = x_max + 1
            # 越界 = 墙/地面 = 占用
            if not (is_inside(vox, nx_left,  y, z) and is_inside(vox, nx_right, y, z)):
                continue
            if vox[nx_left,  y, z] == 0 and vox[nx_right, y, z] == 0:
                pts.append(voxel_center(nx_left,  y, z))
                pts.append(voxel_center(nx_right, y, z))

    # ---- Y 方向：对每个 (x,z) 行检查两侧外邻 ----
    for x in range(x_min, x_max + 1):
        for z in range(z_min, z_max + 1):
            ny_down = y_min - 1
            ny_up   = y_max + 1
            if not (is_inside(vox, x, ny_down, z) and is_inside(vox, x, ny_up, z)):
                continue
            if vox[x, ny_down, z] == 0 and vox[x, ny_up, z] == 0:
                pts.append(voxel_center(x, ny_down, z))
                pts.append(voxel_center(x, ny_up,   z))

    # ---- Z 方向（如果你之后要处理上下夹取，可以打开）----
    if check_z:
        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                nz_down = z_min - 1
                nz_up   = z_max + 1
                if not (is_inside(vox, x, y, nz_down) and is_inside(vox, x, y, nz_up)):
                    continue
                if vox[x, y, nz_down] == 0 and vox[x, y, nz_up] == 0:
                    pts.append(voxel_center(x, y, nz_down))
                    pts.append(voxel_center(x, y, nz_up))

    return pts

# ------------------------------
# 6. 可视化
# ------------------------------
def plot_bricks_with_edge_scan_points(vox, bricks, check_z=False):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    cmap = cm.get_cmap("tab10")
    id_to_color = {}

    # 画砖块
    for idx, b in enumerate(bricks):
        bid = b["id"]
        if bid not in id_to_color:
            id_to_color[bid] = cmap(idx % 10)
        color = id_to_color[bid]

        x0, y0, z0, sx, sy, sz = brick_bbox(b)
        ax.bar3d(
            x0, y0, z0,
            sx, sy, sz,
            color=color,
            alpha=0.6,
            shade=True,
            edgecolor="k",
            linewidth=0.5,
        )

    # 画黑点
    for b in bricks:
        pts = scan_edge_pairs_for_brick(vox, b, check_z=check_z)
        print(f"Brick {b['id']} points:", len(pts))
        if not pts:
            continue
        pts = np.array(pts)
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                   color="black", s=60, zorder=10)

    ax.set_xlabel("X (stud)")
    ax.set_ylabel("Y (stud)")
    ax.set_zlabel("Z (layer)")
    ax.set_title("LEGO bricks + edge scan grasp points")

    X, Y, Z = vox.shape
    ax.set_xlim(0, X)
    ax.set_ylim(0, Y)
    ax.set_zlim(0, Z)

    plt.tight_layout()
    plt.show()

# ------------------------------
# 7. main
# ------------------------------
if __name__ == "__main__":
    bricks = load_yaml_string(YAML_EXAMPLE)
    vox = bricks_to_voxels(bricks)
    print("Voxel grid shape:", vox.shape)
    # 先只看侧面抓取
    plot_bricks_with_edge_scan_points(vox, bricks, check_z=False)