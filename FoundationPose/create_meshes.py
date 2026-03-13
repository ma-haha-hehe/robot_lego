import trimesh
import numpy as np
import os

def create_lego_brick(rows, cols, filename):
    # 乐高标准尺寸 (单位: 毫米 -> 米)
    PITCH = 8.0 * 0.001
    WIDTH = rows * PITCH
    LENGTH = cols * PITCH
    HEIGHT = 9.6 * 0.001
    STUD_DIA = 4.8 * 0.001
    STUD_HEIGHT = 1.7 * 0.001
    
    # 1. 砖身 (移动到底面 Z=0)
    base = trimesh.creation.box(extents=[WIDTH, LENGTH, HEIGHT])
    base.apply_translation([0, 0, HEIGHT / 2.0])

    # 2. 凸起 (Studs)
    studs = []
    start_x = -((rows - 1) * PITCH) / 2.0
    start_y = -((cols - 1) * PITCH) / 2.0

    for r in range(rows):
        for c in range(cols):
            x = start_x + r * PITCH
            y = start_y + c * PITCH
            z = HEIGHT + (STUD_HEIGHT / 2.0)
            stud = trimesh.creation.cylinder(radius=STUD_DIA/2, height=STUD_HEIGHT, sections=16)
            stud.apply_translation([x, y, z])
            studs.append(stud)

    # 3. 合并
    lego_mesh = trimesh.util.concatenate([base] + studs)
    
    # 确保目录存在
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    lego_mesh.export(filename)
    print(f"✅ Generated: {filename}")

if __name__ == "__main__":
    create_lego_brick(2, 4, "meshes/lego_2x4.obj")
    create_lego_brick(2, 2, "meshes/lego_2x2.obj")