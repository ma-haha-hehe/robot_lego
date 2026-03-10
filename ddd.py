import trimesh

mesh = trimesh.load('/home/i6user/Desktop/robot_lego/FoundationPose/meshes/LEGO_Duplo_brick_2x2.stl')

# 计算包围盒的中点 (min + max) / 2
center = mesh.bounds.mean(axis=0)

# 将所有顶点减去这个中心点
mesh.vertices -= center

# 导出覆盖原文件或保存新文件
mesh.export('centered_model.stl')