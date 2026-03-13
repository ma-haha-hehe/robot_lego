import trimesh
import numpy as np

# 加载你的 STL 文件
mesh = trimesh.load("LEGO_Duplo_brick_4x2.stl")

# 执行你代码里的居中逻辑，确保观察的是处理后的模型
mesh.vertices -= mesh.bounds.mean(axis=0)

# 显示模型并附带坐标轴 (红色=X, 绿色=Y, 蓝色=Z)
mesh.show()