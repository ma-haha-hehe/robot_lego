# /home/i6user/Desktop/robot_lego/src/my_robot_vision/FoundationPose/__init__.py

import os
import sys

# 自动将当前目录加入系统路径
# 这样当你在外部 import FoundationPose 时，它内部的文件也能互相找到
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)
    # 如果有 root 文件夹，也一并加入
    root_dir = os.path.join(current_dir, "root")
    if os.path.exists(root_dir):
        sys.path.append(root_dir)

# 可选：显式暴露核心类，方便外部调用
try:
    from .estimater import ScorePredictor, PoseRefinePredictor
except ImportError:
    # 如果 estimater 还没准备好，先跳过
    pass