import trimesh
import os

# ===在这里修改你的方块尺寸 (单位: 米)===
# 假设是 5cm x 5cm x 8cm 的大方块
WIDTH  = 0.05   # 宽 5厘米
LENGTH = 0.05   # 长 5厘米
HEIGHT = 0.08   # 高 8厘米 (看截图感觉比较高)

def create_simple_box():
    # 生成一个简单的长方体
    mesh = trimesh.creation.box(extents=[WIDTH, LENGTH, HEIGHT])
    
    # 导出为 .obj
    save_path = "meshes/yellow_block.obj"
    os.makedirs("meshes", exist_ok=True)
    mesh.export(save_path)
    print(f"✅ 已生成临时模型: {save_path}")
    print(f"   尺寸: {WIDTH*100}x{LENGTH*100}x{HEIGHT*100} cm")

if __name__ == "__main__":
    create_simple_box()