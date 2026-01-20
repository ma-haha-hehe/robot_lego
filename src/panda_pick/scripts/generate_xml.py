import numpy as np
import os

# =================配置=================
BRICK_METRIC_SIZE = 0.03  # 3cm
GEOM_SIZE = BRICK_METRIC_SIZE / 2 
OUTPUT_FILE = "lego_structure.xml"

# 颜色定义 (RGBA)
COLORS = [
    "0.8 0.2 0.2 1", "0.2 0.8 0.2 1", "0.2 0.2 0.8 1", 
    "0.8 0.8 0.2 1", "0.2 0.8 0.8 1", "0.8 0.2 0.8 1", "0.8 0.5 0.2 1"
]

def get_robot_layout():
    layout = []
    def add(x, y, z, c):
        layout.append({"grid": (x, y, z), "color_id": c})
    # A. 双腿 (Z=0,1,2)
    for z in range(3): add(0, 1, z, 0); add(0, -1, z, 0)
    # B. 胯部 (Z=3)
    add(0, 1, 3, 1); add(0, 0, 3, 2); add(0, -1, 3, 1)
    # C. 躯干 (Z=4)
    add(0, 2, 4, 3); add(0, 1, 4, 3); add(0, 0, 4, 4); add(0, -1, 4, 3); add(0, -2, 4, 3)
    # D. 手部 (Z=5)
    add(0, 2, 5, 5); add(0, -2, 5, 5)
    # E. 头部 (Z=5,6,7)
    add(0, 0, 5, 4); add(0, 0, 6, 6); add(0, 0, 7, 6)
    return layout

def generate_mujoco_xml(layout):
    # 放置位置调整到机器人前方 0.5m
    xml_content = "<mujoco>\n  <worldbody>\n"
    xml_content += f"    <body name=\"lego_base\" pos=\"0.5 0 0\">\n" 

    for i, item in enumerate(layout):
        gx, gy, gz = item["grid"]
        c_id = item["color_id"] % len(COLORS)
        
        # 坐标计算 (Z轴从0开始，物理高度=gz*0.03 + 半个积木高)
        px = gx * BRICK_METRIC_SIZE
        py = gy * BRICK_METRIC_SIZE
        pz = (gz * BRICK_METRIC_SIZE) + GEOM_SIZE 
        
        pos_str = f"{px:.3f} {py:.3f} {pz:.3f}"
        rgba = COLORS[c_id]
        
        xml_content += f"      <body name=\"brick_{i}\" pos=\"{pos_str}\">\n"
        xml_content += f"        <geom type=\"box\" size=\"{GEOM_SIZE} {GEOM_SIZE} {GEOM_SIZE}\" rgba=\"{rgba}\" group=\"1\" mass=\"0.05\" />\n"
        xml_content += f"      </body>\n"

    xml_content += "    </body>\n  </worldbody>\n</mujoco>\n"

    with open(OUTPUT_FILE, "w") as f:
        f.write(xml_content)
    print(f"生成的 XML 文件位置: {os.path.abspath(OUTPUT_FILE)}")

if __name__ == "__main__":
    generate_mujoco_xml(get_robot_layout())