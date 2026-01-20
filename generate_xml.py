import numpy as np
import os

# =================配置=================
BRICK_METRIC_SIZE = 0.03  # 3cm
GEOM_SIZE = BRICK_METRIC_SIZE / 2 
OUTPUT_FILE = "lego_structure.xml"

COLORS = [
    "0.8 0.2 0.2 1", "0.2 0.8 0.2 1", "0.2 0.2 0.8 1", 
    "0.8 0.8 0.2 1", "0.2 0.8 0.8 1", "0.8 0.2 0.8 1", "0.8 0.5 0.2 1"
]

def get_robot_layout():
    layout = []
    def add(x, y, z, c):
        layout.append({"grid": (x, y, z), "color_id": c})
    # 定义结构 (保持不变)
    for z in range(3): add(0, 1, z, 0); add(0, -1, z, 0)
    add(0, 1, 3, 1); add(0, 0, 3, 2); add(0, -1, 3, 1)
    add(0, 2, 4, 3); add(0, 1, 4, 3); add(0, 0, 4, 4); add(0, -1, 4, 3); add(0, -2, 4, 3)
    add(0, 2, 5, 5); add(0, -2, 5, 5)
    add(0, 0, 5, 4); add(0, 0, 6, 6); add(0, 0, 7, 6)
    return layout

def generate_mujoco_xml(layout):
    xml_content = "<mujoco>\n  <worldbody>\n"
    # [关键修改] Z=0.42 放在桌面上 (桌子高0.4, 半厚0.02)
    xml_content += f"    <body name=\"lego_base\" pos=\"0.5 0 0.42\">\n" 

    for i, item in enumerate(layout):
        gx, gy, gz = item["grid"]
        c_id = item["color_id"] % len(COLORS)
        
        px = gx * BRICK_METRIC_SIZE
        py = gy * BRICK_METRIC_SIZE
        pz = (gz * BRICK_METRIC_SIZE) + GEOM_SIZE 
        
        pos_str = f"{px:.3f} {py:.3f} {pz:.3f}"
        rgba = COLORS[c_id]
        
        xml_content += f"      <body name=\"brick_{i}\" pos=\"{pos_str}\">\n"
        # 增加摩擦力 friction="1 0.005 0.0001" 以便抓取
        xml_content += f"        <geom type=\"box\" size=\"{GEOM_SIZE} {GEOM_SIZE} {GEOM_SIZE}\" rgba=\"{rgba}\" group=\"1\" mass=\"0.05\" friction=\"1.5 0.005 0.0001\" />\n"
        xml_content += f"      </body>\n"

    xml_content += "    </body>\n  </worldbody>\n</mujoco>\n"

    with open(OUTPUT_FILE, "w") as f:
        f.write(xml_content)
    print(f"XML 生成完毕: {os.path.abspath(OUTPUT_FILE)}")

if __name__ == "__main__":
    generate_mujoco_xml(get_robot_layout())