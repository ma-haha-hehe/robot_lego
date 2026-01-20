import yaml
import os
import numpy as np

# ================= 配置 =================
YAML_FILE = "src/panda_pick/scripts/lego_structure.yaml"
# 直接覆盖 scene.xml 引用的文件
XML_OUTPUT = "src/panda_pick/scripts/lego_structure.xml" 

# 布局配置
START_POS = np.array([0.3, -0.2]) # 起始 X, Y (注意这里不写 Z)
ROW_SPACING = 0.06     # 零件间距
TABLE_SURFACE_Z = 0.42 # 桌子表面高度 (桌子中心0.4 + 半厚0.02)
# =======================================

def generate_scattered_xml():
    if not os.path.exists(YAML_FILE):
        print(f"错误: 找不到 {YAML_FILE}")
        return

    with open(YAML_FILE, 'r') as f:
        data = yaml.safe_load(f)
    
    # 兼容处理
    raw_blocks = data.get("blocks", []) if isinstance(data, dict) else data
    if isinstance(raw_blocks, dict):
        if "pos" in raw_blocks: raw_blocks = [raw_blocks]
        else: raw_blocks = list(raw_blocks.values())

    xml_content = ["<mujocoinclude>"]
    xml_content.append(f'  ')
    xml_content.append('  <worldbody>')

    current_y = START_POS[1]
    current_x = START_POS[0]
    
    for i, block in enumerate(raw_blocks):
        name = block.get("name", f"brick_{i}")
        
        # 1. 获取尺寸 (YAML里通常是全长)
        dims = np.array(block.get("dims", [0.03, 0.03, 0.03]))
        # 2. 计算半长 (MuJoCo size)
        size = dims / 2
        
        # 3. 【关键修复】动态计算 Z 轴高度
        # 中心高度 = 桌子表面 + 积木半高 + 1mm微小缝隙(防止穿模)
        block_z = TABLE_SURFACE_Z + size[2] + 0.001
        
        pos_str = f"{current_x:.4f} {current_y:.4f} {block_z:.4f}"
        size_str = f"{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}"

        # 排列逻辑
        current_y += ROW_SPACING
        if current_y > 0.2: 
            current_y = START_POS[1]
            current_x += 0.06 # 换行

        # 颜色 (蓝色系表示零件)
        rgba = "0.2 0.6 0.8 1" 

        xml_content.append(f'    <body name="{name}" pos="{pos_str}">')
        xml_content.append(f'      <geom type="box" size="{size_str}" rgba="{rgba}" mass="0.05" friction="1 0.005 0.0001"/>')
        xml_content.append('      <freejoint/>') 
        xml_content.append('    </body>')

    xml_content.append('  </worldbody>')
    xml_content.append("</mujocoinclude>")

    with open(XML_OUTPUT, 'w') as f:
        f.write("\n".join(xml_content))
    
    print(f"成功！已生成修复高度的 XML: {XML_OUTPUT}")

if __name__ == "__main__":
    generate_scattered_xml()