import yaml
import os
import numpy as np

# ================= 配置 =================
# 1. 输入和输出文件
YAML_FILE = "src/panda_pick/scripts/lego_structure.yaml"
XML_OUTPUT = "src/panda_pick/scripts/lego_structure.xml"

# 2. 积木在仿真里出现的位置偏移
# 比如你想让成品直接出现在桌子上 (桌高0.4m, x=0.5m)
OFFSET = np.array([0.4, 0.0, 0.42]) 
# =======================================

def generate_xml():
    # 1. 读取 YAML
    if not os.path.exists(YAML_FILE):
        print(f"错误: 找不到 {YAML_FILE}")
        return

    with open(YAML_FILE, 'r') as f:
        data = yaml.safe_load(f)
    
    blocks = data.get("blocks", []) if isinstance(data, dict) else data

    # 2. 开始构建 XML 字符串
    # 使用 <mujocoinclude> 标签，这样它可以被安全地 include 到 scene.xml 中
    xml_content = ["<mujocoinclude>"]
    xml_content.append(f'  ')
    xml_content.append('  <worldbody>')

    for i, block in enumerate(blocks):
        name = block.get("name", f"brick_{i}")
        
        # A. 获取位置并加偏移
        # YAML 里的 pos 是 [x, y, z]
        rel_pos = np.array(block.get("pos", [0, 0, 0]))
        abs_pos = rel_pos + OFFSET
        pos_str = f"{abs_pos[0]:.4f} {abs_pos[1]:.4f} {abs_pos[2]:.4f}"

        # B. 获取尺寸 (注意：MuJoCo 使用半长 size，而 YAML 里通常是全长 dims)
        # 假设 YAML 里的 dims 是 [长, 宽, 高] (全长)
        # 那么 MuJoCo size = dims / 2
        dims = np.array(block.get("dims", [0.03, 0.03, 0.03]))
        size = dims / 2
        size_str = f"{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}"

        # C. 颜色 (随机或固定)
        # 这里简单给个红色，你可以根据逻辑改
        rgba = "0.8 0.2 0.2 1" 

        # D. 生成 XML 节点
        # 定义 body 和 geom
        xml_content.append(f'    <body name="{name}" pos="{pos_str}">')
        xml_content.append(f'      <geom type="box" size="{size_str}" rgba="{rgba}" mass="0.05" friction="1 0.005 0.0001"/>')
        xml_content.append('      <freejoint/>') # 加上这个，积木才能被推倒！如果不加，积木就是固定在空中的
        xml_content.append('    </body>')

    xml_content.append('  </worldbody>')
    xml_content.append("</mujocoinclude>")

    # 3. 写入 XML 文件
    with open(XML_OUTPUT, 'w') as f:
        f.write("\n".join(xml_content))
    
    print(f"成功！已将 YAML 转换为 XML: {XML_OUTPUT}")

if __name__ == "__main__":
    generate_xml()