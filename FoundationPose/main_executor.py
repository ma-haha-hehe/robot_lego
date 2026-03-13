import numpy as np
import yaml
import os
import time
from scipy.spatial.transform import Rotation as R

class RobotSystem:
    """
    机器人系统配置类：存储相机外参及物理坐标偏移
    """
    # --- 物理配置 (基于方案 B 验证结果) ---
    # 相机位置: 前 51cm(0.51), 右 37cm(-0.37), 高 11cm(0.11)
    T_CAM_TO_BASE = np.eye(4)
    T_CAM_TO_BASE[0:3, 3] = [0.51, -0.37, 0.11]

    # 旋转配置 (方案 B)
    # 映射关系: Base X = Cam X; Base Y = -Cam Z; Base Z = Cam Y
    _r_align = np.array([
        [1,  0,  0],  # 相机 X 映射到 机器人 X
        [0,  0, -1],  # 相机 Z 的负向映射到 机器人 Y
        [0,  1,  0]   # 相机 Y 映射到 机器人 Z
    ])

    # 俯视 20 度补偿 (绕相机自身的 X 轴旋转)
    _r_tilt = R.from_euler('x', 20, degrees=True).as_matrix()

    # 最终旋转矩阵 = 轴向重映射 @ 俯视旋转
    T_CAM_TO_BASE[0:3, 0:3] = _r_align @ _r_tilt

    # 组装区中心相对于基座 (根据实验室布局实际情况调整)
    ASSEMBLY_CENTER_BASE = np.array([0.20, 0.0, 0.0])

class Executor:
    """
    任务执行逻辑类：协调视觉服务与机器人动作指令生成
    """
    def __init__(self):
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.task_file = os.path.join(self.base_dir, "tasks.yaml")
        self.active_file = os.path.join(self.base_dir, "active_task.yaml")
        self.target_file = os.path.join(self.base_dir, "current_target.txt")
        self.vision_file = os.path.join(self.base_dir, "vision_output.yaml")

    def _extract_T_obj_cam(self, v_data, label: str):
        """
        解析并兼容不同格式的视觉输出文件
        - 新格式：顶层包含 target 和 T_cam_obj 字段
        - 旧格式：直接以物体名称为 key，其值为 4x4 矩阵
        """
        if not isinstance(v_data, dict):
            return None

        # ===== 尝试解析新格式 =====
        if "T_cam_obj" in v_data:
            tgt = str(v_data.get("target", "")).strip()
            # 校验目标名称，确保读取的是当前需要的物体
            if tgt and tgt != label:
                return None
            T = np.array(v_data["T_cam_obj"], dtype=np.float32)
            if T.shape == (4, 4):
                return T
            return None

        # ===== 尝试解析旧格式 =====
        if label in v_data:
            T = np.array(v_data[label], dtype=np.float32)
            if T.shape == (4, 4):
                return T
            return None

        return None

    def run(self):
        if not os.path.exists(self.task_file):
            print(f"❌ 找不到原始任务文件: {self.task_file}")
            return

        print(">>> 正在加载任务清单...")
        with open(self.task_file, 'r', encoding="utf-8") as f:
            tasks_data = yaml.safe_load(f)
            tasks = tasks_data.get('tasks', [])

        for task in tasks:
            label = task['name']
            print(f"\n🚀 --- 正在处理目标: {label} ---")

            # 1. 触发视觉识别：向 current_target.txt 写入物体名
            with open(self.target_file, 'w', encoding="utf-8") as f:
                f.write(label)

            # 2. 预留时间：给视觉算法处理和稳定数据的缓冲
            print(f"⏳ 正在等待 5 秒以获取最稳定的位姿...")
            time.sleep(5.0)

            # 3. 读取视觉输出结果
            real_pick_base = None
            if os.path.exists(self.vision_file):
                try:
                    with open(self.vision_file, 'r', encoding="utf-8") as f:
                        v_data = yaml.safe_load(f)

                    t_obj_cam = self._extract_T_obj_cam(v_data, label)
                    if t_obj_cam is not None:
                        # 坐标转换逻辑：物体在基座坐标系 = T_相机到基座 @ 物体在相机坐标系
                        real_pick_base = RobotSystem.T_CAM_TO_BASE @ t_obj_cam
                        print("🎯 已获取最新的位姿转换结果。")
                except Exception as e:
                    print(f"⚠️ 读取视觉输出文件失败: {e}")

            if real_pick_base is None:
                print(f"❌ 错误: 5 秒后仍未检测到有效结果，跳过此任务。")
                continue

            # 4. 组装最终任务参数
            # 抓取位姿
            final_pick_pos = real_pick_base[0:3, 3].tolist()
            final_pick_ori = task['pick']['orientation']  # 保持任务文件定义的抓取方向

            # 放置位姿 (基于组装区中心点的相对偏移)
            raw_place_pos = np.array(task['place']['pos'])
            final_place_pos = (RobotSystem.ASSEMBLY_CENTER_BASE + raw_place_pos).tolist()
            final_place_ori = task['place']['orientation']

            # 5. 生成 active_task.yaml：这是给机械臂控制节点的最终指令
            active_step = {
                "id": task['id'],
                "name": label,
                "pick": {
                    "pos": final_pick_pos, 
                    "orientation": final_pick_ori
                },
                "place": {
                    "pos": final_place_pos, 
                    "orientation": final_place_ori
                }
            }

            with open(self.active_file, 'w', encoding="utf-8") as f:
                yaml.dump(active_step, f, default_flow_style=False, sort_keys=False)

            print(f"✅ 实时指令已生成: {self.active_file}")
            print(f"📍 抓取点 (Base): {[round(i, 4) for i in final_pick_pos]}")
            print(f"📍 放置点 (Base): {[round(i, 4) for i in final_place_pos]}")

            # 6. 文件握手：等待机器人控制器执行完毕并删除 active_task.yaml
            print("⏳ 等待机器人执行动作...")
            while os.path.exists(self.active_file):
                time.sleep(1)

            # 任务结束后清理临时文件，防止下一轮误读
            print(f"🧹 任务 [{label}] 完成，正在清理中间数据...")
            if os.path.exists(self.target_file):
                os.remove(self.target_file)
            if os.path.exists(self.vision_file):
                os.remove(self.vision_file)

if __name__ == "__main__":
    try:
        Executor().run()
    except KeyboardInterrupt:
        print("\n👋 停止执行。")