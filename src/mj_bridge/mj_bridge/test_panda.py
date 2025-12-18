import mujoco
import mujoco_viewer  # pip install mujoco mujoco-python-viewer

model = mujoco.MjModel.from_xml_path("scene.xml")#panda.xml  # 或者 "scene.xml"
data = mujoco.MjData(model)
viewer = mujoco_viewer.MujocoViewer(model, data)

while viewer.is_alive:
    mujoco.mj_step(model, data)
    viewer.render()
