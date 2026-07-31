import os

import mujoco
import mujoco.viewer

SCENE_PATH = os.path.expanduser('~/unitree_mujoco/unitree_robots/go2/scene.xml')

def main():
    print("Openning Simulator...")

    model = mujoco.MjModel.from_xml_path(SCENE_PATH)                           # pyright: ignore[reportAttributeAccessIssue]
    data = mujoco.MjData(model)                                                # pyright: ignore[reportAttributeAccessIssue]
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():


            mujoco.mj_step(model, data)                                        # pyright: ignore[reportAttributeAccessIssue]
            viewer.sync()
