import os
import time
import queue

import mujoco
import mujoco.viewer

from data_structures import Action

from hardware_abstraction_layer import OutputLayer


SCENE_PATH = os.path.expanduser('~/unitree_mujoco/unitree_robots/go2/scene.xml')

class Simulator(OutputLayer):
    def __init__(self):
        self._action_queue:queue.Queue[Action] = queue.Queue()

    def connect(self):
        model = mujoco.MjModel.from_xml_path(SCENE_PATH)                           # pyright: ignore[reportAttributeAccessIssue]
        data = mujoco.MjData(model)                                                # pyright: ignore[reportAttributeAccessIssue]
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                step_start = time.time()

                # Event Check: Has an action event arrived?
                try:
                    # Non-blocking check for new event
                    new_action = self._action_queue.get_nowait()
                    data.ctrl[:] = new_action
                except queue.Empty:
                    pass  # Keep previous action in data.ctrl automatically

                # Physics engine steps continuously holding the last action state
                mujoco.mj_step(model, data)                                        # pyright: ignore[reportAttributeAccessIssue]
                viewer.sync()

                # Maintain physics rate
                time_until_next_step = model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    def send_action(self, target_angles, feedforward_torques):
        action = Action(target_angles, feedforward_torques)
        self._action_queue.put(action)

    def get_low_state(self):
        pass
