import os
import time
import threading
import queue
import numpy as np

import mujoco
import mujoco.viewer

from data_structures import Action
from hardware_abstraction_layer import OutputLayer

from kinematic_controller.gait_definition import LEG_COUNT
from kinematic_controller.ik_solver import _HIP_ABDUCTOR_TORQUE_LIMIT, _HIP_TORQUE_LIMIT, _KNEE_TORQUE_LIMIT


SCENE_PATH = os.path.expanduser('~/unitree_mujoco/unitree_robots/go2/scene.xml')

class Simulator(OutputLayer):
    def __init__(self):
        self._action_queue:queue.Queue[Action] = queue.Queue()

    def _run(self, interrupt:threading.Event):
        model = mujoco.MjModel.from_xml_path(SCENE_PATH)                           # pyright: ignore[reportAttributeAccessIssue]
        data = mujoco.MjData(model)                                                # pyright: ignore[reportAttributeAccessIssue]
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while ( not interrupt.is_set() ) and ( viewer.is_running() ):
                step_start = time.time()

                # TODO: Handle target angles and torques properly
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

    def connect(self, terminate_connection:threading.Event):
        print("[SO]  Openning simulator...")
        self._run(terminate_connection)

    def send_action(self, action:Action):
        self._action_queue.put(action)

    def get_low_state(self):
        pass


    # TODO: Later, dynamics model
    def _get_feedforward_torques(self):
        return np.array(([_HIP_ABDUCTOR_TORQUE_LIMIT.maximum, _HIP_TORQUE_LIMIT.maximum, _KNEE_TORQUE_LIMIT.maximum] * LEG_COUNT), dtype=np.float64)

    def _send_action_preset(self, angle_targets_dict:dict):
        # Convert dict of tuples into 1D array
        target_angles = np.array([item for tup in angle_targets_dict.values() for item in tup])

        action = Action(target_angles, self._get_feedforward_torques())
        self.send_action(action)


    #     Angle data from: https://github.com/maanas444/go2-simulation/blob/main/mujoco/go2_IK.py#L244
    def send_stand_action(self):
        REAL_STAND = {
            "FL": (-0.018,  0.663, -1.369),
            "FR": ( 0.018,  0.667, -1.377),
            "RL": (-0.082,  0.658, -1.351),
            "RR": ( 0.085,  0.660, -1.353),
        }
        self._send_action_preset(REAL_STAND)

    #     Angle data from: https://github.com/maanas444/go2-simulation/blob/main/mujoco/go2_IK.py#L251
    def send_lay_down_action(self):
        REAL_SIT = {
            "FL": (-0.068,  1.241, -2.770),
            "FR": ( 0.061,  1.236, -2.761),
            "RL": (-0.402,  1.244, -2.758),
            "RR": ( 0.383,  1.243, -2.756),
        }
        self._send_action_preset(REAL_SIT)