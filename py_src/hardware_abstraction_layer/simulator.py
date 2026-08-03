import os
import time
import threading
import queue
import numpy as np

import mujoco
import mujoco.viewer

from data_structures import Action
from hardware_abstraction_layer import OutputLayer

from data_structures.gait_definition import LEG_COUNT, JOINT_COUNT
from kinematics.ik_solver import _HIP_ABDUCTOR_TORQUE_LIMIT, _HIP_TORQUE_LIMIT, _KNEE_TORQUE_LIMIT

from control.pid import PIDController


SCENE_PATH = os.path.expanduser('~/unitree_mujoco/unitree_robots/go2/scene.xml')

# MuJoCo Indexes
#     from: https://github.com/maanas444/go2-simulation/blob/main/unitreego2/main.py#L18
CTRL_INDEXES = np.array([[0, 1, 2], [3, 4, 5], [6, 7, 8], [9, 10, 11]]).flatten()
QPOS_INDEXES = np.array([[7, 8, 9], [10, 11, 12], [13, 14, 15], [16, 17, 18]]).flatten()
QVEL_INDEXES = np.array([[6, 7, 8], [9, 10, 11], [12, 13, 14], [15, 16, 17]]).flatten()
# TODO: Implement cleaner alternative
# CTRL_IDX = slice(0, 12)    # 0 to 11
# QPOS_IDX = slice(7, 19)    # 7 to 18
# QVEL_IDX = slice(6, 18)    # 6 to 17

class Simulator(OutputLayer):
    _REAL_LAY_DOWN = {
        "FR": ( 0.061,  1.236, -2.761),
        "FL": (-0.068,  1.241, -2.770),
        "RR": ( 0.383,  1.243, -2.756),
        "RL": (-0.402,  1.244, -2.758),
    } # from: https://github.com/maanas444/go2-simulation/blob/main/mujoco/go2_IK.py#L244
    _REAL_STAND = {
        "FR": ( 0.018,  0.667, -1.377),
        "FL": (-0.018,  0.663, -1.369),
        "RR": ( 0.085,  0.660, -1.353),
        "RL": (-0.082,  0.658, -1.351),
    } # from: https://github.com/maanas444/go2-simulation/blob/main/mujoco/go2_IK.py#L244

    def __init__(self, pids:list[PIDController]):
        self._action_queue:queue.Queue[Action] = queue.Queue()

        self.pids = pids

    # TODO: Fix dog not responding
    def _run(self, interrupt:threading.Event):
        model = mujoco.MjModel.from_xml_path(SCENE_PATH)                           # pyright: ignore[reportAttributeAccessIssue]
        data = mujoco.MjData(model)                                                # pyright: ignore[reportAttributeAccessIssue]

        # Timestep
        DT = 0.002  # 500Hz loop
        model.opt.timestep = DT

        # Persist angle targets
        target_angles = self._get_target_angles_from_dict(self._REAL_LAY_DOWN)

        # Run simulation loop
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while ( not interrupt.is_set() ) and ( viewer.is_running() ):
                step_start = time.time()

                # Create/Clear feedforward torque buffer
                feedforward_torques = np.zeros(JOINT_COUNT)

                # 1. Update targets if a new action has arrived
                try:
                    # Non-blocking check for new event
                    new_action = self._action_queue.get_nowait()

                    target_angles       = new_action.target_angles
                    feedforward_torques = new_action.feedforward_torques
                except queue.Empty:
                    pass  # Keep previous action in data.ctrl automatically

                # 2. Get current state from MuJoCo
                current_angles     = data.qpos[QPOS_INDEXES]
                current_velocities = data.qvel[QVEL_INDEXES]

                # 3. Calculate applied torques using PID controller
                applied_torques = np.empty(JOINT_COUNT)
                zipped = zip(self.pids, target_angles, current_angles, current_velocities)

                for i, (pid, target, current, vel) in enumerate(zipped):
                    applied_torques[i] = pid.update(target, current, vel, DT)

                # 4. Send the calculated applied torques to MuJoCo
                data.ctrl[:] = (applied_torques + feedforward_torques)  # TODO: Use CTRL_INDEXES

                # # TODO: Remove, for debugging
                # if int(time.time() * 1000) % 100 == 0:
                #     threshold = 1e-6
                #     _applied_torques = np.where(np.abs(applied_torques) < threshold, 0, applied_torques)
                #     _feedforward_torques = np.where(np.abs(feedforward_torques) < threshold, 0, feedforward_torques)
                #
                #     np.set_printoptions(suppress=True, precision=6)
                #     print(f"{str(int(time.time() * 1000))[-6:-2]} | {_applied_torques} + {_feedforward_torques}")

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

    def _get_target_angles_from_dict(self, preset_dict:dict):
        # Convert dict of tuples into 1D array
        return np.array([item for tup in preset_dict.values() for item in tup])

    def _send_action_preset(self, angle_targets_dict:dict):
        target_angles = self._get_target_angles_from_dict(angle_targets_dict)

        action = Action(target_angles, self._get_feedforward_torques())
        self.send_action(action)

    # Presets
    # TODO: Add in transition between sit (lay down) and stand. (Just queue up those actions for each time step?)
    def send_stand_action(self):
        self._send_action_preset(self._REAL_STAND)

    def send_lay_down_action(self):
        self._send_action_preset(self._REAL_LAY_DOWN)