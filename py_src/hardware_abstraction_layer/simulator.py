import os
import time
import threading
import queue
from pathlib import Path
import numpy as np

import mujoco
import mujoco.viewer

from data_structures import Position, LegPoseList
from hardware_abstraction_layer import OutputLayer

from data_structures.gait_definition import LEG_COUNT, JOINT_COUNT
from kinematics.ik_solver import _HIP_ABDUCTOR_TORQUE_LIMIT, _HIP_TORQUE_LIMIT, _KNEE_TORQUE_LIMIT

from control.pid import PIDController


SCENE_PATH = Path(os.environ.get(
    "UNITREE_MUJOCO_SCENE",
    "~/unitree_mujoco/unitree_robots/go2/scene.xml",
)).expanduser()

# MuJoCo Indexes
#     from: https://github.com/maanas444/go2-simulation/blob/main/unitreego2/main.py#L18
CTRL_INDEXES = np.array([[0, 1, 2], [3, 4, 5], [6, 7, 8], [9, 10, 11]]).flatten()         # 0 to 11, Output signals (Torque or Angles??)
# QPOS_INDEXES = np.array([[7, 8, 9], [10, 11, 12], [13, 14, 15], [16, 17, 18]]).flatten()  # 7 to 18, Joint Angles
# QVEL_INDEXES = np.array([[6, 7, 8], [9, 10, 11], [12, 13, 14], [15, 16, 17]]).flatten()   # 6 to 17, Joint Angular Velocities
QPOS_INDEXES = np.array([[10, 11, 12], [7, 8, 9], [16, 17, 18], [13, 14, 15]]).flatten()
QVEL_INDEXES = np.array([[9, 10, 11], [6, 7, 8], [15, 16, 17], [12, 13, 14]]).flatten()
# TODO: Implement cleaner alternative
# CTRL_IDX = slice(0, 12)    # 0 to 11
# QPOS_IDX = slice(7, 19)    # 7 to 18
# QVEL_IDX = slice(6, 18)    # 6 to 17


class _SimulatorIMU:
    def __init__(self, accelerometer, gyroscope):
        self.accelerometer = np.asarray(accelerometer, dtype=np.float64)
        self.gyroscope = np.asarray(gyroscope, dtype=np.float64)


class _SimulatorLowState:
    def __init__(self, accelerometer, gyroscope, foot_force):
        self.imu_state = _SimulatorIMU(accelerometer, gyroscope)
        self.foot_force = np.asarray(foot_force, dtype=np.float64)

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
    _FOOT_NAMES = ("FL", "FR", "RL", "RR")

    def __init__(self, pids:list[PIDController]):
        self._position_queue:queue.Queue[Position] = queue.Queue()
        self._state_lock = threading.Lock()
        self._low_state = _SimulatorLowState(
            accelerometer=(0.0, 0.0, 9.81),
            gyroscope=(0.0, 0.0, 0.0),
            foot_force=(0.0, 0.0, 0.0, 0.0),
        )

        self.pids = pids

    def _update_low_state(self, data, foot_geom_ids:dict[str, int]):
        contacts = np.zeros(4, dtype=np.float64)
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            for foot_index, foot_name in enumerate(self._FOOT_NAMES):
                foot_geom_id = foot_geom_ids[foot_name]
                if foot_geom_id in (contact.geom1, contact.geom2):
                    contacts[foot_index] = 100.0

        low_state = _SimulatorLowState(
            accelerometer=data.sensor("imu_acc").data.copy(),
            gyroscope=data.sensor("imu_gyro").data.copy(),
            foot_force=contacts,
        )
        with self._state_lock:
            self._low_state = low_state

    def _set_pose(self, data, pose, z_height:float):
        data.qpos[:] = 0.0  # Reset positions
        data.qvel[:] = 0.0  # Reset velocities

        # Set position and orientation
        data.qpos[2] = z_height
        data.qpos[3] = 1.0  # Set orientation; (w: real part of quaternion)
        # data.qpos[4] = np.pi  # Start upside-down; roll 180

        # Apply pose
        data.qpos[QPOS_INDEXES] = pose[:12]

    # TODO: Fix dog not responding
    def _run(self, interrupt:threading.Event):
        if not SCENE_PATH.is_file():
            raise FileNotFoundError(
                f"MuJoCo scene not found: {SCENE_PATH}\n"
                "Clone unitree_mujoco or set UNITREE_MUJOCO_SCENE to the Go2 "
                "scene.xml path."
            )

        model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))                      # pyright: ignore[reportAttributeAccessIssue]
        data = mujoco.MjData(model)                                                # pyright: ignore[reportAttributeAccessIssue]

        # Timestep
        DT = 0.002  # 500Hz loop
        model.opt.timestep = DT

        # Initialise position
        initial_pose = self._get_target_angles_from_dict(self._REAL_STAND)
        self._set_pose(data, initial_pose, z_height=0.30)  # z_height is in meters
        mujoco.mj_forward(model, data)                                         # pyright: ignore[reportAttributeAccessIssue]
        foot_geom_ids = {
            foot_name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, foot_name)
            for foot_name in self._FOOT_NAMES
        }
        self._update_low_state(data, foot_geom_ids)

        # Persist angle targets
        target_angles = initial_pose

        # Bounded catch-up setup
        sim_time = 0.0
        wall_origin = time.perf_counter()

        # Run simulation loop
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while ( not interrupt.is_set() ) and ( viewer.is_running() ):

                # Catch physics up to real time
                #     but capped so that a stall doesn't trigger a step runaway
                target_sim = min(time.perf_counter() - wall_origin, sim_time + 0.050) # Capped at 50ms

                while sim_time < target_sim:

                    # Create/Clear feedforward torque buffer
                    feedforward_torques = np.zeros(JOINT_COUNT)

                    # 1. Update targets if a new position has arrived
                    try:
                        # Non-blocking check for new event
                        new_position = self._position_queue.get_nowait()

                        target_angles       = new_position.target_angles
                        feedforward_torques = new_position.feedforward_torques
                    except queue.Empty:
                        pass  # Keep previous position in data.ctrl automatically  # TODO: Correct comment

                    # 2. Get current state from MuJoCo
                    current_angles     = data.qpos[QPOS_INDEXES]
                    current_velocities = data.qvel[QVEL_INDEXES]

                    # 3. Calculate applied torques using PID controller
                    applied_torques = np.empty(JOINT_COUNT)
                    zipped = zip(self.pids, target_angles, current_angles, current_velocities)

                    for i, (pid, target, angle, vel) in enumerate(zipped):
                        applied_torques[i] = pid.update(target, angle, vel, DT)

                    # 4. Send the calculated applied torques to MuJoCo
                    data.ctrl[:12] = applied_torques  # TODO: Use CTRL_INDEXES
                    # data.ctrl[:12] = (applied_torques + feedforward_torques)  # TODO: Use CTRL_INDEXES

                    # for leg in range(4):
                    #     leg_num = leg * 3
                    #     hip_t, thigh_t, calf_t = target_angles[leg_num:(leg_num+3)]

                    #     targets = [hip_t, thigh_t, calf_t]
                    #     for j in range(3):
                    #         current_p = data.qpos[QPOS_INDEXES[leg_num + j]]
                    #         current_v = data.qvel[QVEL_INDEXES[leg_num + j]] 
                    #         data.ctrl[leg_num + j] = self.pids[leg_num + j].update(targets[j], current_p, current_v, DT)

                    # # TODO: Remove, for debugging
                    # if int(time.time() * 1000) % 100 == 0:
                    #     # threshold = 1e-6
                    #     # _applied_torques = np.where(np.abs(applied_torques) < threshold, 0, applied_torques)
                    #     # _feedforward_torques = np.where(np.abs(feedforward_torques) < threshold, 0, feedforward_torques)
                    
                    #     np.set_printoptions(suppress=True, precision=6)
                    #     # print(f"{str(int(time.time() * 1000))[-6:-2]} | {_applied_torques} + {_feedforward_torques}")
                    #     print(f"[S2]  {str(int(time.time() * 1000))[-6:-2]}")
                    #     # print(f"[S2]        : {np.asarray(range(20))}")
                    #     # print(f"[S2]    d_qp: {data.qpos}")
                    #     # print(f"[S2]    d_qv: {data.qvel}")
                    #     print(f"[S2]    d_ct: {data.ctrl}")
                    #     # print(f"[S2]    ap_t: {_applied_torques}")
                    #     # print(f"[S2]    ff_t: {_feedforward_torques}")
                    #     print(f"[S2]    t_ag: {target_angles}")

                    mujoco.mj_step(model, data)                                        # pyright: ignore[reportAttributeAccessIssue]
                    self._update_low_state(data, foot_geom_ids)
                    sim_time += DT

                viewer.sync()  # Render
    
    def connect(self, terminate_connection:threading.Event):
        print("[SO]  Opening simulator...")
        self._run(terminate_connection)

    def send_position(self, position:Position):
        self._position_queue.put(position)

    def get_low_state(self):
        with self._state_lock:
            return _SimulatorLowState(
                accelerometer=self._low_state.imu_state.accelerometer.copy(),
                gyroscope=self._low_state.imu_state.gyroscope.copy(),
                foot_force=self._low_state.foot_force.copy(),
            )


    # TODO: Later, dynamics model
    def _get_feedforward_torques(self):
        return np.array(([_HIP_ABDUCTOR_TORQUE_LIMIT.maximum, _HIP_TORQUE_LIMIT.maximum, _KNEE_TORQUE_LIMIT.maximum] * LEG_COUNT), dtype=np.float64)

    def _get_target_angles_from_dict(self, preset_dict:dict):
        # Convert dict of tuples into 1D array
        return np.array([item for tup in preset_dict.values() for item in tup])

    def _send_position_preset(self, angle_targets_dict:dict):
        target_angles:LegPoseList = self._get_target_angles_from_dict(angle_targets_dict).tolist()

        position = Position(target_angles, self._get_feedforward_torques())
        self.send_position(position)

    # Presets
    # TODO: Separate positions from simulator class and store as position objects (pending creation)
    # TODO: Add in transition between sit (lay down) and stand. (Just queue up those positions for each time step?)
    def send_stand_position(self):
        self._send_position_preset(self._REAL_STAND)

    def send_lay_down_position(self):
        self._send_position_preset(self._REAL_LAY_DOWN)
