#!/usr/bin/env python3
import os
import time
import threading
import numpy as np

import mujoco
import mujoco.viewer

from hardware_abstraction_layer.input_layer import InputLayer
from hardware_abstraction_layer.gamepad_controller import GamepadController
from hardware_abstraction_layer.output_layer import MujocoOutput

from kinematic_controller.gaits import TROT
from kinematic_controller.gait_definition import Gait
from kinematic_controller.gait_engine import GaitEngine

XML_PATH = os.path.expanduser('~/unitree_mujoco/unitree_robots/go2/scene.xml')

# Exact poses from the working reference script
REAL_STAND = {
    "FR": ( 0.018,  0.667, -1.377),
    "FL": (-0.018,  0.663, -1.369),
    "RR": ( 0.085,  0.660, -1.353),
    "RL": (-0.082,  0.658, -1.351),
}

REAL_SIT = {
    "FR": ( 0.061,  1.236, -2.761),
    "FL": (-0.068,  1.241, -2.770),
    "RR": ( 0.383,  1.243, -2.756),
    "RL": (-0.402,  1.244, -2.758),
}

_LEG_KEYS = ["FR", "FL", "RR", "RL"]

CTRL_IDX = [[0,1,2],[3,4,5],[6,7,8],[9,10,11]]
QPOS_IDX = [[10,11,12],[7,8,9],[16,17,18],[13,14,15]]

KP, KI, KD = [100.0, 250.0, 200.0], [5.0, 10.0, 10.0], [3.0, 6.0, 5.0]
TORQUE_MAX = [23.7, 23.7, 45.43]

class SimplePID:
    def __init__(self, kp, ki, kd, torque_max):
        self.kp, self.ki, self.kd, self.torque_max = kp, ki, kd, torque_max
        self.integral = 0.0

    def update(self, target, pos, vel, dt):
        error = target - pos
        self.integral = np.clip(self.integral + error * dt, -10.0, 10.0)
        torque = (self.kp * error) + (self.ki * self.integral) - (self.kd * vel)
        return float(np.clip(torque, -self.torque_max, self.torque_max))

def lerp_pose(leg_key, t):
    s = t * t * (3.0 - 2.0 * t)
    sit, stand = REAL_SIT[leg_key], REAL_STAND[leg_key]
    return tuple(sit[i] + s * (stand[i] - sit[i]) for i in range(3))

def reset_robot_pose_standing(data):
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[2] = 0.30  # Stand height
    data.qpos[3] = 1.0   
    for leg in range(4):
        hip, thigh, calf = REAL_STAND[_LEG_KEYS[leg]]
        qp = QPOS_IDX[leg]
        data.qpos[qp[0]], data.qpos[qp[1]], data.qpos[qp[2]] = hip, thigh, calf

def main():
    print(f"[Main] Loading MuJoCo model from: {XML_PATH}")
    gait: Gait = TROT
    in_layer: InputLayer = GamepadController()
    
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    model.opt.timestep = 0.002  

    pids = [[SimplePID(KP[j], KI[j], KD[j], TORQUE_MAX[j]) for j in range(3)] for _ in range(4)]

    go2_pids = {
        "kp": KP,
        "ki": KI,
        "kd": KD,
        "torque_max": TORQUE_MAX
    }
    
    out_layer = MujocoOutput(model, data, pids=go2_pids)
    out_layer.connect()

    reset_robot_pose_standing(data)
    mujoco.mj_forward(model, data)

    engine = GaitEngine(gait, out_layer)
    clock_interval_ms = 2  
    clock_interrupt_flag = threading.Event()
    clock_thread = threading.Thread(target=engine.clock_start, args=(clock_interval_ms, clock_interrupt_flag))
    clock_thread.start()

    # Start directly in STAND state instead of SIT
    state = "STAND"
    sit_stand_t = 1.0
    TRANSITION_DURATION = 1.5  
    prev_btn_a = False
    prev_btn_y = False
    
    sim_time = 0.0
    wall_origin = time.perf_counter()
    trot_throttle = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            
            input_data = in_layer.poll()
            
            just_pressed_a = input_data.btn_a and not prev_btn_a
            just_pressed_y = input_data.btn_y and not prev_btn_y
            prev_btn_a = input_data.btn_a
            prev_btn_y = input_data.btn_y

            if just_pressed_y:
                reset_robot_pose_standing(data)
                mujoco.mj_forward(model, data)
                state, sit_stand_t = "STAND", 1.0
                print("[Main] Robot Reset to STAND")

            if just_pressed_a:
                state = "LOWERING" if state in ("STAND", "RISING") else "RISING"
                print(f"[Main] State changed to: {state}")

            has_input = (abs(input_data.left_stick.delta_x) > 0 or 
                         abs(input_data.left_stick.delta_y) > 0 or 
                         abs(input_data.right_stick.delta_x) > 0)

            if state in ("STAND", "TROT"):
                state = "TROT" if has_input else "STAND"

            target_sim = min(time.perf_counter() - wall_origin, sim_time + 0.050)
            
            while sim_time < target_sim:
                DT = model.opt.timestep

                if state == "RISING":
                    sit_stand_t = min(1.0, sit_stand_t + DT / TRANSITION_DURATION)
                    if sit_stand_t >= 1.0: state = "STAND"
                elif state == "LOWERING":
                    sit_stand_t = max(0.0, sit_stand_t - DT / TRANSITION_DURATION)
                    if sit_stand_t <= 0.0: state = "SIT"

                if state != "TROT":
                    # Handle manual sit/stand/transition poses
                    for leg in range(4):
                        leg_key = _LEG_KEYS[leg]
                        if state in ("RISING", "LOWERING"):
                            hip_t, thigh_t, calf_t = lerp_pose(leg_key, sit_stand_t)
                        elif state == "SIT":
                            hip_t, thigh_t, calf_t = REAL_SIT[leg_key]
                        elif state == "STAND":
                            hip_t, thigh_t, calf_t = REAL_STAND[leg_key]

                        qp, ci = QPOS_IDX[leg], CTRL_IDX[leg]
                        targets = [hip_t, thigh_t, calf_t]
                        for j in range(3):
                            current_p = data.qpos[qp[j]]
                            current_v = data.qvel[qp[j] - 1] 
                            data.ctrl[ci[j]] = pids[leg][j].update(targets[j], current_p, current_v, DT)
                    engine.is_moving = False
                    trot_throttle = 0.0
                else:
                    # Rate-limit input calls to match natural pacing cadence
                    trot_throttle += DT
                    if trot_throttle >= 0.04:  
                        engine.input(input_data, 1)
                        trot_throttle = 0.0
                    engine.is_moving = True

                mujoco.mj_step(model, data)
                sim_time += DT

            viewer.sync()

    clock_interrupt_flag.set()
    clock_thread.join()

if __name__ == "__main__":
    main()