import time  # TODO: Remove, for debugging

import numpy as np
from data_structures.gait_definition import JOINT_COUNT
from kinematics.ik_solver import _HIP_ABDUCTOR_TORQUE_LIMIT, _HIP_TORQUE_LIMIT, _KNEE_TORQUE_LIMIT


KP = [100.0, 250.0, 200.0]
KI = [5.0, 10.0, 10.0]
KD = [3.0, 6.0, 5.0]
TORQUE_MAX = [_HIP_ABDUCTOR_TORQUE_LIMIT.maximum, _HIP_TORQUE_LIMIT.maximum, _KNEE_TORQUE_LIMIT.maximum]

#     Credit: https://github.com/maanas444/go2-simulation/blob/main/mujoco/go2_PID.py
class PIDController:
    last_debug_message_time = time.time()  # TODO: Remove, for debugging

    def __init__(self, kp, ki, kd, torque_max):
        self.kp, self.ki, self.kd, self.torque_max = kp, ki, kd, torque_max
        self.integral = 0.0

    def update(self, target, pos, vel, dt):
        error = target - pos
        self.integral = np.clip(self.integral + error * dt, -10.0, 10.0)
        torque = (self.kp * error) + (self.ki * self.integral) - (self.kd * vel)
        result = float(np.clip(torque, -self.torque_max, self.torque_max))

        # TODO: Remove, for debugging
        if (time.time() - self.last_debug_message_time) * 1000 >= 500:  # ms
            print(f"[PI]      {str(int(time.time() * 1000))[-6:-2]} ({int((time.time() - self.last_debug_message_time) * 1000)} ms since last)")
            print(f"[PI]      pid.update({np.round(target, 5)} Rad, {np.round(pos, 5)} Rad, {np.round(vel, 5)} Rad/s, {np.round(dt, 5)} s);")
            print(f"[PI]           -> {np.round(result, 5)} Nm")
            self.last_debug_message_time = time.time()
        return result
    # def __init__(self, kp, ki, kd, max_torque):
    #     self.kp = kp
    #     self.ki = ki
    #     self.kd = kd
    #     self.max_torque = max_torque
    #     self.integral = 0.0

    # def update(self, target, current, velocity, dt):
    #     error = target - current
    #     self.integral += error * dt

    #     output = (self.kp * error) + (self.ki * self.integral) - (self.kd * velocity)
        
    #     return np.clip(output, -self.max_torque, self.max_torque)

def get_pid_controllers() -> list[PIDController]:
    # 3 motors per leg
    pids = [
        PIDController(KP[i % 3], KI[i % 3], KD[i % 3], TORQUE_MAX[i % 3]) for i in range(JOINT_COUNT)
    ]
    return pids
