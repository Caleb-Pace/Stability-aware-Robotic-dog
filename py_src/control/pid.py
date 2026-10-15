import numpy as np
from data_structures.constants import LEG_COUNT, JOINT_COUNT
from kinematics.ik_solver import _HIP_ABDUCTOR_TORQUE_LIMIT, _HIP_TORQUE_LIMIT, _KNEE_TORQUE_LIMIT


# per leg: [hip (abductor), thigh, calf]
# KP = [40.0, 40.0, 50.0]   # RL-style Go2 policies use ~20-25; IK/MPC trotting is usually 40-60
# KD = [1.0,  1.0,  1.5]    # keep much lower than 5; raise only if you see oscillation
KP = [50.0] * 3
KD = [4.0] * 3
KI = [0.0] * 3  # UNUSED
TORQUE_MAX       = [_HIP_ABDUCTOR_TORQUE_LIMIT.maximum, _HIP_TORQUE_LIMIT.maximum, _KNEE_TORQUE_LIMIT.maximum]
TORQUE_MAX_TILED = np.asarray( TORQUE_MAX * LEG_COUNT )


#     Credit: https://github.com/maanas444/go2-simulation/blob/main/mujoco/go2_PID.py
class PIDController:
    def __init__(self, kp, ki, kd, max_torque):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_torque = max_torque
        self.integral = 0.0

    def update(self, target, current, velocity, dt):
        error = target - current
        self.integral += error * dt

        output = (self.kp * error) + (self.ki * self.integral) - (self.kd * velocity)
        
        return np.clip(output, -self.max_torque, self.max_torque)

def get_pid_controllers() -> list[PIDController]:
    # 3 motors per leg
    pids = [
        PIDController(KP[i % 3], KI[i % 3], KD[i % 3], TORQUE_MAX[i % 3]) for i in range(JOINT_COUNT)
    ]
    return pids