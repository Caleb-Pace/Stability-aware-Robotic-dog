import numpy as np

from kinematics.fk_solver import calculate_joint_positions
    
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


def calculate_positions(leg_angles, is_left:bool):
    origin = np.array([0, 0, 0], dtype=np.float64)
    angles = np.array(leg_angles)

    # Calculate original positions relative to origin
    abductor_pos, hip_pos, knee_pos, foot_pos = calculate_joint_positions(origin, angles, is_left)

    # Offset vector to shift the foot to (0, 0, 0)
    offset = foot_pos

    # Shift all positions relative to foot_pos
    abductor_pos_rel_foot = abductor_pos - offset
    hip_pos_rel_foot      = hip_pos - offset
    knee_pos_rel_foot     = knee_pos - offset
    foot_pos_rel_foot     = (0.0, 0.0, 0.0)

    return abductor_pos_rel_foot, hip_pos_rel_foot, knee_pos_rel_foot, foot_pos_rel_foot

# action = _REAL_STAND
action = _REAL_LAY_DOWN

print(f"Origin")
abductor_pos, _, _, foot_pos = calculate_positions(action["FR"], False)
print(f"FR | {abductor_pos}")
abductor_pos, _, _, foot_pos = calculate_positions(action["FL"], True)
print(f"FL | {abductor_pos}")
abductor_pos, _, _, foot_pos = calculate_positions(action["RR"], False)
print(f"RR | {abductor_pos}")
abductor_pos, _, _, foot_pos = calculate_positions(action["RL"], True)
print(f"RL | {abductor_pos}")
