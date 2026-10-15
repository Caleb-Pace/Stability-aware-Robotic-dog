import numpy as np

from TMP_movement import compute_foot_target, STEP_FREQ, PHASE_OFFSET, STEP_LEN_X, STEP_LEN_Y

from kinematics.fk_solver import calculate_joint_positions


STAND_HIP_ORIGINS = [
    np.asarray([ -0.00707203, -0.00407884,  0.32900787 ]),
    np.asarray([ -0.00709525,  0.01593736,  0.32972751 ]),
    np.asarray([ -0.00548075,  0.0182337,   0.33178612 ]),
    np.asarray([ -0.00581754,  0.03719243,  0.33046114 ])
]

t_global = 0
while t_global <= 1:

    ly = 1.0
    lx = 0.0
    rx = 0.0

    for leg in range(4):
        ph = (STEP_FREQ * 2.0 * np.pi * t_global + PHASE_OFFSET[leg]) % (2.0 * np.pi)

        stride_x = ly * STEP_LEN_X
        stride_y = lx * STEP_LEN_Y

        hip_t, thigh_t, calf_t = compute_foot_target(
            leg, ph, stride_x, stride_y, rx
        )
        print(f" h: {hip_t}, t: {thigh_t}, c: {calf_t} | ", end="")  # TODO: Remove, for debugging
        origin_pos, _, _, foot_pos = calculate_joint_positions(STAND_HIP_ORIGINS[leg], np.asarray([hip_t, thigh_t, calf_t]), (leg % 2 == 0))
        print(f"o:{origin_pos}; f:{foot_pos}", end="")
        print()

    t_global += 0.05