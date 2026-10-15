import numpy as np

LEG_COUNT:int = 4
JOINT_COUNT:int = LEG_COUNT * 3  # LEG_COUNT * (Abductor, Hip, Knee)

SIT_PITCH    = -0.01
SIT_HEIGHT   = 0.092
STAND_HEIGHT = 0.32
CRAWL_HEIGHT = 0.22

# Leg offsets from body origin #
#     Extracted from https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/go2/go2.xml
LEG_OFFSETS_FROM_BODY_ORIGIN = np.array([
    [ 0.1934, -0.0465, 0.0],  # FR
    [ 0.1934,  0.0465, 0.0],  # FL
    [-0.1934, -0.0465, 0.0],  # RR
    [-0.1934,  0.0465, 0.0]   # RL
], dtype=float)
