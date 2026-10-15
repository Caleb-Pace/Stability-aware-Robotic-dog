import math
import numpy as np


# ── Robot geometry ────────────────────────────────────────────────────────────
L_THIGH = 0.213
L_CALF  = 0.213
THIGH_LIM = (-1.571,  3.491)
CALF_LIM  = (-2.723, -0.838)
HIP_LIM   = (-1.047,  1.047)

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

HIP_STAND, THIGH_STAND, CALF_STAND = 0.0, 0.662, -1.363
FOOT_Z_STAND = -(L_THIGH * math.cos(THIGH_STAND) + L_CALF * math.cos(THIGH_STAND + CALF_STAND))

# ── Controller/Gait Config ───────────────────────────────────────────────────
TRANSITION_DURATION = 1.5
STEP_FREQ   = 2.0
STEP_HEIGHT = 0.08
STEP_LEN_X  = 0.18
STEP_LEN_Y  = 0.08
# Turn stride: how far each foot deviates laterally/longitudinally for yaw.
# Front/rear legs use opposite signs to create a proper pivot.
TURN_STRIDE = 0.10

# Diagonal pairs for trotting: (FR, RL) and (FL, RR) are in phase.
# Phase offsets per leg: FR=0, FL=π, RR=π, RL=0
PHASE_OFFSET = [0.0, math.pi, math.pi, 0.0]  # FR FL RR RL

# Pygame Y-axis is typically NEGATIVE when pushed forward.
# If the robot walks backwards when you push forward, flip this to +1.
FWD_AXIS_SIGN = -1.0

# ── Index maps ────────────────────────────────────────────────────────────────
CTRL_IDX = [[0,1,2],[3,4,5],[6,7,8],[9,10,11]]
QPOS_IDX = [[10,11,12],[7,8,9],[16,17,18],[13,14,15]]
QVEL_IDX = [[ 9,10,11],[6,7,8],[15,16,17],[12,13,14]]

AXIS_LX, AXIS_LY, AXIS_RX = 0, 1, 3
BTN_A, BTN_X, BTN_Y, DEADZONE = 0, 2, 3, 0.12
_LEG_KEYS = ["FR", "FL", "RR", "RL"]


def ik(px, pz):
    r = np.clip(math.sqrt(px*px + pz*pz), 0.05, L_THIGH + L_CALF - 0.005)
    cos_c = (L_THIGH**2 + L_CALF**2 - r**2) / (2.0*L_THIGH*L_CALF)
    calf = -(math.pi - math.acos(np.clip(cos_c, -1.0, 1.0)))
    alpha = math.atan2(px, -pz)
    cos_b = (L_THIGH**2 + r**2 - L_CALF**2) / (2.0*L_THIGH*r)
    thigh = alpha + math.acos(np.clip(cos_b, -1.0, 1.0))
    return float(np.clip(thigh, *THIGH_LIM)), float(np.clip(calf, *CALF_LIM))

def lerp_pose(leg_key, t):
    s = t * t * (3.0 - 2.0 * t)
    sit, stand = REAL_SIT[leg_key], REAL_STAND[leg_key]
    return tuple(sit[i] + s * (stand[i] - sit[i]) for i in range(3))


def compute_foot_target(leg, ph, stride_x, stride_y, yaw):
    """
    Compute foot target in the sagittal plane plus hip target.

    Pivot / yaw logic
    -----------------
    A correct in-place pivot requires RIGHT-side legs to step forward and
    LEFT-side legs to step backward (for a positive/right yaw command).
    This is the differential-drive analogy: each side gets an equal-and-
    opposite longitudinal contribution, with ZERO lateral (hip) offset so
    the feet don't splay.  The previous version erroneously used the
    front/rear axis and also applied a hip offset that fought the turn.

    Trot quality
    ------------
    Hip joints are held at the nominal standing value (from REAL_STAND)
    rather than being zeroed.  Zeroing them shifts the CoM laterally on
    every step, producing the waddling artefact.  Lateral (strafe) and
    yaw hip contributions are small additive deltas on top of that bias.

    Parameters
    ----------
    leg      : int   0=FR 1=FL 2=RR 3=RL
    ph       : float current phase in [0, 2π)
    stride_x : float forward stride amplitude (signed, positive = forward)
    stride_y : float lateral stride amplitude  (signed, positive = right strafe)
    yaw      : float yaw rate command [-1..1], positive = turn right
    """
    key = _LEG_KEYS[leg]

    # Right legs get +side, left legs get -side.
    # For a right-yaw turn, right legs step forward and left legs step back.
    side_sign = 1.0 if leg in (0, 2) else -1.0   # FR, RR = right side

    # Nominal hip angle from the calibrated stand pose — keeps CoM centred.
    hip_nominal = REAL_STAND[key][0]

    # Yaw modifies the longitudinal stride per side (differential drive).
    # No lateral/hip contribution from yaw — it causes splay, not rotation.
    yaw_stride = yaw * TURN_STRIDE * side_sign
    total_stride_x = stride_x + yaw_stride

    if ph < math.pi:
        # ── SWING phase ──────────────────────────────────────────────────────
        prog    = ph / math.pi              # 0 → 1 over swing
        swing_t = math.sin(math.pi * prog) # smooth bell for vertical arc

        px = -total_stride_x / 2.0 + total_stride_x * prog
        pz =  FOOT_Z_STAND + STEP_HEIGHT * swing_t

        # Lateral foot placement: strafe only, no yaw hip splay.
        # Strafe: foot sweeps from +y/2 to -y/2 during swing.
        py_delta = stride_y / 2.0 - stride_y * prog

    else:
        # ── STANCE phase ─────────────────────────────────────────────────────
        prog = (ph - math.pi) / math.pi    # 0 → 1 over stance

        px =  total_stride_x / 2.0 - total_stride_x * prog
        pz =  FOOT_Z_STAND

        py_delta = -stride_y / 2.0 + stride_y * prog

    # Hip target = nominal offset + any lateral delta
    hip_t = hip_nominal + py_delta
    thigh_t, calf_t = ik(px, pz)

    print(f"leg{leg} @ {ph} | x: {px}; z: {pz} | ", end="")  # TODO: Remove, for debugging
    return hip_t, thigh_t, calf_t